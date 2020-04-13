/*-
 * BSD LICENSE
 *
 * Copyright(c) 2017-2019 Xilinx, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <sys/fcntl.h>
#include <rte_memzone.h>
#include <rte_string_fns.h>
#include <rte_ethdev_pci.h>
#include <rte_malloc.h>
#include <rte_dev.h>
#include <rte_pci.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_alarm.h>
#include <rte_cycles.h>
#include <unistd.h>
#include <string.h>

#include "qdma.h"
#include "version.h"
#include "qdma_access.h"
#include "qdma_access_export.h"
#include "qdma_mbox.h"
#include "qdma_xcmac.h"
#include "qep_version.h"

/* Register offsets for MAC address */
#define QEP_USR_RX_META_LOW_R (0x200000)
#define QEP_USR_RX_META_HIGH_R (0x201000)

#define BITS_IN_BYTE (8)
#define BITS_IN_WORD (sizeof(uint32_t) * BITS_IN_BYTE)

/* QDMA error poll frequency in microseconds */
/* Poll for QDMA errors every 1 second */
#define QDMA_ERROR_POLL_FRQ (1000000)

static void qdma_device_attributes_get(struct rte_eth_dev *dev);

/* Poll for any QDMA errors */
static void qdma_check_errors(void *arg)
{
	struct qdma_pci_dev *qdma_dev;
	qdma_dev = ((struct rte_eth_dev *)arg)->data->dev_private;
	qdma_dev->hw_access->qdma_hw_error_process(arg);
	rte_eal_alarm_set(QDMA_ERROR_POLL_FRQ, qdma_check_errors, arg);
}

/*
 * The set of PCI devices this driver supports
 */
static struct rte_pci_id qdma_pci_id_tbl[] = {
#define RTE_PCI_DEV_ID_DECL_XNIC(vend, dev) {RTE_PCI_DEVICE(vend, dev)},
#ifndef PCI_VENDOR_ID_XILINX
#define PCI_VENDOR_ID_XILINX 0x10ee
#endif

	RTE_PCI_DEV_ID_DECL_XNIC(PCI_VENDOR_ID_XILINX, 0x7002)	/** PF 2 */
	RTE_PCI_DEV_ID_DECL_XNIC(PCI_VENDOR_ID_XILINX, 0x5016)	/** PF 2 */

	{ .vendor_id = 0, /* sentinel */ },
};

static void qdma_device_attributes_get(struct rte_eth_dev *dev)
{
	struct qdma_pci_dev *qdma_dev;

	qdma_dev = (struct qdma_pci_dev *)dev->data->dev_private;
	qdma_dev->hw_access->qdma_get_device_attributes(dev,
			&qdma_dev->dev_cap);

	/* Check DPDK configured queues per port */
	if (qdma_dev->dev_cap.num_qs > RTE_MAX_QUEUES_PER_PORT)
		qdma_dev->dev_cap.num_qs = RTE_MAX_QUEUES_PER_PORT;

	PMD_DRV_LOG(INFO, "qmax = %d, mm %d, st %d.\n",
	qdma_dev->dev_cap.num_qs, qdma_dev->dev_cap.mm_en,
	qdma_dev->dev_cap.st_en);
}

static inline uint8_t pcie_find_cap(const struct rte_pci_device *pci_dev,
					uint8_t cap)
{
	uint8_t pcie_cap_pos = 0;
	uint8_t pcie_cap_id = 0;

	if (rte_pci_read_config(pci_dev, &pcie_cap_pos, sizeof(uint8_t),
				PCI_CAPABILITY_LIST) < 0) {
		PMD_DRV_LOG(ERR, "PCIe config space read failed..\n");
		return 0;
	}

	if (pcie_cap_pos < 0x40)
		return 0;

	while (pcie_cap_pos >= 0x40) {
		pcie_cap_pos &= ~3;

		if (rte_pci_read_config(pci_dev, &pcie_cap_id, sizeof(uint8_t),
					pcie_cap_pos + PCI_CAP_LIST_ID) < 0) {
			PMD_DRV_LOG(ERR, "PCIe config space read failed..\n");
			goto ret;
		}

		if (pcie_cap_id == 0xff)
			break;

		if (pcie_cap_id == cap)
			return pcie_cap_pos;

		if (rte_pci_read_config(pci_dev, &pcie_cap_pos, sizeof(uint8_t),
					pcie_cap_pos + PCI_CAP_LIST_NEXT) < 0) {
			PMD_DRV_LOG(ERR, "PCIe config space read failed..\n");
			goto ret;
		}
	}

ret:
	return 0;
}

static void pcie_perf_enable(const struct rte_pci_device *pci_dev)
{
	uint16_t value;
	uint8_t pcie_cap_pos = pcie_find_cap(pci_dev, PCI_CAP_ID_EXP);

	if (!pcie_cap_pos)
		return;

	if (pcie_cap_pos > 0) {
		if (rte_pci_read_config(pci_dev, &value, sizeof(uint16_t),
					 pcie_cap_pos + PCI_EXP_DEVCTL) < 0) {
			PMD_DRV_LOG(ERR, "PCIe config space read failed..\n");
			return;
		}

		value |= (PCI_EXP_DEVCTL_EXT_TAG | PCI_EXP_DEVCTL_RELAX_EN);

		if (rte_pci_write_config(pci_dev, &value, sizeof(uint16_t),
					   pcie_cap_pos + PCI_EXP_DEVCTL) < 0) {
			PMD_DRV_LOG(ERR, "PCIe config space write failed..\n");
			return;
		}
	}
}

#ifndef QEP_USE_DEFAULT_MAC_ADDR
static void read_mac_addr(struct qdma_pci_dev *dev, uint8_t *mac_addr)
{
	uint64_t addr;
	uint32_t reg_val;
	int i;

	reg_val = qdma_read_reg(
			(uint64_t)(dev->bar_addr[dev->user_bar_idx]) +
			QEP_USR_RX_META_LOW_R);
	addr = reg_val;

	reg_val = qdma_read_reg(
			(uint64_t)(dev->bar_addr[dev->user_bar_idx]) +
			QEP_USR_RX_META_HIGH_R);
	addr |= ((uint64_t)reg_val << BITS_IN_WORD);

	for (i = 0; i < ETHER_ADDR_LEN; i++)
		mac_addr[ETHER_ADDR_LEN - 1 - i] =
			(uint8_t)((addr >> (BITS_IN_BYTE * i)) & 0xFF);
}

static int is_mac_addr_zero(const uint8_t *mac_addr)
{
	int i;
	for (i = 0; i < ETHER_ADDR_LEN; i++) {
		if (mac_addr[i] != 0x00)
			return 0;
	}
	return -1;
}
#endif //QEP_USE_DEFAULT_MAC_ADDR

/**
 * DPDK callback to register a PCI device.
 *
 * This function creates an Ethernet device for each port of a given
 * PCI device.
 *
 * @param[in] dev
 *   Pointer to Ethernet device structure.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */
int qdma_eth_dev_init(struct rte_eth_dev *dev)
{
	struct qdma_pci_dev *dma_priv;
	uint8_t *baseaddr;
	int i, idx, ret, qbase;
	struct rte_pci_device *pci_dev;
	uint32_t total_q;
	uint16_t num_vfs;
#ifdef QEP_USE_DEFAULT_MAC_ADDR
	uint8_t mac_addr[ETHER_ADDR_LEN] = {0x00, 0x5D, 0x03, 0x00, 0x00, 0x02};
#endif

	/* sanity checks */
	if (dev == NULL)
		return -EINVAL;
	if (dev->data == NULL)
		return -EINVAL;
	if (dev->data->dev_private == NULL)
		return -EINVAL;

	pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	if (pci_dev == NULL)
		return -EINVAL;

	/* for secondary processes, we don't initialise any further as primary
	 * has already done this work.
	 */
	if (rte_eal_process_type() != RTE_PROC_PRIMARY) {
		qdma_dev_ops_init(dev);
		return 0;
	}

	/* Init system & device */
	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;
	dma_priv->is_vf = 0;
	dma_priv->is_master = 0;
	dma_priv->vf_online_count = 0;
	dma_priv->timer_count = DEFAULT_TIMER_CNT_TRIG_MODE_TIMER;
	dma_priv->stmn_c2h_buf_size = -1;
	dma_priv->queue_base = DEFAULT_QUEUE_BASE;
	dma_priv->rsfec = 1;
	dma_priv->trigger_mode = RTE_PMD_QDMA_TRIG_MODE_USER_TIMER;

	if (dma_priv->trigger_mode == RTE_PMD_QDMA_TRIG_MODE_USER_TIMER_COUNT)
		dma_priv->timer_count = DEFAULT_TIMER_CNT_TRIG_MODE_COUNT_TIMER;

	dma_priv->en_desc_prefetch = 0; //Keep prefetch default to 0
	dma_priv->cmpt_desc_len = DEFAULT_QDMA_CMPT_DESC_LEN;
	dma_priv->c2h_bypass_mode = RTE_PMD_QDMA_RX_BYPASS_SIMPLE;
	dma_priv->h2c_bypass_mode = RTE_PMD_QDMA_TX_BYPASS_ENABLE;

	dma_priv->config_bar_idx = DEFAULT_PF_CONFIG_BAR;
	dma_priv->bypass_bar_idx = BAR_ID_INVALID;
	dma_priv->user_bar_idx = BAR_ID_INVALID;
	qep_flow_init(dma_priv);

	/* Check and handle device devargs*/
	if (qdma_check_kvargs(dev->device->devargs, dma_priv)) {
		PMD_DRV_LOG(INFO, "devargs failed\n");
		return -EINVAL;
	}

	/* Store BAR address and length of Config BAR */
	baseaddr = (uint8_t *)
			pci_dev->mem_resource[dma_priv->config_bar_idx].addr;
	dma_priv->bar_addr[dma_priv->config_bar_idx] = baseaddr;

	/*Assigning QDMA access layer function pointers based on the HW design*/
	dma_priv->hw_access = rte_zmalloc("hwaccess",
					sizeof(struct qdma_hw_access), 0);
	if (dma_priv->hw_access == NULL)
		return -ENOMEM;

	idx = qdma_hw_access_init(dev, dma_priv->is_vf, dma_priv->hw_access);
	if (idx < 0) {
		rte_free(dma_priv->hw_access);
		return -EINVAL;
	}

	idx = qdma_get_hw_version(dev);
	if (idx < 0) {
		rte_free(dma_priv->hw_access);
		return -EINVAL;
	}

	idx = qdma_identify_bars(dev);
	if (idx < 0) {
		rte_free(dma_priv->hw_access);
		return -EINVAL;
	}

	/* Store BAR address and length of User BAR */
	if (dma_priv->user_bar_idx >= 0) {
		baseaddr = (uint8_t *)
			    pci_dev->mem_resource[dma_priv->user_bar_idx].addr;
		dma_priv->bar_addr[dma_priv->user_bar_idx] = baseaddr;
	}

	PMD_DRV_LOG(INFO, "QDMA device driver probe:");

	/* allocate space for a single Ethernet MAC address */
	dev->data->mac_addrs = rte_zmalloc("qdma", ETHER_ADDR_LEN * 1, 0);
	if (dev->data->mac_addrs == NULL) {
		PMD_DRV_LOG(ERR, "Cannot allocate memory to store MAC addr\n");
		rte_free(dma_priv->hw_access);
		return -ENOMEM;
	}

#ifdef QEP_USE_DEFAULT_MAC_ADDR
	rte_memcpy(&(dev->data->mac_addrs[0].addr_bytes[0]),
		mac_addr, ETHER_ADDR_LEN);
#else
	/* Read MAC address from device */
	read_mac_addr(dma_priv, &(dev->data->mac_addrs[0].addr_bytes[0]));
	if (is_mac_addr_zero(&(dev->data->mac_addrs[0].addr_bytes[0])) < 0) {
		PMD_DRV_LOG(ERR, "Invalid MAC address read from the device\n");
		rte_free(dma_priv->hw_access);
		rte_free(dev->data->mac_addrs);
		return -EINVAL;
	}
#endif //QEP_USE_DEFAULT_MAC_ADDR

	qdma_dev_ops_init(dev);

	/* Getting the device attributes from the Hardware */
	qdma_device_attributes_get(dev);

	/* Create master resource node for queue management on the given
	 * bus number. Node will be created only once per bus number.
	 */
	qbase = DEFAULT_QUEUE_BASE;
	total_q = QDMA_QUEUES_NUM_MAX;

	ret = qdma_master_resource_create(pci_dev->addr.bus, qbase,
				    total_q);
	if (ret == -QDMA_ERR_NO_MEM) {
		rte_free(dma_priv->hw_access);
		rte_free(dev->data->mac_addrs);
		return -ENOMEM;
	}

	dma_priv->hw_access->qdma_get_function_number(dev,
			&dma_priv->func_id);
	PMD_DRV_LOG(INFO, "PF function ID: %d", dma_priv->func_id);

	/* CSR programming is done once per given board or bus number,
	 * done by the master PF
	 */
	if (ret == QDMA_SUCCESS) {
		RTE_LOG(INFO, PMD, "QDMA PMD VERSION: %s\n", QDMA_PMD_VERSION);
		RTE_LOG(INFO, PMD, "QEP PMD VERSION: %s\n", QEP_PMD_VERSION);
		dma_priv->hw_access->qdma_set_default_global_csr(dev);
		for (i = 0; i < dma_priv->dev_cap.mm_channel_max; i++) {
			if (dma_priv->dev_cap.mm_en) {
				/* Enable MM C2H Channel */
				dma_priv->hw_access->qdma_mm_channel_conf(dev,
							i, 1, 1);
				/* Enable MM H2C Channel */
				dma_priv->hw_access->qdma_mm_channel_conf(dev,
							i, 0, 1);
			} else {
				/* Disable MM C2H Channel */
				dma_priv->hw_access->qdma_mm_channel_conf(dev,
							i, 1, 0);
				/* Disable MM H2C Channel */
				dma_priv->hw_access->qdma_mm_channel_conf(dev,
							i, 0, 0);
			}
		}

		ret = xcmac_setup(dev);
		if (ret != 0) {
			PMD_DRV_LOG(ERR, "\nXcmac setup failed");
			rte_free(dma_priv->hw_access);
			rte_free(dev->data->mac_addrs);
			return ret;
		}

		dma_priv->hw_access->qdma_init_ctxt_memory(dev);
		dma_priv->hw_access->qdma_hw_error_enable(dev, QDMA_ERRS_ALL);
		rte_eal_alarm_set(QDMA_ERROR_POLL_FRQ, qdma_check_errors,
							(void *)dev);
		dma_priv->is_master = 1;
	}

	/*
	 * Create an entry for the device in board list if not already
	 * created
	 */
	ret = qdma_dev_entry_create(pci_dev->addr.bus, dma_priv->func_id);
	if ((ret != QDMA_SUCCESS) &&
		(ret != -QDMA_ERR_RM_DEV_EXISTS)) {
		PMD_DRV_LOG(ERR, "PF-%d(DEVFN) qdma_dev_entry_create failed: %d\n",
			    dma_priv->func_id, ret);
		rte_free(dma_priv->hw_access);
		rte_free(dev->data->mac_addrs);
		return -ENOMEM;
	}

	pcie_perf_enable(pci_dev);
	if (dma_priv->dev_cap.mailbox_en && pci_dev->max_vfs)
		qdma_mbox_init(dev);

	num_vfs = pci_dev->max_vfs;
	if (num_vfs) {
		dma_priv->vfinfo = rte_zmalloc("vfinfo",
				sizeof(struct qdma_vf_info) * num_vfs, 0);
		if (dma_priv->vfinfo == NULL) {
			PMD_DRV_LOG(ERR, "Cannot allocate memory for private VF info\n");
			rte_free(dma_priv->hw_access);
			rte_free(dev->data->mac_addrs);
			return -ENOMEM;
		}

		/* Mark all VFs with invalid function id mapping*/
		for (i = 0; i < num_vfs; i++)
			dma_priv->vfinfo[i].func_id = QDMA_FUNC_ID_INVALID;
	}

	return 0;
}

/**
 * DPDK callback to deregister PCI device.
 *
 * @param[in] dev
 *   Pointer to Ethernet device structure.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */
int qdma_eth_dev_uninit(struct rte_eth_dev *dev)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct qdma_mbox_msg *m = NULL;
	int i, rv;

	/* only uninitialize in the primary process */
	if (rte_eal_process_type() != RTE_PROC_PRIMARY)
		return -EPERM;

	if (qdma_dev->vf_online_count) {
		for (i = 0; i < pci_dev->max_vfs; i++) {
			if (qdma_dev->vfinfo[i].func_id == QDMA_FUNC_ID_INVALID)
				continue;

			m = qdma_mbox_msg_alloc();
			if (!m)
				return -ENOMEM;

			qdma_mbox_compose_pf_offline(m->raw_data,
					qdma_dev->func_id,
					qdma_dev->vfinfo[i].func_id);
			rv = qdma_mbox_msg_send(dev, m, 0);
			if (rv < 0)
				PMD_DRV_LOG(ERR, "Send bye failed from PF:%d to VF:%d\n",
					qdma_dev->func_id,
					qdma_dev->vfinfo[i].func_id);
		}

		PMD_DRV_LOG(INFO,
			"%s: Wait till all VFs shutdown for PF-%d(DEVFN)\n",
			__func__, qdma_dev->func_id);

		i = 0;
		while (i < SHUTDOWN_TIMEOUT) {
			if (!qdma_dev->vf_online_count) {
				PMD_DRV_LOG(INFO,
					"%s: VFs shutdown completed for PF-%d(DEVFN)\n",
					__func__, qdma_dev->func_id);
				break;
			}
			rte_delay_ms(1);
			i++;
		}

		if (i >= SHUTDOWN_TIMEOUT) {
			PMD_DRV_LOG(ERR, "%s: Failed VFs shutdown for PF-%d(DEVFN)\n",
				__func__, qdma_dev->func_id);
		}
	}

	if (qdma_dev->dev_configured)
		qdma_dev_close(dev);

	if (qdma_dev->dev_cap.mailbox_en && pci_dev->max_vfs)
		qdma_mbox_uninit(dev);

	/* cancel pending polls*/
	if (qdma_dev->is_master)
		rte_eal_alarm_cancel(qdma_check_errors, (void *)dev);

	/* Remove the device node from the board list */
	qdma_dev_entry_destroy(pci_dev->addr.bus, qdma_dev->func_id);
	qdma_master_resource_destroy(pci_dev->addr.bus);

	dev->dev_ops = NULL;
	dev->rx_pkt_burst = NULL;
	dev->tx_pkt_burst = NULL;
	dev->data->nb_rx_queues = 0;
	dev->data->nb_tx_queues = 0;
	qep_flow_uninit(qdma_dev);

	if (qdma_dev->vfinfo != NULL) {
		rte_free(qdma_dev->vfinfo);
		qdma_dev->vfinfo = NULL;
	}

	if (dev->data->mac_addrs != NULL) {
		rte_free(dev->data->mac_addrs);
		dev->data->mac_addrs = NULL;
	}

	if (qdma_dev->q_info != NULL) {
		rte_free(qdma_dev->q_info);
		qdma_dev->q_info = NULL;
	}

	if (qdma_dev->hw_access != NULL) {
		rte_free(qdma_dev->hw_access);
		qdma_dev->hw_access = NULL;
	}
	return 0;
}

static int eth_qdma_pci_probe(struct rte_pci_driver *pci_drv __rte_unused,
				struct rte_pci_device *pci_dev)
{
	return rte_eth_dev_pci_generic_probe(pci_dev,
						sizeof(struct qdma_pci_dev),
						qdma_eth_dev_init);
}

/* Detach a ethdev interface */
static int eth_qdma_pci_remove(struct rte_pci_device *pci_dev)
{
	return rte_eth_dev_pci_generic_remove(pci_dev, qdma_eth_dev_uninit);
}

static struct rte_pci_driver rte_qep_pmd = {
	.id_table = qdma_pci_id_tbl,
	.drv_flags = RTE_PCI_DRV_NEED_MAPPING,
	.probe = eth_qdma_pci_probe,
	.remove = eth_qdma_pci_remove,
};

bool
is_pf_device_supported(struct rte_eth_dev *dev)
{
	if (strcmp(dev->device->driver->name, rte_qep_pmd.driver.name))
		return false;

	return true;
}

bool is_qdma_supported(struct rte_eth_dev *dev)
{
	bool is_pf, is_vf;

	is_pf = is_pf_device_supported(dev);
	is_vf = is_vf_device_supported(dev);

	if (!is_pf && !is_vf)
		return false;

	return true;
}

bool is_dev_qep_supported(struct rte_eth_dev *dev)
{
	bool is_pf, is_vf;

	is_pf = is_pf_device_supported(dev);
	is_vf = is_vf_device_supported(dev);

	if (!is_pf && !is_vf)
		return false;

	return true;
}

RTE_PMD_REGISTER_PCI(net_qep, rte_qep_pmd);
RTE_PMD_REGISTER_PCI_TABLE(net_qep, qdma_pci_id_tbl);
