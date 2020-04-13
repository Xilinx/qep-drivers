/*
 * Copyright (c) 2019 Xilinx, Inc.
 * All rights reserved.
 *
 *This source code is free software; you can redistribute it and/or modify it
 *under the terms and conditions of the GNU General Public License,
 *version 2, as published by the Free Software Foundation.
 *
 *This program is distributed in the hope that it will be useful, but WITHOUT
 *ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *more details.
 *
 *The full GNU General Public License is included in this distribution in
 *the file called "COPYING".
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/errno.h>
#include <linux/aer.h>
#include <linux/types.h>
#include <linux/rtnetlink.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/debugfs.h>
#include <net/busy_poll.h>
#include "qep.h"
#include "qep_stmn.h"

/* Module parameter debug which enables debug prints based on levels */
static int debug = -1;
module_param(debug, int, 0660);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

/* Module parameter loopback_en which enables CMAC loopback */
static int loopback_en = QEP_LOOPBACK_EN;
module_param(loopback_en, int, 0660);

/* PCI id for devices*/
static const struct pci_device_id qep_pci_ids[] = {
#if defined(QAP_DESIGN) || defined(QAP_STMN_DESIGN)
	{ PCI_DEVICE(0x10ee, 0x903f) },
#else
	{ PCI_DEVICE(0x10ee, 0x7002) },
	{ PCI_DEVICE(0x10ee, 0x5016) },
#endif
	{0}
};
MODULE_DEVICE_TABLE(pci, qep_pci_ids);

/* Enable QDMA poll mode instead of interrupt mode*/
static unsigned short int poll_mode;

/* Enable Burst Mode in Classifier logic */
static unsigned short int clcr_qburst = 3;

/* Total QEP Device Count in the system */
static u8 qep_device_count;

/* Enable QDMA Descriptor bypass mode */
#ifdef QAP_DESIGN
static bool c2h_byp_mode;
static bool h2c_byp_mode;
#else
static bool c2h_byp_mode = 1;
static bool h2c_byp_mode = 1;
#endif

#define QEP_IPV6 6
#define QEP_L4_UDP_LEN 2
#define QEP_CMAC_RST_MAX_TRY 1

/* Time in jiffies to conclude transmitter timeout. */
#define QEP_TX_TIMEOUT (1*HZ)

#define qep_dbg_csum_str(netdev, msg, csum)   qep_dbg(netdev, \
	"%s l4:%d l4_bad:%d l4_hdr:%d l4_off:%d " \
	" l3:%d l3_bad:%d l3_hdr:%d l3_off:%d " \
	"frag:%d udp:%d tcp:%d ipv6:%d ipv4:%d vlan:%d", \
	msg, csum.hw_l4_chksum_valid, csum.hw_l4_chksum_bad, \
	csum.l4_hdr_dw, csum.l4_offset, csum.hw_l3_chksum_valid, \
	csum.hw_l3_chksum_bad, csum.l3_hdr_dw, csum.l3_offset, \
	csum.is_ipfrag,  csum.is_udp, csum.is_tcp, csum.is_ipv6, \
	csum.is_ipv4, csum.is_vlan) \

enum qep_design_support {
	QEP,
	QAP,
	QAP_STMN,
	QEP_DESIGN_MAX
};

static const char *design_name[QEP_DESIGN_MAX] = {
	"QEP design",
	"QAP design",
	"QAP_STMN design"
};

/*****************************************************************************/
static int qep_tx_done(struct qdma_request *, unsigned int bytes_done, int err);
static int qep_rx_poll(struct napi_struct *napi, int quota);
static int qep_rx_pkt_process(unsigned long qhndl, unsigned long quld,
				 unsigned int len, unsigned int sgcnt,
				 struct qdma_sw_sg *sgl, void *udd);
static void qep_isr_tx_bottom_half(unsigned long data);
static void qep_isr_tx_tophalf(unsigned long qhndl, unsigned long uld);
static void qep_isr_rx_tophalf(unsigned long qhndl, unsigned long uld);
static int qep_cmac_restart(struct qep_priv *xpriv);
/*****************************************************************************/

void stmn_reg_write(void *dev_hndl, uint32_t reg_offst, uint32_t val)
{
	void __iomem *base = ((struct qep_priv *)dev_hndl)->tm_dev.stm_regs;

	writel(val, (void __iomem *)((u64)base + reg_offst));
}

uint32_t stmn_reg_read(void *dev_hndl, uint32_t reg_offst)
{
	u32 val;
	void __iomem *base = ((struct qep_priv *)dev_hndl)->tm_dev.stm_regs;

	val = readl((void __iomem *)((u64)base + reg_offst));
	return val;
}


/* Read CMAC link status from HW */
enum cmac_link_state qep_cmac_link_status(struct xcmac *inst)
{
	int ret = 0;
	struct xcmac_rx_lane_am_status lane_status;

	ret = xcmac_get_rx_lane_status(inst, &lane_status);
	if (ret != 0) {
		pr_err("%s: xcmac_get_rx_lane_status() failed with status %d\n",
		__func__, ret);
		return CMAC_LINK_READ_ERROR;
	}

	if (lane_status.aligned == true)
		return CMAC_LINK_UP;

	return CMAC_LINK_DOWN;
}

/*
 * This function polls CMAC link status in the background for
 * every QEP_LINK_CHECK_INTERVAL(ms)
 */
static int qep_thread_monitor_link(void *th_arg)
{
	int retry = 0;
	struct qep_priv *xpriv;
	struct net_device *netdev = (struct net_device *)th_arg;
	enum cmac_link_state link_state = CMAC_LINK_DOWN;
	int rst_try = 0;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv->lnk_state = CMAC_LINK_DOWN;
	while (!kthread_should_stop()) {
		wait_event_interruptible(xpriv->link_mon_needed,
			((xpriv->dev_state == QEP_DEV_STATE_UP) ||
			(xpriv->dev_state == QEP_DEV_STATE_EXIT)));

		/* return if interface is down or driver is to exit*/
		spin_lock(&xpriv->lnk_state_lock);
		if (xpriv->dev_state != QEP_DEV_STATE_UP) {
			spin_unlock(&xpriv->lnk_state_lock);
			link_state = CMAC_LINK_DOWN;
			xpriv->lnk_state = CMAC_LINK_DOWN;
			rst_try = 0;
			qep_err(link, "%s: Device Down\n", netdev->name);
			continue;
		}

		retry = 100;
		while (retry-- && !kthread_should_stop()) {
			link_state = qep_cmac_link_status(
					&xpriv->cmac_dev);
			if ((link_state == CMAC_LINK_UP) &&
				(xpriv->lnk_state == link_state))
				goto reloop;
			usleep_range(5000, 10000);
		}
		/* reset the cmac to try up the HW link */
		if (link_state == xpriv->lnk_state) {
			qep_err(link, "No Link");
			if ((rst_try < QEP_CMAC_RST_MAX_TRY) &&
				(link_state == CMAC_LINK_DOWN)) {
				qep_warn(link, "Link Retry\n");
				rst_try++;
				qep_cmac_restart(xpriv);
			}
			goto reloop;
		} else if (link_state == CMAC_LINK_UP &&
				!netif_carrier_ok(netdev)) {
			xpriv->lnk_state = link_state;
			/* start up the transmission queue */
			netif_tx_start_all_queues(xpriv->netdev);
			netif_carrier_on(netdev);
			rst_try = 0;
			qep_info(link, "Link Up\n");
		} else {
			netif_carrier_off(netdev);
			xpriv->lnk_state = link_state;
			qep_err(link, "Link Down\n");
		}
reloop:
		spin_unlock(&xpriv->lnk_state_lock);
		if (kthread_should_stop())
			break;
		msleep(QEP_LINK_CHECK_INTERVAL);
	}

	qep_info(link, "%s: Link status thread exiting\n", __func__);

	return 0;
}

/* This function starts link monitoring thread */
static int qep_thread_start(struct qep_priv *xpriv)
{
	if (xpriv->link_thread) {
		pr_warn("%s: Link thread is already present\n",
			 __func__);
		return 0;
	}

	xpriv->link_thread = kthread_create(qep_thread_monitor_link,
					xpriv->netdev, "qep_link_mon%d",
					qep_device_count);
	qep_device_count++;

	if (!xpriv->link_thread) {
		pr_warn("%s: link monitor thread creation failed\n",
			__func__);
		return -EINVAL;
	}
	/* Increments usage counter. */
	get_task_struct(xpriv->link_thread);
	wake_up_process(xpriv->link_thread);
	pr_info("%s: link monitor thread created\n", __func__);

	return 0;
}

/* This function stops link monitoring thread */
static void qep_thread_stop(struct qep_priv *xpriv)
{
	enum qep_dev_state last_state;

	if (!xpriv->link_thread) {
		qep_warn(link, "%s: link monitor thread is not present\n",
			 __func__);
		return;
	}

	/* Stop link thread if it is running */
	spin_lock(&xpriv->lnk_state_lock);
	last_state = xpriv->dev_state;
	xpriv->dev_state = QEP_DEV_STATE_EXIT;
	spin_unlock(&xpriv->lnk_state_lock);

	wake_up_interruptible(&xpriv->link_mon_needed);
	kthread_stop(xpriv->link_thread);
	put_task_struct(xpriv->link_thread);

	xpriv->link_thread = NULL;
	xpriv->dev_state = last_state;
	qep_info(link, "%s: link monitor thread stopped\n", __func__);
}

enum clcr_mode { CLCR_FIXED_MODE = 0,
				CLCR_ROUND_ROBIN,
				CLCR_LOOKUP_MODE};

struct clcr_fixed_mode {
	u32 qnum;
};

struct clcr_round_robin {
	u32 num_queue;
	u32 qbase;
};

struct clcr_config {
	enum clcr_mode mode;
	union {
		struct clcr_fixed_mode fixed;
		struct clcr_round_robin rr;
		struct clcr_fixed_mode lookup;
	};
};

/* This function configure Classifier Block */
static int qep_clcr_setup(struct qep_priv *xpriv, struct clcr_config *config)
{
	u32 reg_val, qbase = 0;
	void __iomem *io_addr;

	io_addr = (void __iomem *)((u64)xpriv->bar_base +
			QEP_DMA_USER_C2H_BASE);
	reg_val = readl((void __iomem *)((u64)io_addr +
			QEP_DMA_USER_C2H_CLCR_OFFSET));

	if (config->mode == CLCR_FIXED_MODE) {
		reg_val = reg_val & (~QEP_DMA_USER_C2H_QCTRL_MASK);

		reg_val = reg_val & (~QEP_DMA_USER_C2H_QNUM_MASK);
		reg_val = reg_val | (config->fixed.qnum - 1);
		qbase = 0;
	} else if (config->mode == CLCR_ROUND_ROBIN) {
		reg_val = reg_val & (~QEP_DMA_USER_C2H_QCTRL_MASK);
		reg_val = reg_val | (1 << QEP_DMA_USER_C2H_QCTRL_BIT);

		reg_val = reg_val & (~QEP_DMA_USER_C2H_QNUM_MASK);
		reg_val = reg_val | (config->rr.num_queue - 1);
		reg_val &= ~(QEP_DMA_USER_C2H_QBURST_MASK
				<< QEP_DMA_USER_C2H_QBURST_BIT);
		if (clcr_qburst)
			reg_val |= (clcr_qburst <<
					QEP_DMA_USER_C2H_QBURST_BIT);
		qbase = config->rr.qbase;
	} else if (config->mode == CLCR_LOOKUP_MODE) {

		reg_val = reg_val & (~QEP_DMA_USER_C2H_QCTRL_MASK);
		reg_val = reg_val | (0x2 << QEP_DMA_USER_C2H_QCTRL_BIT);

		reg_val = reg_val & (~QEP_DMA_USER_C2H_QNUM_MASK);
		reg_val = reg_val | (config->lookup.qnum - 1);
		qbase = 0;
	} else
		return -EINVAL;

	writel(reg_val, (void __iomem *)((u64)io_addr +
			QEP_DMA_USER_C2H_CLCR_OFFSET));

	/* set the qbase */
	writel(qbase, (void __iomem *)((u64)io_addr +
			QEP_DMA_USER_C2H_CLCR2_OFFSET));

	qep_dbg(xpriv->netdev, "qctl:%ux qbase:%ux\n",
			readl((void __iomem *)((u64)io_addr +
					QEP_DMA_USER_C2H_CLCR_OFFSET)),
			readl((void __iomem *)((u64)io_addr +
					QEP_DMA_USER_C2H_CLCR2_OFFSET)));
	return 0;
}

/* This function resets CMAC CTL */
int qep_cmac_ctrl_reset(struct qep_priv *xpriv)
{
	int ret = 0, retry = 20;
	void __iomem *io_addr =
			(void __iomem *)((u64)xpriv->bar_base +
					QEP_BASE_OFFSET);

	/* cmac base reset */
	writel(1, (void __iomem *)((u64)io_addr +
			QEP_BASE_CMAC_RESET_CONTROL_OFFSET));
	msleep(20);
	writel(0, (void __iomem *)((u64)io_addr +
			QEP_BASE_CMAC_RESET_CONTROL_OFFSET));

	msleep(100);

	/* cmac ctl reset */
	io_addr = (void __iomem *)((u64)xpriv->bar_base +
			QEP_CMAC_CTRL_BASE);

	/* TODO: Create macros for value */
	writel(0x133, (void __iomem *)((u64)io_addr +
				QEP_CMAC_CTRL_RST_OFFSET));

	msleep(100);

	writel(0, (void __iomem *)((u64)io_addr +
			QEP_CMAC_CTRL_RST_OFFSET));

	while (retry--) {
		msleep(500);
		ret = readl((void __iomem *)((u64)io_addr +
				QEP_CMAC_CTRL_ERR_OFFSET));
		if (ret == 0) {
			qep_dbg(xpriv->netdev, "%s: CMAC Reset done\n",
				__func__);
			break;
		}
		qep_info(hw, "%s: CMAC Reset is not done. Retrying\n",
			 __func__);
	}
	if (retry <= 0) {
		qep_err(hw, "%s: CMAC Reset is not done\n", __func__);
		return -EINVAL;
	}

	return 0;
}

/* This function enables CMAC loopback */
static void qep_cmac_loopback_en(struct qep_priv *xpriv, bool en)
{
	void __iomem *io_addr;
	int val = 0x492;


	io_addr = (void __iomem *)((u64)xpriv->bar_base +
			QEP_CMAC_CTRL_BASE +
			QEP_CMAC_CTRL_LOOPBACK);

	if (en)
		writel(val, io_addr);
	else
		writel(0, io_addr);
}


/* This function resets USER logic */
static int qep_reset_all(struct qep_priv *xpriv)
{
	struct xcmac *inst = &xpriv->cmac_dev;
	int ret = 0;

	ret = qep_cmac_ctrl_reset(xpriv);
	if (ret) {
		qep_err(hw, "%s: CMAC reset failed\n", __func__);
		return -EINVAL;
	}

	ret = xcmac_initialize(
		inst, ((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE));
	if (ret != 0) {
		qep_err(hw, "%s: Error in xcmac_initialize\n", __func__);
		return ret;
	}

	ret = xcmac_reset_gt(inst);
	if (ret != 0) {
		qep_err(hw, "%s: Error in resetting CMAC GT\n", __func__);
		return ret;
	}

	ret = xcmac_reset_core(inst, XCMAC_CORE_BOTH);
	if (ret != 0) {
		qep_err(hw, "%s: Error in resetting CMAC cores\n", __func__);
		return ret;
	}

	return ret;
}

/* This function initializes and sets up CMAC IP */
static int qep_cmac_setup(struct qep_priv *xpriv)
{
	int ret = 0;
	unsigned int cmac_mtu;
	void __iomem *io_addr;
	struct xcmac *inst = &xpriv->cmac_dev;

	/* Configure RS-FEC */
	ret = xcmac_config_rsfec(inst, xpriv->rs_fec_en);
	if (ret != 0) {
		qep_err(hw, "%s: Error in configuring RS-FEC in CMAC Core\n",
			__func__);
		return ret;
	}

	if (loopback_en)
		qep_cmac_loopback_en(xpriv, true);

	/* Setting MTU Size in CMAC */
	cmac_mtu = xpriv->netdev->mtu + ETH_HLEN + ETH_FCS_LEN;
	io_addr = (void __iomem *)((u64)xpriv->bar_base +
			QEP_CMAC_DRP_OFFSET +
			QEP_CMAC_CTL_RX_MAX_PACKET_LEN_OFFSET);
	writel(cmac_mtu, io_addr);

	cmac_mtu = readl(io_addr);
	qep_dbg(xpriv->netdev, "%s: Default MTU size = %d CMAC:%d\n", __func__,
		xpriv->netdev->mtu, cmac_mtu);
	return ret;
}

/* This function enables CMAC IP */
static int qep_cmac_start(struct qep_priv *xpriv)
{
	int ret = 0;
	u32 reg_val = 0;
	void __iomem *io_addr;
	struct xcmac *inst = &xpriv->cmac_dev;

	if (xpriv->is_cmac_on == 1)
		return 0;

	ret = xcmac_enable(inst, XCMAC_CORE_RX);
	if (ret != 0) {
		qep_err(hw, "%s: Error in enabling CMAC Rx core\n", __func__);
		return ret;
	}

	ret = xcmac_set_fault_indication(inst, true);
	if (ret != 0) {
		qep_err(hw, "%s: Error in setting Tx fault indication\n",
			__func__);
		return ret;
	}

	ret = xcmac_enable(inst, XCMAC_CORE_TX);
	if (ret != 0) {
		qep_err(hw, "%s: Error in enabling CMAC Tx core\n", __func__);
		return ret;
	}

	ret = xcmac_set_fault_indication(inst, false);
	if (ret != 0) {
		qep_err(hw, "%s: Error in clearing Tx fault indication\n",
			__func__);
		return ret;
	}

	/* Start TX output for peer to realize that link is up */
	io_addr = (void __iomem *)((u64)xpriv->bar_base +
		(QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_TX_CTRL_OFFSET));
	reg_val = readl(io_addr);
	reg_val = reg_val & ~(1 << QEP_CMAC_CTRL_TX_CTRL_TX_INHIBIT_SHIFT);
	writel(reg_val, io_addr);
	readl(io_addr); /* dummy read */
	msleep(100);

	xpriv->is_cmac_on = 1;
	return ret;
}

/* This function disables CMAC IP */
static int qep_cmac_stop(struct qep_priv *xpriv)
{
	int ret = 0;
	u32 reg_val = 0;
	void __iomem *io_addr;
	struct xcmac *inst = &xpriv->cmac_dev;

	if (xpriv->is_cmac_on == 0)
		return 0;

	xpriv->is_cmac_on = 0;
	io_addr = (void __iomem *)((u64)xpriv->bar_base +
		(QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_TX_CTRL_OFFSET));

	/* Stop TX output for peer to realize link is down */
	reg_val = readl(io_addr);
	reg_val = reg_val | (1 << QEP_CMAC_CTRL_TX_CTRL_TX_INHIBIT_SHIFT);
	writel(reg_val, io_addr);
	readl(io_addr);

	ret = xcmac_disable(inst, XCMAC_CORE_RX);
	if (ret != 0) {
		qep_err(hw, "%s: Error in disabling CMAC Rx core\n", __func__);
		return ret;
	}

	ret = xcmac_disable(inst, XCMAC_CORE_TX);
	if (ret != 0) {
		qep_err(hw, "%s: Error in disabling CMAC Tx core\n", __func__);
		return ret;
	}

	return ret;
}

static int qep_cmac_restart(struct qep_priv *xpriv)
{
	int ret;

	if (xpriv->dev_state != QEP_DEV_STATE_UP) {
		qep_err(drv, "%s: Device state is not up\n", __func__);
		return -EINVAL;
	}

	ret = qep_cmac_stop(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_cmac_stop() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	ret = qep_reset_all(xpriv);
	if (ret < 0) {
		qep_err(drv, "%s : qep_reset_all() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	ret = qep_cmac_setup(xpriv);
	if ((ret != 0) && (ret != -EAGAIN)) {
		qep_err(drv, "%s: qep_cmac_setup() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	ret = qep_cmac_start(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_cmac_start() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	return ret;
}
/* This function gets global CSR and sets the default indexes of the
 * xpriv structure. If the default value set in the driver doesn't match with
 * any of the values supported, index 0 is returned by default.
 */
static int qep_qdma_csr_index_setup(struct qep_priv *xpriv)
{
	int ret = 0, index = 0;
	struct global_csr_conf csr_conf;

	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return -EINVAL;
	}

	index = qep_arr_find(csr_conf.ring_sz, QDMA_GLOBAL_CSR_ARRAY_SZ,
			QEP_DEFAULT_RING_SIZE);
	if (index < 0) {
		qep_err(drv, "%s: Expected ring size %d not found", __func__,
				QEP_DEFAULT_RING_SIZE);
		return index;
	}
	xpriv->rx_desc_rng_sz_idx = index;
	xpriv->tx_desc_rng_sz_idx = index;
	xpriv->cmpl_rng_sz_idx = index;

	index = qep_arr_find(csr_conf.c2h_timer_cnt, QDMA_GLOBAL_CSR_ARRAY_SZ,
				QEP_DEFAULT_C2H_TIMER_COUNT);
	if (index < 0) {
		qep_err(drv,
			"%s: Expected default C2H Timer count %d not found",
			__func__, QEP_DEFAULT_C2H_TIMER_COUNT);
		return index;
	}
	xpriv->rx_timer_idx = index;

	index = qep_arr_find(csr_conf.c2h_cnt_th, QDMA_GLOBAL_CSR_ARRAY_SZ,
				QEP_DEFAULT_C2H_COUNT_THRESHOLD);
	if (index < 0) {
		qep_err(drv,
			"%s: Expected default C2H count threshold count %d not found",
			__func__, QEP_DEFAULT_C2H_COUNT_THRESHOLD);
		return index;
	}
	xpriv->rx_cnt_th_idx = index;

	index = qep_arr_find(csr_conf.c2h_buf_sz, QDMA_GLOBAL_CSR_ARRAY_SZ,
				QEP_DEFAULT_C2H_BUFFER_SIZE);
	if (index < 0) {
		qep_err(drv,
			"%s: Expected default C2H Buffer size %d not found",
			__func__, QEP_DEFAULT_C2H_BUFFER_SIZE);
		return index;
	}
	xpriv->rx_buf_sz_idx = index;
	return 0;
}

/* Add a RX queue to QDMA */
static int qep_qdma_rx_queue_add(struct qep_priv *xpriv, u32 q_no, u8 timer_idx,
				 u8 cnt_th_idx)
{
	int ret = 0;
	char error_str[QEP_ERROR_STR_BUF_LEN] = { '0' };
	unsigned long q_handle = 0;
	struct qdma_queue_conf qconf;

	memset(&qconf, 0, sizeof(struct qdma_queue_conf));
	qconf.st = 1;
	qconf.q_type = Q_C2H;
	qconf.irq_en = 0;
	qconf.adaptive_rx = xpriv->rx_q[q_no].adaptive_update;
	qconf.pfetch_en = 0;
	qconf.latency_optimize = 1;

	if (c2h_byp_mode) {
		qconf.desc_bypass = 1;
		qconf.pfetch_bypass = 1;
	} else {
		qconf.desc_bypass = 0;
		qconf.pfetch_bypass = 0;
	}

	qconf.fetch_credit = 1;
	qconf.cmpl_stat_en = 1;
	qconf.cmpl_udd_en = 1;
	qconf.cmpl_desc_sz = DESC_SZ_8B;

	qconf.cmpl_rng_sz_idx = xpriv->cmpl_rng_sz_idx;
	qconf.desc_rng_sz_idx = xpriv->rx_desc_rng_sz_idx;
	qconf.c2h_buf_sz_idx = xpriv->rx_buf_sz_idx;
	if (!qconf.adaptive_rx) { /* override with queue specific values */
		struct global_csr_conf csr_conf;

		ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
		if (ret != 0) {
			qep_err(drv,
				"%s: qdma_global_csr_get() failed with status %d\n",
				__func__, ret);
			return ret;
		}
		timer_idx = qep_arr_find(csr_conf.c2h_timer_cnt,
			QDMA_GLOBAL_CSR_ARRAY_SZ,
			xpriv->rx_q[q_no].coalesce_usecs);
		cnt_th_idx = qep_arr_find(csr_conf.c2h_cnt_th,
			QDMA_GLOBAL_CSR_ARRAY_SZ,
			xpriv->rx_q[q_no].coalesce_frames);
		/*
		 * No validation of index is required as they are validated
		 * at source
		 */
	} else {
		xpriv->rx_q[q_no].coalesce_usecs = QEP_DEFAULT_C2H_TIMER_COUNT;
		xpriv->rx_q[q_no].coalesce_frames =
				QEP_DEFAULT_C2H_COUNT_THRESHOLD;
	}
	qconf.cmpl_timer_idx = timer_idx;
	qconf.cmpl_cnt_th_idx = cnt_th_idx;

	qconf.cmpl_trig_mode = TRIG_MODE_COMBO;
	qconf.cmpl_en_intr = (poll_mode == 0);

	qconf.quld = (unsigned long)xpriv;
	qconf.fp_descq_isr_top = qep_isr_rx_tophalf;
	qconf.fp_descq_c2h_packet = qep_rx_pkt_process;
	qconf.fp_proc_ul_cmpt_entry = qep_stmn_parse_cmpl_entry;

	qep_dbg(xpriv->netdev,
		"%s: c2h_rng_sz_idx = %d, desc_rng_sz_idx = %d, c2h_buf_sz_idx = %d, c2h_timer_idx = %d, c2h_cnt_th_idx = %d\n",
		__func__, qconf.cmpl_rng_sz_idx, qconf.desc_rng_sz_idx,
		qconf.c2h_buf_sz_idx, qconf.cmpl_timer_idx,
		qconf.cmpl_cnt_th_idx);

	qconf.qidx = q_no;
	ret = qdma_queue_add(xpriv->dev_handle, &qconf, &q_handle, error_str,
				QEP_ERROR_STR_BUF_LEN);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_queue_add() failed for queue %d with status %d(%s)\n",
			__func__, qconf.qidx, ret, error_str);
		return ret;
	}

	/* Get base q_handle */
	if (q_no == 0)
		xpriv->base_rx_q_handle = q_handle;

	xpriv->rx_q[q_no].parent = xpriv;
	xpriv->rx_q[q_no].q_handle = q_handle;
	xpriv->rx_q[q_no].queue_id = q_no;

	return ret;
}

/* This function releases Tx queues */
static void qep_qdma_tx_queue_release(struct qep_priv *xpriv, int num_queues)
{
	int ret = 0, q_no = 0;
	char error_str[QEP_ERROR_STR_BUF_LEN] = { '0' };

	for (q_no = 0; q_no < num_queues; q_no++) {
		ret = qdma_queue_remove(xpriv->dev_handle,
					(xpriv->base_tx_q_handle + q_no),
					error_str, QEP_ERROR_STR_BUF_LEN);
		if (ret != 0) {
			qep_err(drv,
				"%s: qdma_queue_remove() failed for queue %d with status %d(%s)\n",
				__func__, q_no, ret, error_str);
		}

		tasklet_kill(&xpriv->task_tx_done[q_no]);
		xpriv->tx_q[q_no].q_handle = 0;
		xpriv->tx_q[q_no].parent = NULL;
	}

	kfree(xpriv->task_tx_done);
}

/* This function releases Rx queues */
static void qep_qdma_rx_queue_release(struct qep_priv *xpriv, int num_queues)
{
	int ret = 0, q_no = 0;
	char error_str[QEP_ERROR_STR_BUF_LEN] = { '0' };

	for (q_no = 0; q_no < num_queues; q_no++) {
		ret = qdma_queue_remove(xpriv->dev_handle,
					(xpriv->base_rx_q_handle + q_no),
					error_str, QEP_ERROR_STR_BUF_LEN);
		if (ret != 0) {
			qep_err(drv,
				"%s: qdma_queue_remove() failed for queue %d with status %d(%s)\n",
				__func__, q_no, ret, error_str);
		}
		netif_napi_del(&xpriv->napi[q_no]);

		xpriv->rx_q[q_no].q_handle = 0;
		xpriv->rx_q[q_no].parent = NULL;
	}

	kfree(xpriv->napi);
}

/* This function sets up RX queues*/
static int qep_qdma_rx_queue_setup(struct qep_priv *xpriv)
{
	int ret = 0, q_no = 0;


	xpriv->napi = kcalloc(xpriv->netdev->real_num_rx_queues,
				sizeof(struct napi_struct),
				GFP_KERNEL);
	if (!xpriv->napi) {
		kfree(xpriv->rx_q);
		return -ENOMEM;
	}

	for (q_no = 0; q_no < xpriv->netdev->real_num_rx_queues; q_no++) {
		ret = qep_qdma_rx_queue_add(xpriv, q_no, xpriv->rx_timer_idx,
				xpriv->rx_cnt_th_idx);
		if (ret != 0) {
			qep_err(drv,
				"%s: qep_qdma_rx_queue_add() failed for queue %d with status %d\n",
				__func__, q_no, ret);
			goto release_rx_q;
		}

		netif_napi_add(xpriv->netdev, &xpriv->napi[q_no], qep_rx_poll,
				QEP_NAPI_WEIGHT);
	}

	return 0;

release_rx_q:
	qep_qdma_rx_queue_release(xpriv, q_no);
	return ret;
}

/* This function sets up Tx queues */
static int qep_qdma_tx_queue_setup(struct qep_priv *xpriv)
{
	int ret = 0, q_no = 0;
	char error_str[QEP_ERROR_STR_BUF_LEN] = { '0' };
	unsigned long q_handle = 0;
	struct qdma_queue_conf qconf;

	xpriv->task_tx_done = kcalloc(xpriv->netdev->real_num_tx_queues,
		sizeof(struct tasklet_struct), GFP_KERNEL);
	if (!xpriv->task_tx_done)
		return -ENOMEM;

	for (q_no = 0; q_no < xpriv->netdev->real_num_tx_queues; q_no++) {
		memset(&qconf, 0, sizeof(struct qdma_queue_conf));
		qconf.st = 1;
		qconf.q_type = Q_H2C;
		qconf.pidx_acc = xpriv->tx_q[q_no].coalesce_frames;
		qconf.pidx_upd_timeout_usecs = xpriv->tx_q[q_no].coalesce_usecs;
		qconf.irq_en = 1;
		qconf.wb_status_en = 1;
		qconf.fetch_credit = 1;
		qconf.cmpl_stat_en = 1;
		qconf.cmpl_status_acc_en = 1;
		qconf.cmpl_status_pend_chk = 1;
		qconf.desc_bypass = h2c_byp_mode;
		qconf.desc_rng_sz_idx = xpriv->tx_desc_rng_sz_idx;
		qconf.fp_descq_isr_top = qep_isr_tx_tophalf;
		qconf.fp_bypass_desc_fill = qep_stmn_bypass_desc_fill;
		qconf.quld = (unsigned long)xpriv;
		qconf.qidx = q_no;

		ret = qdma_queue_add(xpriv->dev_handle, &qconf, &q_handle,
					error_str, QEP_ERROR_STR_BUF_LEN);
		if (ret != 0) {
			qep_err(drv,
				"%s: qdma_queue_add() failed for queue %d with status %d(%s)\n",
				__func__, qconf.qidx, ret, error_str);
			goto cleanup_tx_q;
		}
		if (q_no == 0)
			xpriv->base_tx_q_handle = q_handle;

		xpriv->tx_q[q_no].parent = xpriv;
		xpriv->tx_q[q_no].q_handle = q_handle;
		xpriv->tx_q[q_no].queue_id = q_no;

		tasklet_init(&xpriv->task_tx_done[q_no], qep_isr_tx_bottom_half,
				(unsigned long)&(xpriv->tx_q[q_no]));
	}
	return 0;

cleanup_tx_q:
	qep_qdma_tx_queue_release(xpriv, q_no);
	return ret;
}

/* Configure QDMA Device, Global CSR Registers */
static int qep_qdma_setup(struct qep_priv *xpriv)
{
	int ret, status;
	enum qdma_drv_mode qdma_drv_mode;
#if 0
	void __iomem *io_addr =
			(void __iomem *)((u64)xpriv->bar_base +
					QEP_BASE_OFFSET);

	writel(1, (void __iomem *)((u64)io_addr +
			QEP_BASE_QDMA_RESET_CONTROL_OFFSET);
	msleep(100);
#endif

	memset(&xpriv->qdma_dev_conf, 0, sizeof(struct qdma_dev_conf));

	xpriv->qdma_dev_conf.msix_qvec_max = QEP_NUM_IRQ_MAX;
	xpriv->qdma_dev_conf.data_msix_qvec_max = QEP_NUM_IRQ_MAX;
	xpriv->qdma_dev_conf.user_msix_qvec_max = 0;
	xpriv->qdma_dev_conf.master_pf = 0;
	xpriv->qdma_dev_conf.intr_moderation = 1;
	xpriv->qdma_dev_conf.qsets_max = QEP_NUM_MAX_QUEUES;
	xpriv->qdma_dev_conf.qsets_base = QEP_QUEUE_BASE;
	xpriv->qdma_dev_conf.pdev = xpriv->pcidev;
	xpriv->qdma_dev_conf.debugfs_dev_root = xpriv->debugfs_dev_root;
	if (poll_mode)
		qdma_drv_mode = POLL_MODE;
	else
		qdma_drv_mode = DIRECT_INTR_MODE;
	xpriv->qdma_dev_conf.qdma_drv_mode = qdma_drv_mode;

	ret = qdma_device_open(DRV_NAME, &xpriv->qdma_dev_conf,
				&xpriv->dev_handle);
	if (ret != 0) {
		qep_err(drv, "%s: qdma_device_open() failed, Error Code : %d\n",
			__func__, ret);
		return -EINVAL;
	}

	status = qep_qdma_csr_index_setup(xpriv);
	if (status != 0) {
		qep_err(drv,
			"%s: qep_qdma_csr_index_setup() failed with status %d CSR not programmed,\n",
			__func__, status);
		ret = -EINVAL;
		goto close_qdma;
	}
	pr_info("%s: %02x:%02x.%02x, done\n", __func__,
		xpriv->pcidev->bus->number,
		PCI_SLOT(xpriv->pcidev->devfn),
		PCI_FUNC(xpriv->pcidev->devfn));

	return 0;

close_qdma:
	qdma_device_close(xpriv->pcidev, xpriv->dev_handle);
	return ret;
}

/* This function stops Rx and Tx queues operations */
static int qep_qdma_stop(struct qep_priv *xpriv, unsigned short int txq,
			 unsigned short int rxq)
{
	int ret = 0, q = 0, err = 0;
	char error_str[QEP_ERROR_STR_BUF_LEN] = {'0'};

	for (q = 0; q < rxq; q++) {
		ret = qdma_queue_stop(xpriv->dev_handle,
				(xpriv->base_rx_q_handle + q), error_str,
				QEP_ERROR_STR_BUF_LEN);
		if (ret < 0) {
			qep_err(drv,
			"%s: qdma_queue_stop() failed for Rx queue %d with status %d msg: %s\n",
			__func__, q, ret, error_str);
			err = -EINVAL;
		}
	}

	for (q = 0; q < txq; q++) {
		ret = qdma_queue_stop(xpriv->dev_handle,
				(xpriv->base_tx_q_handle + q), error_str,
				QEP_ERROR_STR_BUF_LEN);
		if (ret < 0) {
			qep_err(drv,
			"%s: qdma_queue_stop() failed for Tx queue %d with status %d msg: %s\n",
			__func__, q, ret, error_str);
			err = -EINVAL;
		}
	}

	return err;
}

/* This function starts Rx and Tx queues operations */
static int qep_qdma_start(struct qep_priv *xpriv)
{
	int ret, q_no;
	char error_str[QEP_ERROR_STR_BUF_LEN] = { '0' };

	for (q_no = 0; q_no < xpriv->netdev->real_num_rx_queues; q_no++) {
		ret = qdma_queue_start(xpriv->dev_handle,
				(xpriv->base_rx_q_handle + q_no),
				error_str, QEP_ERROR_STR_BUF_LEN);
		if (ret != 0) {
			qep_err(drv,
				"%s: qdma_queue_start() failed for Rx queue %d with status %d(%s)\n",
				__func__, q_no, ret, error_str);
			qep_qdma_stop(xpriv, 0, q_no);
			return ret;
		}
	}

	for (q_no = 0; q_no < xpriv->netdev->real_num_tx_queues; q_no++) {
		ret = qdma_queue_start(xpriv->dev_handle,
				(xpriv->base_tx_q_handle + q_no),
				error_str, QEP_ERROR_STR_BUF_LEN);
		if (ret != 0) {
			qep_err(drv,
				"%s: qdma_queue_start() failed for Tx queue %d with status %d(%s)\n",
				__func__, q_no, ret, error_str);
			qep_qdma_stop(xpriv, q_no,
				 xpriv->netdev->real_num_rx_queues);
			return ret;
		}
	}
	return 0;
}

/* This function stops QEP data path */
static int qep_stop_data_path(struct qep_priv *xpriv)
{
	int ret = 0, q_no = 0;
	struct clcr_config config;

	netif_carrier_off(xpriv->netdev);

	/* Stop CMAC */
	ret = qep_cmac_stop(xpriv);
	if (ret != 0)
		qep_err(drv, "%s: qep_cmac_stop() failed with status %d\n",
			__func__, ret);

	ret = qep_qdma_stop(xpriv, xpriv->netdev->real_num_tx_queues,
			xpriv->netdev->real_num_rx_queues);
	if (ret != 0)
		qep_err(drv, "%s: qep_qdma_stop() failed with status %d\n",
			__func__, ret);

	for (q_no = 0; q_no < xpriv->netdev->real_num_rx_queues; q_no++)
		napi_disable(&xpriv->napi[q_no]);

	config.mode = CLCR_FIXED_MODE;
	config.fixed.qnum = 0;
	ret = qep_clcr_setup(xpriv, &config);
	if (ret != 0)
		qep_err(drv, "%s: qep_clcr_setup() failed with status %d\n",
			__func__, ret);
	return ret;
}

/* This function starts QEP data path */
static int qep_start_data_path(struct qep_priv *xpriv)
{
	int ret = 0, q_no = 0;
	struct clcr_config config;

	ret = qep_qdma_start(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_qdma_start() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	for (q_no = 0; q_no < xpriv->netdev->real_num_rx_queues; q_no++)
		napi_enable(&xpriv->napi[q_no]);

	if (xpriv->netdev->features & NETIF_F_RXHASH) {
		config.mode = CLCR_LOOKUP_MODE;
		config.lookup.qnum = xpriv->netdev->real_num_rx_queues;
	} else {
		config.mode = CLCR_ROUND_ROBIN;
		config.rr.num_queue = xpriv->netdev->real_num_rx_queues;
		config.rr.qbase = 0;
	}
	ret = qep_clcr_setup(xpriv, &config);
	if (ret != 0) {
		qep_err(drv, "%s: qep_clcr_setup() failed with status %d\n",
			__func__, ret);
		goto disable_napi;
	}
	msleep(100);

	if (poll_mode)
		for (q_no = 0; q_no < xpriv->netdev->real_num_rx_queues; q_no++)
			napi_schedule(&xpriv->napi[q_no]);

	ret = qep_cmac_start(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_cmac_start() failed with status %d\n",
			__func__, ret);
		goto disable_napi;
	}

	return ret;

disable_napi:
	for (q_no = 0; q_no < xpriv->netdev->real_num_rx_queues; q_no++)
		napi_disable(&xpriv->napi[q_no]);
	qep_qdma_stop(xpriv, xpriv->netdev->real_num_tx_queues,
			 xpriv->netdev->real_num_rx_queues);
	return ret;
}

/* This function sets netdev features
 */
static int qep_set_netdev_features(struct net_device *netdev)
{
	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	netdev->gflags |= IFF_PROMISC;

	netdev->vlan_features |=
#ifdef QEP_CSO_EN
			NETIF_F_IP_CSUM |
			NETIF_F_RXCSUM |
#ifdef NETIF_F_IPV6_CSUM
			NETIF_F_IPV6_CSUM |
#endif /* NETIF_F_IPV6_CSUM */
#if defined(NETIF_F_RXHASH) && defined(QEP_RSS_EN)
			NETIF_F_RXHASH |
#endif /* NETIF_F_RXHASH */
#endif
			NETIF_F_SG |
			NETIF_F_HIGHDMA;

	netdev->hw_features |= netdev->vlan_features;
	netdev->features |= netdev->hw_features;

	return 0;
}
int qep_stats_alloc(struct qep_priv *xpriv)
{
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}
	if (!xpriv->netdev) {
		pr_err("%s: xpriv->netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv->mac_stats = kcalloc(1,
			sizeof(struct qep_cmac_stats) +
			sizeof(struct qep_drv_stats) +
			(xpriv->netdev->real_num_tx_queues *
			sizeof(struct rtnl_link_stats64)) +
			(xpriv->netdev->real_num_rx_queues *
			sizeof(struct rtnl_link_stats64))
			, GFP_KERNEL);

	if (!xpriv->mac_stats) {
		qep_err(drv, "%s: Memory allocation failure for stats\n",
			__func__);
		return -ENOMEM;
	}

	xpriv->drv_stats = (struct qep_drv_stats *)(xpriv->mac_stats + 1);
	xpriv->tx_qstats = (struct rtnl_link_stats64 *)(xpriv->drv_stats + 1);
	xpriv->rx_qstats = xpriv->tx_qstats + xpriv->netdev->real_num_tx_queues;

	return 0;
}

/* This function gets called when interface gets 'UP' request via 'ifconfig up'
 * In this function, Rx and Tx queues are setup and send/receive operations
 * are started
 */
int qep_open(struct net_device *netdev)
{
	int ret = 0;
	struct qep_priv *xpriv;
	void __iomem *io_addr;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	spin_lock(&xpriv->lnk_state_lock);
	if (xpriv->dev_state == QEP_DEV_STATE_UP) {
		spin_unlock(&xpriv->lnk_state_lock);
		return 0;
	}
	spin_unlock(&xpriv->lnk_state_lock);

	spin_lock(&xpriv->config_lock);

	ret = qep_stats_alloc(xpriv);
	if (ret != 0) {
		qep_err(drv,
			"%s: qep_stats_alloc() failed with status %d\n",
			__func__, ret);
		spin_unlock(&xpriv->config_lock);
		return ret;
	}
	io_addr = (void __iomem *)((u64)xpriv->bar_base +
			QEP_BASE_OFFSET);

	/* reset h2c and c2h stmn cores */
	writel(1, (void __iomem *)((u64)io_addr +
			QEP_BASE_DMA_USER_H2C_OFFSET));
	writel(0, (void __iomem *)((u64)io_addr +
			QEP_BASE_DMA_USER_H2C_OFFSET));

	writel(1, (void __iomem *)((u64)io_addr +
			QEP_BASE_DMA_USER_C2H_OFFSET));
	writel(0, (void __iomem *)((u64)io_addr +
			QEP_BASE_DMA_USER_C2H_OFFSET));

	ret = qep_qdma_rx_queue_setup(xpriv);
	if (ret != 0) {
		qep_err(drv,
			"%s: qep_qdma_rx_queue_setup() failed with status %d\n",
			__func__, ret);
		goto err_exit;
	}

	ret = qep_qdma_tx_queue_setup(xpriv);
	if (ret != 0) {
		qep_err(drv,
			"%s: qep_qdma_tx_queue_setup() failed with status %d\n",
			__func__, ret);
		goto release_rx_queues;
	}

	ret = qep_start_data_path(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_start_data_path() failed with status %d\n",
			__func__, ret);
		goto release_queues;
	}

	ret = stmn_snap_stats(xpriv->tm_dev.xdev);
	if (ret < 0)
		qep_err(drv, "%s: stmn_snap_stats() failed\n",
			__func__);

	spin_unlock(&xpriv->config_lock);

	spin_lock(&xpriv->lnk_state_lock);
	xpriv->lnk_state = CMAC_LINK_DOWN;
	xpriv->dev_state = QEP_DEV_STATE_UP;
	spin_unlock(&xpriv->lnk_state_lock);

	wake_up_interruptible(&xpriv->link_mon_needed);
	qep_info(drv, "device open done");

	return 0;

release_queues:
	qep_qdma_tx_queue_release(xpriv, xpriv->netdev->real_num_tx_queues);
release_rx_queues:
	qep_qdma_rx_queue_release(xpriv, xpriv->netdev->real_num_rx_queues);
err_exit:
	netif_tx_stop_all_queues(xpriv->netdev);
	netif_carrier_off(netdev);
	xpriv->dev_state = QEP_DEV_STATE_DOWN;
	spin_unlock(&xpriv->config_lock);
	return ret;
}

/* This function gets called when there is a interface down request.
 * In this function, Tx/Rx operations on the queues are stopped
 */
int qep_stop(struct net_device *netdev)
{
	int ret = 0;
	struct qep_priv *xpriv;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	spin_lock(&xpriv->lnk_state_lock);
	if (xpriv->dev_state == QEP_DEV_STATE_DOWN) {
		spin_unlock(&xpriv->lnk_state_lock);
		return 0;
	}
	xpriv->dev_state = QEP_DEV_STATE_DOWN;
	xpriv->lnk_state = CMAC_LINK_DOWN;
	spin_unlock(&xpriv->lnk_state_lock);


	spin_lock(&xpriv->config_lock);
	netif_tx_stop_all_queues(xpriv->netdev);

	ret = qep_stop_data_path(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: Error in stopping data path, Error = %d\n",
			__func__, ret);
	}
	qep_qdma_rx_queue_release(xpriv, xpriv->netdev->real_num_rx_queues);
	qep_qdma_tx_queue_release(xpriv, xpriv->netdev->real_num_tx_queues);

	kfree(xpriv->mac_stats);
	spin_unlock(&xpriv->config_lock);

	qep_info(drv, "%s: device close done\n", __func__);
	return ret;
}


/* This function free skb allocated memory */
static int qep_unmap_free_pkt_data(struct qdma_request *req)
{
	u16 nb_frags = 0, frag_index = 0;
	struct qdma_sw_sg *qdma_sgl;
	struct qep_tx_cb_arg *qep_tx_cb;
	struct net_device *netdev;
	struct sk_buff *skb;
	skb_frag_t *frag;

	if (req == NULL) {
		pr_err("%s: req is NULL\n", __func__);
		return -EINVAL;
	}

	qep_tx_cb = (struct qep_tx_cb_arg *)req->uld_data;
	if (!qep_tx_cb) {
		pr_err("%s: qep_tx_cb is NULL\n", __func__);
		return -EINVAL;
	}

	netdev = qep_tx_cb->netdev;
	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	skb = qep_tx_cb->skb;
	if (!skb) {
		pr_err("%s: skb is NULL\n", __func__);
		return -EINVAL;
	}

	if (skb_shinfo(skb)->nr_frags)
		nb_frags = skb_shinfo(skb)->nr_frags;

	qdma_sgl = req->sgl;
	dma_unmap_single(netdev->dev.parent, qdma_sgl->dma_addr,
			 skb_headlen(skb), DMA_TO_DEVICE);

	qep_dbg(netdev,
		"%s: skb->len = %d skb_headlen(skb) = %d, dma_addr = %llx nb_frags:%d\n",
		__func__, skb->len, skb_headlen(skb), qdma_sgl->dma_addr,
		nb_frags);

	qdma_sgl = qdma_sgl->next;
	while (qdma_sgl && (frag_index < nb_frags)) {
		frag = &skb_shinfo(skb)->frags[frag_index];
		qdma_sgl->len = skb_frag_size(frag);
		dma_unmap_single(netdev->dev.parent, qdma_sgl->dma_addr,
				 qdma_sgl->len, DMA_TO_DEVICE);
		qdma_sgl = qdma_sgl->next;
		frag_index++;
	}

	if (skb)
		dev_kfree_skb_any(skb);

	kfree(qep_tx_cb);
	return 0;
}

/* This function is called from networking stack in order to send packet */
static int qep_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	u16 q_id = 0, nb_frags = 0, frag_index = 0;
	int ret = 0, count = 0;
	unsigned int cmac_mtu;
	unsigned long q_handle;
	struct qep_priv *xpriv;
	struct qep_tx_cb_arg *qep_cb_args;
	struct qdma_request *qdma_req;
	struct qdma_sw_sg *qdma_sgl;
	struct iphdr *ip_header;
	union h2c_metadata *csum_data;
	skb_frag_t *frag;
	u8 l4_protocol = 0;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (unlikely(skb->len <= ETH_HLEN)) {
		qep_err(tx_err, "%s: skb->len = %d is less than eth header len %d\n",
			__func__, skb->len, ETH_HLEN);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	if (unlikely(skb->len == 0)) {
		qep_err(tx_err, "%s: skb->len = %d is zero\n",
			__func__, skb->len);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	cmac_mtu = netdev->mtu + ETH_HLEN + ETH_FCS_LEN;
	if (unlikely(skb->len > cmac_mtu)) {
		qep_err(tx_err, "%s: skb->len = %d is greater than MTU %d\n",
			__func__, skb->len, cmac_mtu);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	q_id = skb_get_queue_mapping(skb);
	if (q_id >= xpriv->netdev->real_num_tx_queues) {
		qep_err(tx_err, "%s: Invalid queue mapping. q_id = %d\n",
			__func__, q_id);
		dev_kfree_skb_any(skb);
		return -EINVAL;
	}

	if (!netif_carrier_ok(netdev)) {
		qep_err(tx_err, "%s: Packet sent when carrier is down\n",
					__func__);
		dev_kfree_skb_any(skb);
		return -EINVAL;
	}

	if (skb_is_gso(skb)) {
		qep_err(tx_err, "%s: Received GSO SKB\n", __func__);
		return -EINVAL;
	}

	q_handle = xpriv->base_tx_q_handle + q_id;

	if (skb_shinfo(skb)->nr_frags) {
		qep_dbg(xpriv->netdev,
			"%s: Detected %d number of skb fragments\n",
			__func__, skb_shinfo(skb)->nr_frags);
		nb_frags = skb_shinfo(skb)->nr_frags;
	}
	/* Allocate memory in single call of kzalloc call. Offset access will be
	 * 1) struct qep_tx_cb_arg
	 * 2) struct qdma_request
	 * 3) struct qdma_sw_sg * (nb_frags + 1)
	 */
	qep_cb_args = kzalloc(
		sizeof(struct qep_tx_cb_arg) + sizeof(struct qdma_request) +
			(sizeof(struct qdma_sw_sg) * (nb_frags + 1)),
		GFP_ATOMIC);
	if (qep_cb_args == NULL) {
		qep_err(tx_err, "%s: qep_cb_args allocation failed\n",
			__func__);
		return -ENOMEM;
	}
	qdma_req = (struct qdma_request *)(qep_cb_args + 1);
	qdma_sgl = (struct qdma_sw_sg *)(qdma_req + 1);

	qep_cb_args->skb = skb;
	qep_cb_args->netdev = netdev;

	qdma_req->sgl = qdma_sgl;

	ip_header = (struct iphdr *)skb_network_header(skb);
	qep_dbg(xpriv->netdev,
		"%s: Received-SKB: len:%d data_len:%d headlen:%d truesize:%d mac_len:%d Frags:%d Proto:%d\n",
		__func__, skb->len, skb->data_len, skb->truesize, skb->mac_len,
		skb_headlen(skb), nb_frags, ip_header->protocol);

#if (QEP_DBG_DUMP_EN)
	dump_skb(xpriv, skb);
#endif

	qdma_sgl->len = skb_headlen(skb);
	qdma_req->count = qdma_sgl->len;

	qdma_sgl->dma_addr = dma_map_single(netdev->dev.parent, skb->data,
					skb_headlen(skb), DMA_TO_DEVICE);
	if (dma_mapping_error(netdev->dev.parent, qdma_sgl->dma_addr)) {
		qep_err(tx_err, "%s: dma_map_single() failed\n", __func__);
		kfree(qep_cb_args);
		ret = -EFAULT;
		goto free_packet_data;
	}

	qdma_sgl->next = NULL;
	qdma_req->sgcnt++;

	/* DMA mapping for fragments data */
	for (frag_index = 0; frag_index < nb_frags; frag_index++) {
		qdma_sgl->next = (qdma_sgl + 1);
		qdma_sgl = qdma_sgl->next;
		qdma_sgl->next = NULL;

		frag = &skb_shinfo(skb)->frags[frag_index];
		qdma_sgl->len = skb_frag_size(frag);
		qdma_req->count += qdma_sgl->len;

		qep_dbg(xpriv->netdev, "%s: frag no = %d, skb_frag_size = %d\n",
			__func__, frag_index, qdma_sgl->len);

		qdma_sgl->dma_addr = (unsigned long)skb_frag_dma_map(
			netdev->dev.parent, frag, 0, skb_frag_size(frag),
			DMA_TO_DEVICE);
		if (dma_mapping_error(netdev->dev.parent, qdma_sgl->dma_addr)) {
			qep_err(tx_err, "%s: dma_map_single failed\n",
				__func__);
			ret = -EFAULT;
			goto free_packet_data;
		}
		qdma_req->sgcnt++;
	}

	qdma_req->write = 1;
	qdma_req->dma_mapped = 1;
	qdma_req->timeout_ms = 0;

	qdma_req->fp_done = qep_tx_done;
	qdma_req->uld_data = (unsigned long)qep_cb_args;

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		csum_data = (union h2c_metadata *)qdma_req->udd;
		csum_data->l3_hdr_off = skb_network_offset(skb);
		csum_data->l4_hdr_off = skb_network_offset(skb) +
						skb_network_header_len(skb);
		csum_data->l3_cksum_gen = 1;
		csum_data->l4_cksum_gen = 1;

		switch (ip_hdr(skb)->version) {
		case IPVERSION:
			/* Calculate additional header length */
			csum_data->l3_len = ip_hdr(skb)->ihl - 5;
			l4_protocol = ip_hdr(skb)->protocol;
			break;
		case QEP_IPV6:
			/* TODO: longer IPv6 packets */
			csum_data->l3_len = 0;
			l4_protocol = ipv6_hdr(skb)->nexthdr;
			break;
		default:
			skb_checksum_help(skb);
			goto csum_failed;
		}
#ifdef QEP_SW_COUNTER_EN
		xpriv->drv_stats->tx_pkt_l3_csum++;
#endif
		if (l4_protocol == IPPROTO_TCP) {
			struct tcphdr *tcp = tcp_hdr(skb);

			csum_data->udp = 0;
			csum_data->l4_len = tcp->doff;
		} else if (l4_protocol == IPPROTO_UDP) {
			csum_data->udp = 1;
			csum_data->l4_len = QEP_L4_UDP_LEN;
		} else {
			csum_data->l4_cksum_gen = 0;
			skb_checksum_help(skb);
		}
#ifdef QEP_SW_COUNTER_EN
		xpriv->drv_stats->tx_pkt_l4_csum++;
#endif
	}

csum_failed:

	count = qdma_queue_packet_write(xpriv->dev_handle, q_handle, qdma_req);
	if (count < 0) {
		qep_err(tx_err,
			"%s: qdma_queue_packet_write() failed, err = %d\n",
			__func__, count);
		ret = count;
		goto free_packet_data;
	}

	xpriv->tx_qstats[q_id].tx_packets++;
	xpriv->tx_qstats[q_id].tx_bytes += skb->len;

	return 0;

free_packet_data:
	if (qep_unmap_free_pkt_data(qdma_req) != 0)
		qep_err(tx_err, "%s: qep_unmap_free_pkt_data() failed\n",
			__func__);
	return ret;
}

/* This function is called by QDMA core when one or multiple packet
 * transmission is completed.
 * This function frees skb associated with the transmitted packets.
 */
static int qep_tx_done(struct qdma_request *req, unsigned int bytes_done,
			int err)
{
	int ret = 0;

	ret = qep_unmap_free_pkt_data(req);
	if (ret != 0)
		pr_err("%s: qep_unmap_free_pkt_data() failed\n", __func__);

	pr_debug("%s: bytes_done = %d, error = %d\n",
		__func__, bytes_done, err);

	return ret;
}

/* TX interrupt Handler Bottom Half */
static void qep_isr_tx_bottom_half(unsigned long data)
{
	struct qep_dma_q *tx_q_priv = (struct qep_dma_q *)data;
	struct qep_priv *xpriv = tx_q_priv->parent;

	/* Call queue service for QDMA Core to service queue,
	 * budget is not honored in case of Tx
	 */
	qdma_queue_service(xpriv->dev_handle, tx_q_priv->q_handle, 0, true);
}

/* This function is interrupt handler (TOP half).
 * once packet is written in QDMA queue, relevant interrupt will be generated.
 * The generated interrupt gets served from this handler
 */
static void qep_isr_tx_tophalf(unsigned long qhndl, unsigned long uld)
{
	u32 q_no;
	struct qep_priv *xpriv = (struct qep_priv *)(uld);

	/* If ISR is for Tx queue */
	q_no = (qhndl - xpriv->base_tx_q_handle);
#ifdef QEP_SW_COUNTER_EN
	xpriv->drv_stats->tx_int_cnt++;
#endif
	tasklet_schedule(&xpriv->task_tx_done[q_no]);

	qep_dbg(xpriv->netdev, "%s: Tx interrupt called. Mapped queue no = %d\n",
		__func__, q_no);
}
/* This function is RX interrupt handler (TOP half) */
static void qep_isr_rx_tophalf(unsigned long qhndl, unsigned long uld)
{
	u32 q_no;
	struct qep_priv *xpriv = (struct qep_priv *)(uld);

	/* ISR is for Rx queue */
	q_no = (qhndl - xpriv->base_rx_q_handle);
#ifdef QEP_SW_COUNTER_EN
	xpriv->drv_stats->rx_int_cnt++;
#endif
	napi_schedule(&xpriv->napi[q_no]);

	qep_dbg(xpriv->netdev, "%s: Rx interrupt called. Mapped queue no = %d\n",
		__func__, q_no);
}

/* This is deffered NAPI task for processing incoming Rx packet from DMA queue
 * This function will form sk_buff from Rx queue data and
 * pass it to above networking layers for processing
 */
static int qep_rx_poll(struct napi_struct *napi, int quota)
{
	int queue_id;
	unsigned long q_handle;
	unsigned int udd_cnt = 0, pkt_cnt = 0, data_len = 0;
	struct qep_priv *xpriv;
	struct net_device *netdev;
	int rv;

	if (!napi) {
		pr_err("%s: Invalid NAPI\n", __func__);
		return -EINVAL;
	}

	netdev = napi->dev;
	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}
#ifdef QEP_SW_COUNTER_EN
	xpriv->drv_stats->rx_napi_cnt++;
#endif
	queue_id = (int)(napi - xpriv->napi);
	q_handle = (xpriv->base_rx_q_handle + queue_id);

	/* Call queue service for QDMA Core to service queue */
	rv = qdma_queue_service(xpriv->dev_handle, q_handle, quota, true);
	/* Indicate napi_complete irrespective of rv */
	napi_complete(napi);
	if (rv < 0) {
		pr_debug("%s: qdma_queue_service for queue=%d returned status=%d\n",
				__func__, queue_id, rv);
		return rv;
	}

	if (xpriv->qdma_dev_conf.intr_moderation)
		qdma_queue_c2h_peek(xpriv->dev_handle, q_handle, &udd_cnt,
			&pkt_cnt, &data_len);

	qdma_queue_update_pointers(xpriv->dev_handle, q_handle);

	if (poll_mode || (pkt_cnt >= quota))
		napi_reschedule(napi);

	return 0;
}


/* This function validates Rx packet checksum */
static void qep_rx_cksum(struct qep_priv *xpriv, struct sk_buff *skb,
				struct stmn_cmpt_entry *cmpt_entry)
{
	struct net_device *netdev = xpriv->netdev;
	union c2h_metadata metadata;

	if (!cmpt_entry) {
		qep_err(rx_err, "%s: cmpt_entry is NULL\n", __func__);
		return;
	}

	metadata.user = cmpt_entry->metadata;
	skb->ip_summed = CHECKSUM_NONE;

	/* RX Checksum disabled */
	if (unlikely(!(netdev->features & NETIF_F_RXCSUM)))
		return;

	/* For fragmented packets checksum is invalid */
	if (metadata.is_ipfrag)
		return;

	if (metadata.hw_l4_chksum_valid
		&& !metadata.hw_l4_chksum_bad
		&& metadata.hw_l3_chksum_valid
		&& !metadata.hw_l3_chksum_bad
	) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		skb->csum_level = 1;
#ifdef QEP_SW_COUNTER_EN
		xpriv->drv_stats->rx_pkt_l3_csum++;
		xpriv->drv_stats->rx_pkt_l4_csum++;
		qep_dbg_csum_str(xpriv->netdev, "L4 L3 valid", metadata);
#endif
		return;
	}

	if (metadata.hw_l3_chksum_valid && !metadata.hw_l3_chksum_bad) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		skb->csum_level = 0;
#ifdef QEP_SW_COUNTER_EN
		xpriv->drv_stats->rx_pkt_l3_csum++;
		qep_dbg_csum_str(xpriv->netdev, "L3 valid", metadata);
#endif
		return;
	}
#ifdef QEP_SW_COUNTER_EN
	qep_dbg_csum_str(xpriv->netdev, "csum invalid", metadata);
#endif
}

/* This function creates skb and moves data from dma request to network domain*/
static int qep_rx_deliver(struct qep_priv *xpriv, u32 q_no, unsigned int len,
			unsigned int sgcnt, struct qdma_sw_sg *sgl, void *udd)
{
	struct net_device *netdev = xpriv->netdev;
	struct sk_buff *skb = NULL;
	struct qdma_sw_sg *c2h_sgl = sgl;
	struct stmn_cmpt_entry *cmpt_entry = NULL;

	if (!sgcnt) {
		qep_err(drv, "%s: SG Count is NULL\n", __func__);
		return -EINVAL;
	}
	if (!sgl) {
		qep_err(drv, "%s: SG List is NULL\n", __func__);
		return -EINVAL;
	}

	if (udd)
		cmpt_entry = (struct stmn_cmpt_entry *)udd;

#if (QEP_DBG_DUMP_EN)
	dump_qdma_sw_sgl(sgcnt, sgl);
#endif
	if (len <= QEP_RX_COPY_THRES) {
		skb = napi_alloc_skb(&xpriv->napi[q_no], len);
		if (unlikely(!skb)) {
			qep_err(rx_err, "%s: napi_alloc_skb() failed\n",
				__func__);
			return -ENOMEM;
		}

		skb_copy_to_linear_data(skb, page_address(c2h_sgl->pg), len);
		__skb_put(skb, len);
		__free_pages(c2h_sgl->pg, 0);
	} else {
		unsigned int nr_frags = 0;
		unsigned int frag_len;
		unsigned int frag_offset;

		/* Main body length for sk_buffs used for Rx Ethernet packets
		 * with fragments. Should be >= QEP_RX_PULL_LEN
		 */
		skb = napi_alloc_skb(&xpriv->napi[q_no], QEP_RX_PULL_LEN);
		if (unlikely(!skb)) {
			qep_err(rx_err, "%s: napi_alloc_skb() failed\n",
				__func__);
			return -ENOMEM;
		}

		skb_copy_to_linear_data(skb,
			page_address(c2h_sgl->pg + c2h_sgl->offset),
			QEP_RX_PULL_LEN);
		__skb_put(skb, QEP_RX_PULL_LEN);

		c2h_sgl->offset += QEP_RX_PULL_LEN;

		while (sgcnt && c2h_sgl) {
			frag_len = c2h_sgl->len;
			frag_offset = c2h_sgl->offset;
			skb_fill_page_desc(skb, nr_frags, c2h_sgl->pg,
					 frag_offset, frag_len - frag_offset);
			sgcnt--;
			c2h_sgl = c2h_sgl->next;
			nr_frags++;
		};

		skb->len = len;
		skb->data_len = len - QEP_RX_PULL_LEN;
		skb->truesize += skb->data_len;
	}

	skb->protocol = eth_type_trans(skb, netdev);
	skb_record_rx_queue(skb, q_no);

	/* Validate Checksum offload */
	qep_rx_cksum(xpriv, skb, cmpt_entry);

	skb_mark_napi_id(skb, &xpriv->napi[q_no]);

	netif_receive_skb(skb);

	return 0;
}

/* This function moves data from dma request to network domain when GRO enable*/
static int qep_rx_gro(struct qep_priv *xpriv, u32 q_no, unsigned int len,
			 unsigned int sgcnt, struct qdma_sw_sg *sgl, void *udd)
{
	struct qdma_sw_sg *l_sgl = sgl;
	unsigned int nr_frags = 0;
	unsigned int total_len = len;
	struct sk_buff *skb = xpriv->napi[q_no].skb;
	struct stmn_cmpt_entry *cmpt_entry = NULL;

	if (udd)
		cmpt_entry = (struct stmn_cmpt_entry *)udd;

	if (!skb) {
		skb = napi_get_frags(&xpriv->napi[q_no]);
		if (unlikely(!skb)) {
			qep_err(rx_err, "%s: napi_get_frags() failed\n",
				__func__);
			return -ENOMEM;
		}
		skb_record_rx_queue(skb, q_no);
	}
	nr_frags = skb_shinfo(skb)->nr_frags;
	total_len += skb->len;
	do {
		skb_fill_page_desc(skb, nr_frags, l_sgl->pg, l_sgl->offset,
				 l_sgl->len - l_sgl->offset);
		l_sgl = l_sgl->next;
		nr_frags++;
		sgcnt--;
	} while (sgcnt);

	skb->len = total_len;
	skb->data_len += len;
	skb->truesize += skb->data_len;

	/* Validate Checksum offload */
	qep_rx_cksum(xpriv, skb, cmpt_entry);

	napi_gro_frags(&xpriv->napi[q_no]);

	return 0;
}

/* This function process RX dma request */
static int qep_rx_pkt_process(unsigned long qhndl, unsigned long quld,
				 unsigned int len, unsigned int sgcnt,
				 struct qdma_sw_sg *sgl, void *udd)
{
	u32 q_no;
	int ret = 0;
	struct qdma_sw_sg *l_sgl = sgl;
	struct qep_priv *xpriv = (struct qep_priv *)quld;
	struct net_device *netdev = xpriv->netdev;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (!sgcnt) {
		qep_err(drv, "%s: SG Count is zero\n", __func__);
		return -EINVAL;
	}

	q_no = (qhndl - xpriv->base_rx_q_handle);
	if (netdev->features & NETIF_F_GRO)
		ret = qep_rx_gro(xpriv, q_no, len, sgcnt, sgl, udd);
	else
		ret = qep_rx_deliver(xpriv, q_no, len, sgcnt, sgl, udd);
	if (ret < 0) {
		while (sgcnt) {
			__free_pages(l_sgl->pg, 0);
			l_sgl = l_sgl->next;
			sgcnt--;
		}
	}

	xpriv->rx_qstats[q_no].rx_packets++;
	xpriv->rx_qstats[q_no].rx_bytes += len;

	qep_dbg(xpriv->netdev,
		"%s: q_no = %u, qhndl = %lu, len = %d processed\n", __func__,
		q_no, qhndl, len);

	return ret;
}

/* This function provides statistics to ifconfig command. */
#if KERNEL_VERSION(4, 10, 0) < LINUX_VERSION_CODE
static void qep_get_stats64(struct net_device *netdev,
			struct rtnl_link_stats64 *stats)
#else
static struct rtnl_link_stats64 *
qep_get_stats64(struct net_device *netdev, struct rtnl_link_stats64 *stats)
#endif
{
	u32 q_num = 0;
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
#if KERNEL_VERSION(4, 10, 0) < LINUX_VERSION_CODE
#else
		return -EINVAL;
#endif
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
#if KERNEL_VERSION(4, 10, 0) < LINUX_VERSION_CODE
#else
		return -EINVAL;
#endif
	}
	/* Added below check to avoid race condition while removing driver.
	 * Where qep_pci_remove gets call which frees cmac and then
	 * qep_get_stats64 is getting called which results in crash in kernel
	 */
	if ((xpriv != NULL) && (xpriv->cmac_dev.base_address) &&
	(xpriv->rx_qstats != NULL) && (xpriv->tx_qstats != NULL)) {
		for (q_num = 0; q_num < xpriv->netdev->real_num_rx_queues;
			q_num++) {
			stats->rx_bytes += xpriv->rx_qstats[q_num].rx_bytes;
			stats->rx_packets += xpriv->rx_qstats[q_num].rx_packets;
		}
		for (q_num = 0; q_num < xpriv->netdev->real_num_tx_queues;
			q_num++) {
			stats->tx_bytes += xpriv->tx_qstats[q_num].tx_bytes;
			stats->tx_packets += xpriv->tx_qstats[q_num].tx_packets;
		}
		xpriv->drv_stats->rx_bytes = stats->rx_bytes;
		xpriv->drv_stats->rx_packets = stats->rx_packets;
		xpriv->drv_stats->tx_bytes = stats->tx_bytes;
		xpriv->drv_stats->tx_packets = stats->tx_packets;
	}

#if KERNEL_VERSION(4, 10, 0) < LINUX_VERSION_CODE
#else
	return stats;
#endif
}

/* This function validated the ether address */
static int qep_validate_addr(struct net_device *netdev)
{
	struct qep_priv *xpriv;
#if defined(QEP_DESIGN)
	int ret = 0;
	static u8 mac_addr[ETH_ALEN];
#endif

	if (!netdev)
		return -EINVAL;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return -EINVAL;

	qep_dbg(xpriv->netdev, "Called %s()\n", __func__);

	/*Check that the Ethernet address (MAC) is not 00:00:00:00:00:00,
	 *is not a multicast address, and is not FF:FF:FF:FF:FF:FF
	 */
	if (!is_valid_ether_addr(netdev->dev_addr)) {
		qep_err(drv, "%s: MAC Address is zero or is multicast address\n",
			__func__);
		return -EINVAL;
	}

#if defined(QEP_DESIGN)
	ret = read_mac_hw(xpriv->bar_base, mac_addr);
	if (ret != 0) {
		qep_err(drv, "%s : MAC Address is not configured\n",
			__func__);
		return -EINVAL;
	}

	/*Compare device MAC address and HW MAC address*/
	if (!is_etherdev_addr(netdev, mac_addr)) {
		qep_err(drv, "%s: HW and netdev MAC addresses are different\n"
					, __func__);
		return -EINVAL;
	}
#endif
	return 0;
}

/* This function sets MAC address to the interface */
static int qep_set_mac_address(struct net_device *netdev, void *p)
{
	int ret = 0;
	struct sockaddr *address = p;
	char pre_dev_addr[ETH_ALEN];
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (!xpriv)
		return -EINVAL;

	qep_dbg(xpriv->netdev, "Called %s()\n", __func__);

	/*Preserve the exesting MAC address to assign back in case of error*/
	memcpy(pre_dev_addr, netdev->dev_addr, netdev->addr_len);

	ether_addr_copy(netdev->dev_addr, (u8 *)address->sa_data);

	ret = qep_validate_addr(netdev);
	if (ret) {
#if defined(QEP_DESIGN)
		qep_err(drv,
			"%s: Invalid MAC address or MAC address is different from the HW stored MAC address %d\n",
			__func__, (u8 *)address->sa_data);

#endif
		ether_addr_copy(netdev->dev_addr, pre_dev_addr);
		return -EADDRNOTAVAIL;
	}

	return 0;
}
static void print_qep_build_info(void __iomem *base)
{
	void __iomem *io_addr = base + QEP_BASE_OFFSET;
	u32 board = 0, build_id = 0, ts_ymd = 0, ts_hms = 0, version = 0;

	board = readl(io_addr + 0x4);
	build_id = readl(io_addr + 0x8);
	ts_ymd = readl(io_addr + 0xc);
	ts_hms = readl(io_addr + 0x10);
	version = readl(io_addr + 0x14);
	pr_info("QEP Info: Board:%x Build:%x YMD:%x HMS:%x version:%x\n",
			board, build_id, ts_ymd, ts_hms, version);

}
/* This function prints design version */
static void print_version(void __iomem *base)
{
	u32 ver, maj, min, sub_ver;

#if defined(QAP_DESIGN) || defined(QAP_STMN_DESIGN)
	ver = readl((void __iomem *)((u64)base + 0x14));
	min = ver & 0xffff;
	maj = (ver & 0xffff0000) >> 16;
	sub_ver = readl((void __iomem *)((u64)base + 0x8));
#else
	ver = readl((void __iomem *)((u64)base + QEP_VERSION_REG));
	sub_ver = ver & 0xffff;
	min = (ver & 0xff0000) >> 16;
	maj = (ver & 0xff000000) >> 24;
#endif
	pr_info("%s: Platform Version Major: %d Minor: %d Sub Version: %d\n",
			__func__, maj, min, sub_ver);
	print_qep_build_info(base);
}

/* This function reads MAC addr from HW */
#if !defined(QAP_DESIGN) && !defined(QAP_STMN_DESIGN)
static int read_mac_hw(void __iomem *base, u8 *mac_addr)
{
	u32 rlow, rhigh;

	rlow = readl((void __iomem *)((u64)base +
			QEP_USR_RX_META_LOW_R));
	rhigh = readl((void __iomem *)((u64)base +
			QEP_USR_RX_META_HIGH_R));

	if (rlow == 0 && rhigh == 0)
		return -EINVAL;

	pr_debug("REGLOW:%x REGHIGH:%x\n", rlow, rhigh);

	mac_addr[0] = (rhigh & 0xff00) >> 8;
	mac_addr[1] = rhigh & 0xff;

	mac_addr[2] = (rlow & 0xff000000) >> 24;
	mac_addr[3] = (rlow & 0xff0000) >> 16;
	mac_addr[4] = (rlow & 0xff00) >> 8;
	mac_addr[5] = rlow & 0xff;

	return 0;
}
#endif

/* This function changes MTU size of the interface */
static int qep_change_mtu(struct net_device *netdev, int new_mtu)
{
	int ret = 0;
	struct qep_priv *xpriv;
	unsigned int cmac_mtu;
	void __iomem *io_addr;
	bool is_dev_up = false;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (netif_running(netdev)) {
		is_dev_up = true;
#if KERNEL_VERSION(4, 7, 0) <= LINUX_VERSION_CODE
		netif_trans_update(netdev);
#endif
		ret = qep_stop(netdev);
		if (ret != 0) {
			qep_err(drv, "%s: qep_stop() failed with status %d\n",
				__func__, ret);
			return ret;
		}
	}

	if (new_mtu > QEP_MAX_MTU) {
		qep_err(rx_err,
			"%s: MTU = %d is greater than MAX MTU of %d supported\n",
			__func__, new_mtu, QEP_MAX_MTU);
		return -EINVAL;
	}

	if (new_mtu < QEP_MIN_MTU) {
		qep_err(rx_err,
			"%s: MTU = %d is smaller than MIN MTU of %d supported\n",
			__func__, new_mtu, QEP_MIN_MTU);
		return -EINVAL;
	}

	qep_dbg(xpriv->netdev, "%s: netdev mtu=%d new_mtu=%d\n", netdev->name,
		netdev->mtu, new_mtu);
	cmac_mtu = new_mtu + ETH_HLEN + ETH_FCS_LEN;

	io_addr = (void __iomem *)((u64)xpriv->bar_base +
			QEP_CMAC_DRP_OFFSET +
			QEP_CMAC_CTL_RX_MAX_PACKET_LEN_OFFSET);
	writel(cmac_mtu, io_addr);
	cmac_mtu = readl(io_addr);

	netdev->mtu = new_mtu;
	qep_info(drv, "%s: Netdev MTU:%d CMAC MTU = %d\n",
		 __func__, netdev->mtu, cmac_mtu);

	if (!is_dev_up)
		return ret;

	ret = qep_open(netdev);
	if (ret != 0) {
		qep_err(drv, "%s: qep_open() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	return ret;
}

/*This function is called when TX timeout occurs*/
static void qep_tx_timeout(struct net_device *netdev)
{
	struct qep_priv *xpriv;
	int ret = 0;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return;
	}

	qep_dbg(xpriv->netdev, "%s() called\n", __func__);

	/*Increment the TX timeout error count*/
	xpriv->drv_stats->tx_timeout_count++;

	if (netif_running(netdev)) {
		netif_device_detach(netdev);
		qep_dbg(xpriv->netdev,
			"%s: Restarting the device for TX timeout event\n",
			__func__);
		/*Restart the device*/
		ret = qep_stop(netdev);
		if (ret != 0) {
			qep_err(drv, "%s qep_stop() failed with status %d\n",
				__func__, ret);
			return;
		}
		ret = qep_open(netdev);
		if (ret != 0) {
			qep_err(drv, "%s: qep_open() failed with status %d\n",
				__func__, ret);
			return;
		}
		netif_device_attach(netdev);
	}

	/*Prevent TX timeout*/
	netif_trans_update(netdev);
	/*Allow upper layers to call the device hard_start_xmit routine*/
	netif_wake_queue(netdev);
}

/*This function is called to take the apropriate
 *action for the changed features. Kernel will update
 *the changed features into device features if this function
 *returns 0 otherwise the changed features are ignored
 */
static int qep_set_features(struct net_device *netdev,
			netdev_features_t features)
{
	struct qep_priv *xpriv;
	int ret = 0;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	qep_dbg(xpriv->netdev, "%s() called\n", __func__);

	if ((features & NETIF_F_RXHASH) !=
		(netdev->features & NETIF_F_RXHASH)) {
		if (netif_running(netdev)) {
			/*Restart the device */
			ret = qep_stop(netdev);
			if (ret != 0) {
				qep_err(drv,
					"%s: qep_stop() failed with status %d\n",
					__func__, ret);
				return ret;
			}
			netdev->features = features;
			ret = qep_open(netdev);
			if (ret != 0) {
				qep_err(drv,
					"%s: qep_open() failed with status %d\n",
					__func__, ret);
				return ret;
			}

		}
	}

	return 0;
}
/* Network Device Operations */
static const struct net_device_ops qep_netdev_ops = {
	.ndo_open = qep_open,
	.ndo_stop = qep_stop,
	.ndo_start_xmit = qep_start_xmit,
	.ndo_get_stats64 = qep_get_stats64,
	.ndo_set_mac_address = qep_set_mac_address,
	.ndo_change_mtu = qep_change_mtu,
	.ndo_validate_addr = qep_validate_addr,
	.ndo_tx_timeout = qep_tx_timeout,
	.ndo_set_features = qep_set_features
};

/* qep_setup - net device setup function
 * @netdev: Pointer to the net_device structure
 * This function sets up the net device for register
 */
static void qep_setup(struct net_device *netdev)
{
	/* Sets several net_device fields with appropriate values,
	 * It is called if alloc_etherdev is used instead of alloc_netdev
	 */
	ether_setup(netdev);

	netdev->netdev_ops = &qep_netdev_ops;

#if KERNEL_VERSION(4, 10, 0) <= LINUX_VERSION_CODE
	netdev->max_mtu = QEP_MAX_MTU;
#else
	netdev->mtu = QEP_MAX_MTU;
#endif

	qep_set_ethtool_ops(netdev);
}

/* This function Initializes STMN IP */
static int qep_stmn_init(struct qep_priv *xpriv, unsigned short dev_id)
{
	int ret;
	void __iomem *io_addr;

	xpriv->tm_dev.xdev = xpriv;
	xpriv->tm_dev.bar_num = QEP_PCI_USR_BAR;
	xpriv->tm_dev.stm_regs = (void *)((u64)xpriv->bar_base +
			QEP_STMN_BAR_OFFSET);

	/* reset h2c and c2h stmn cores */
	io_addr = (void __iomem *)((u64)xpriv->bar_base +
			QEP_BASE_OFFSET);
	writel(1, (void __iomem *)((u64)io_addr +
			QEP_BASE_DMA_USER_H2C_OFFSET));
	writel(0, (void __iomem *)((u64)io_addr +
			QEP_BASE_DMA_USER_H2C_OFFSET));

	writel(1, (void __iomem *)((u64)io_addr +
			QEP_BASE_DMA_USER_C2H_OFFSET));
	writel(0, (void __iomem *)((u64)io_addr +
			QEP_BASE_DMA_USER_C2H_OFFSET));

	ret = stmn_initialize(xpriv, &(xpriv->tm_dev), dev_id);
	if (ret < 0) {
		qep_err(drv, "%s: Unable to init STMN on BAR %d\n", __func__,
			 QEP_PCI_USR_BAR);
		return ret;
	}

	ret = stmn_write_c2h_buf_size(xpriv, QEP_DEFAULT_C2H_BUFFER_SIZE);
	if (ret < 0) {
		qep_err(drv, "%s: Unable to Set STMN C2h BUF Size\n",
			 __func__);
		return ret;
	}

	return ret;
}

/* This is probe function which is called when Linux kernel detects PCIe device
 * with Vendor ID and Device ID listed in the in qep_pci_ids table.
 * From this function netdevice and device initialization are done
 */
static int qep_pci_probe(struct pci_dev *pdev,
			 const struct pci_device_id *pci_dev_id)
{
#if defined(QAP_DESIGN) || defined(QAP_STMN_DESIGN)
	u8 rand_byte[3];
#endif
	u8 mac_addr[ETH_ALEN] = {0x00, 0x5D, 0x03, 0x00, 0x00, 0x02};
	unsigned short int nb_queues;
	int ret;
	int cpu_count;
	int num_msix;
	int q_num;
	u64 bar_start;
	u64 bar_len;
	struct net_device *netdev;
	struct qep_priv *xpriv;

	cpu_count = num_online_cpus();
	num_msix = pci_msix_vec_count(pdev);
	/* Number of queues used is the minimum of available CPUs and MSI-X */
	nb_queues = min_t(int, cpu_count, num_msix);

	netdev = alloc_netdev_mqs(sizeof(struct qep_priv), "xeth%d",
#if KERNEL_VERSION(3, 17, 0) <= LINUX_VERSION_CODE
				NET_NAME_ENUM,
#endif
				qep_setup,
				QEP_NUM_MAX_QUEUES,
				QEP_NUM_MAX_QUEUES);

	if (!netdev) {
		pr_err("%s: alloc_netdev_mqs() failed\n", __func__);
		return -ENODEV;
	}

	/*Set TX timeout interval*/
	netdev->watchdog_timeo = QEP_TX_TIMEOUT;

	/*Set TX queues */
	ret = netif_set_real_num_tx_queues(netdev, nb_queues);
	if (ret != 0) {
		pr_err("%s: netif_set_real_num_tx_queues() failed with status %d\n",
			__func__, ret);
		goto exit;
	}

	/*Set RX queues */
	ret = netif_set_real_num_rx_queues(netdev, nb_queues);
	if (ret != 0) {
		pr_err("%s: netif_set_real_num_rx_queues() failed with status %d\n",
			__func__, ret);
		goto exit;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);

	/* Initialize Driver private data */
	xpriv = netdev_priv(netdev);
	xpriv->netdev = netdev;
	xpriv->pcidev = pdev;
	xpriv->is_cmac_on = 0;
	xpriv->rs_fec_en = QEP_RS_FEC_EN;
	xpriv->msg_enable = netif_msg_init(debug, QEP_DEFAULT_MSG_ENABLE);

	ret = qep_debugfs_dev_init(xpriv);
	if (ret < 0)
		qep_warn(probe,
			"%s: qep_debugfs_dev_init() failed with status %d\n",
			__func__, ret);

	ret = qep_set_netdev_features(netdev);
	if (ret != 0) {
		qep_err(probe,
			"%s: qep_set_netdev_features() failed with status %d\n",
			__func__, ret);
		goto err_exit;
	}

	/* Map the User BAR */
	bar_start = pci_resource_start(pdev, QEP_PCI_USR_BAR);
	bar_len = pci_resource_len(pdev, QEP_PCI_USR_BAR);
	xpriv->bar_base = ioremap(bar_start, bar_len);
	if (!xpriv->bar_base) {
		qep_err(probe, "%s: ioremap() failed for BAR%d\n", __func__,
			QEP_PCI_USR_BAR);
		ret = -EIO;
		goto err_exit;
	}

	print_version(xpriv->bar_base);

#if defined(QAP_DESIGN) || defined(QAP_STMN_DESIGN)
	get_random_bytes(rand_byte, 3);
	memcpy(mac_addr + 3, rand_byte, 3);
#else
	/* Read the MAC address from HW that was programmed from Mgmt-PF */
	ret = read_mac_hw(xpriv->bar_base, mac_addr);
	if (ret != 0) {
		qep_err(probe, "%s: read_mac_hw() failed with status %d\n",
			__func__, ret);
		goto err_exit;
	}
#endif
	memcpy(netdev->dev_addr, mac_addr, netdev->addr_len);

	/* Reset User Logic */
	ret = qep_reset_all(xpriv);
	if (ret != 0) {
		qep_err(probe, "%s: qep_reset_all() failed with status %d\n",
			__func__, ret);
		goto err_exit;
	}

	/* Initialize Streaming Traffic Manager for Networking */
	ret = qep_stmn_init(xpriv, pdev->device);
	if (ret != 0) {
		qep_err(probe, "%s: qep_stmn_init() failed with status %d\n",
			__func__, ret);
		goto err_exit;
	}

	/* Initialize RSS Redirection Table */
	if (xpriv->netdev->features & NETIF_F_RXHASH) {
		ret = qep_init_reta(xpriv, 0,
				xpriv->netdev->real_num_rx_queues);
		if (ret != 0) {
			qep_err(probe, "%s: qep_init_reta() failed with status %d\n",
					__func__, ret);
			goto err_exit;
		}
	}

	ret = qep_qdma_setup(xpriv);
	if (ret != 0) {
		qep_err(probe, "%s: qep_qdma_setup() failed with status %d\n",
			__func__, ret);
		goto err_exit;
	}

	xpriv->tx_q = kcalloc(QEP_NUM_MAX_QUEUES * 2, sizeof(struct qep_dma_q),
			GFP_KERNEL);
	if (!xpriv->tx_q)
		goto close_qdma_device;

	xpriv->rx_q = xpriv->tx_q + QEP_NUM_MAX_QUEUES;
	for (q_num = 0; q_num < QEP_NUM_MAX_QUEUES; q_num++) {
		/* Adaptive TX not supported */
		xpriv->tx_q[q_num].adaptive_update = 0;
		xpriv->tx_q[q_num].coalesce_frames =
				QEP_DEFAULT_H2C_COUNT_THRESHOLD;
		xpriv->tx_q[q_num].coalesce_usecs = QEP_DEFAULT_H2C_TIMER_COUNT;
		xpriv->rx_q[q_num].adaptive_update = 1;
		xpriv->rx_q[q_num].coalesce_frames =
				QEP_DEFAULT_C2H_COUNT_THRESHOLD;
		xpriv->rx_q[q_num].coalesce_usecs =
				 QEP_DEFAULT_C2H_TIMER_COUNT;
	}

	/* Setup 100G MAC */
	ret = qep_cmac_setup(xpriv);
	if ((ret != 0) && (ret != -EAGAIN)) {
		qep_err(probe,
			"%s: qep_cmac_setup() failed with status %d\n",
			__func__, ret);
		goto free_mem;
	}

	spin_lock_init(&xpriv->lnk_state_lock);
	spin_lock_init(&xpriv->config_lock);
	init_waitqueue_head(&xpriv->link_mon_needed);

	/* Start link thread to monitor CMAC link status */
	ret = qep_thread_start(xpriv);
	if (ret != 0) {
		qep_err(probe,
			"%s: qep_thread_start() failed with status %d\n",
			__func__, ret);
		goto free_mem;
	}

	ret = register_netdev(netdev);
	if (ret != 0) {
		qep_err(probe, "%s: Failed to register network driver\n",
			__func__);
		goto close_thread;
	}

	spin_lock(&xpriv->lnk_state_lock);
	xpriv->dev_state = QEP_DEV_STATE_DOWN;
	netif_carrier_off(xpriv->netdev);
	spin_unlock(&xpriv->lnk_state_lock);

	qep_info(drv, "%s : Device probe done\n", __func__);
	return 0;

close_thread:
	qep_thread_stop(xpriv);
free_mem:
	kfree(xpriv->tx_q);
close_qdma_device:
	qdma_device_close(pdev, xpriv->dev_handle);
err_exit:
	qep_debugfs_dev_exit(xpriv);
exit:
	kfree(netdev);
	return ret;
}

/* This function gets called when PCIe device has been removed from the bus
 * This function cleans up all initialized components of the driver
 */
static void qep_pci_remove(struct pci_dev *pdev)
{
	int ret;
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct qep_priv *xpriv;
	struct clcr_config config;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return;
	}

	/* Stop link thread if it is running */
	qep_thread_stop(xpriv);

	if (loopback_en)
		qep_cmac_loopback_en(xpriv, false);

	/* stop CMAC */
	ret = qep_cmac_stop(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_cmac_stop failed with status %d\n",
			__func__, ret);
	}
	pci_set_drvdata(pdev, NULL);
	unregister_netdev(netdev);

	config.mode = CLCR_FIXED_MODE;
	config.fixed.qnum = 0;
	ret = qep_clcr_setup(xpriv, &config);
	if (ret != 0)
		qep_err(drv, "%s: CLCR setup failed with status %d\n",
			__func__, ret);

	ret = xcmac_deintialize(&xpriv->cmac_dev);
	if (ret != 0)
		qep_err(drv, "%s: xcmac_deintialize failed with satus %d\n",
			__func__, ret);

	kfree(xpriv->tx_q);

	qdma_device_close(pdev, xpriv->dev_handle);
	debugfs_remove_recursive(xpriv->debugfs_dev_root);

	if (xpriv->bar_base)
		iounmap(xpriv->bar_base);

	free_netdev(netdev);
}

/*Power management ops*/
int qep_suspend(struct device *dev)
{
	struct qep_priv *xpriv;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);
	int ret = 0;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	rtnl_lock();

	/* Mark device as removed */
	netif_device_detach(netdev);

	ret = qep_stop(netdev);
	if (ret) {
		qep_err(drv, "%s: qep_stop() failed with status %d\n", __func__,
				ret);
	}

	rtnl_unlock();
	pci_save_state(pdev);
	return pci_set_power_state(pdev, PCI_D3hot);
}

int qep_resume(struct device *dev)
{
	struct qep_priv *xpriv;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);
	int ret = 0;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	ret = pci_set_power_state(pdev, PCI_D0);
	if (ret) {
		qep_err(drv, "%s: pci_set_power_state() failed with status %d\n",
			__func__, ret);
		return ret;
	}
	pci_restore_state(pdev);

	rtnl_lock();

	ret = qep_open(netdev);
	if (ret != 0) {
		rtnl_unlock();
		netif_device_attach(xpriv->netdev);
		pci_set_power_state(pdev, PCI_D0);
		qep_err(drv, "%s qep_open() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	netif_device_attach(xpriv->netdev);

	rtnl_unlock();

	return 0;
}

static int qep_pci_pm_freeze(struct device *dev)
{
	struct qep_priv *xpriv;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	qep_info(drv, "%s() called\n", __func__);

	return qep_suspend(dev);
}

static int qep_pci_pm_thaw(struct device *dev)
{
	struct qep_priv *xpriv;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	qep_info(drv, "%s() called\n", __func__);

	return qep_resume(dev);

}

static int qep_pci_pm_poweroff(struct device *dev)
{
	struct qep_priv *xpriv;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	qep_info(drv, "%s() called\n", __func__);

	return qep_suspend(dev);
}

static int qep_pci_pm_resume(struct device *dev)
{
	struct qep_priv *xpriv;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	qep_info(drv, "%s() called\n", __func__);

	return qep_resume(dev);
}

static int qep_pci_pm_restore(struct device *dev)
{
	struct qep_priv *xpriv;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	qep_info(drv, "%s() called\n", __func__);

	return qep_resume(dev);
}

static int qep_pci_pm_suspend(struct device *dev)
{
	struct qep_priv *xpriv;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);
	int ret = 0;

	if (!netdev) {
		pr_err("%s: netdev is NULL\n", __func__);
		return -EINVAL;
	}

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	qep_info(drv, "%s() called\n", __func__);

	ret = qep_suspend(dev);
	if (ret)
		ret = qep_resume(dev);
	return ret;
}

static const struct dev_pm_ops qep_pm_ops = {
	.suspend	= qep_pci_pm_suspend,
	.resume		= qep_pci_pm_resume,
	.freeze		= qep_pci_pm_freeze,
	.thaw		= qep_pci_pm_thaw,
	.poweroff	= qep_pci_pm_poweroff,
	.restore	= qep_pci_pm_restore,
};
/* This function handles PCI bus related errors
 * This function gets called if any PCI related errors are detected
 */
static pci_ers_result_t qep_pci_error_detected(struct pci_dev *pdev,
						pci_channel_state_t state)
{
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);
	int ret = 0;

	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return PCI_ERS_RESULT_DISCONNECT;
	}
	rtnl_lock();

	switch (state) {
	case pci_channel_io_normal:
		rtnl_unlock();
		return PCI_ERS_RESULT_CAN_RECOVER;

	case pci_channel_io_frozen:
		qep_dev_warn(
			"%s: Frozen state error, reset controller\n", __func__);
		/* Mark device as removed */
		netif_device_detach(xpriv->netdev);
		if (netif_running(xpriv->netdev)) {
			ret = qep_stop(xpriv->netdev);
			if (ret != 0)
				qep_dev_err("%s: qep_stop() failed with status %d\n",
					__func__, ret);
		}
		pci_disable_device(pdev);
		rtnl_unlock();
		return PCI_ERS_RESULT_NEED_RESET;

	case pci_channel_io_perm_failure:
		qep_dev_warn(
			"%s: Failure state error, req. disconnect\n", __func__);
		if (netif_running(xpriv->netdev)) {
			ret = qep_stop(xpriv->netdev);
			if (ret != 0)
				qep_dev_err("%s: qep_stop() failed with status %d\n",
					__func__, ret);
		}
		rtnl_unlock();
		return PCI_ERS_RESULT_DISCONNECT;
	}

	rtnl_unlock();
	return PCI_ERS_RESULT_NEED_RESET;
}

/* This function handles PCI reset request */
static pci_ers_result_t qep_pci_slot_reset(struct pci_dev *pdev)
{
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);

	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return PCI_ERS_RESULT_DISCONNECT;
	}
	qep_dev_info("%s: pdev = 0x%p, xpriv = 0x%p restart after slot reset\n",
		__func__, pdev, xpriv);
	if (pci_enable_device_mem(pdev)) {
		qep_dev_err("%s: failed to renable after slot reset\n",
				__func__);
		return PCI_ERS_RESULT_DISCONNECT;
	}

	pci_set_master(pdev);
	pci_restore_state(pdev);
	pci_save_state(pdev);

	return PCI_ERS_RESULT_RECOVERED;
}

/* This function handles PCI resume request */
static void qep_pci_error_resume(struct pci_dev *pdev)
{
	int ret = 0;
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);

	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return;
	}

	qep_dev_info("%s: pdev = 0x%p, xpriv = 0x%p.\n", __func__, pdev, xpriv);
	pci_cleanup_aer_uncorrect_error_status(pdev);

	if (netif_running(xpriv->netdev)) {
		ret = qep_open(xpriv->netdev);
		if (ret) {
			qep_dev_err(
				"%s: Device initialization failed after reset\n",
				__func__);
			return;
		}
	} else {
		qep_dev_err("%s: Device was not running prior to EEH\n",
			__func__);
	}
	netif_device_attach(xpriv->netdev);
}

#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
/* This function handles PCI reset request */
static void qep_pci_reset_prepare(struct pci_dev *pdev)
{
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);
	int ret = 0;

	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return;
	}
	qep_dev_info("%s: %s pdev = 0x%p, xpriv = 0x%p, hndl = 0x%lx.\n",
			__func__, dev_name(&pdev->dev), pdev, xpriv,
			xpriv->dev_handle);

	qdma_device_offline(pdev, xpriv->dev_handle, 1);
	qdma_device_flr_quirk_set(pdev, xpriv->dev_handle);
	if (netif_running(xpriv->netdev)) {
		ret = qep_stop(xpriv->netdev);
		if (ret != 0)
			qep_dev_err("%s: qep_stop() failed with status %d\n",
				__func__, ret);
	}
	qdma_device_flr_quirk_check(pdev, xpriv->dev_handle);
}

/* This function handles reset done */
static void qep_pci_reset_done(struct pci_dev *pdev)
{
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);
	int ret = 0;

	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return;
	}
	qep_dev_info("%s: %s pdev = 0x%p, xpriv = 0x%p, hndl = 0x%lx.\n",
			__func__, dev_name(&pdev->dev), pdev, xpriv,
			xpriv->dev_handle);
	qdma_device_online(pdev, xpriv->dev_handle, 1);
	if (netif_running(xpriv->netdev)) {
		ret = qep_open(xpriv->netdev);
		if (ret != 0) {
			qep_dev_err("%s: qep_open() failed with status %d\n",
				__func__, ret);
		}
	}
}

#elif KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
/* This function handles PCI reset request */
static void qep_pci_reset_notify(struct pci_dev *pdev, bool prepare)
{
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);
	int ret = 0;

	if (!xpriv) {
		pr_err("%s: xpriv is NULL\n", __func__);
		return;
	}
	qep_dev_info("%s: pdev = 0x%p, xpriv = 0x%p, prepare = %d.\n",
		__func__, pdev, xpriv, prepare);

	if (prepare) {
		qdma_device_offline(pdev, xpriv->dev_handle, 1);
		qdma_device_flr_quirk_set(pdev, xpriv->dev_handle);
		if (netif_running(xpriv->netdev)) {
			ret = qep_stop(xpriv->netdev);
			qep_dev_err("%s: qep_stop() failed with status %d\n",
				__func__, ret);
		}
		qdma_device_flr_quirk_check(pdev, xpriv->dev_handle);
	} else {
		qdma_device_online(pdev, xpriv->dev_handle, 1);
		if (netif_running(xpriv->netdev)) {
			ret = qep_open(xpriv->netdev);
			qep_dev_err("%s: qep_open() failed with status %d\n",
				__func__, ret);
		}
	}
}
#endif

/* PCIe Error handlers */
static const struct pci_error_handlers qep_pci_err_handler = {
	.error_detected = qep_pci_error_detected,
	.slot_reset = qep_pci_slot_reset,
	.resume = qep_pci_error_resume,
#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
	.reset_prepare = qep_pci_reset_prepare,
	.reset_done = qep_pci_reset_done,
#elif KERNEL_VERSION(3, 16, 0) <= LINUX_VERSION_CODE
	.reset_notify = qep_pci_reset_notify,
#endif
};

/* PCIe Driver */
static struct pci_driver qep_pci_driver = {
		.name = DRV_NAME,
		.id_table = qep_pci_ids,
		.probe = qep_pci_probe,
		.remove = qep_pci_remove,
		.driver.pm = &qep_pm_ops,
		.err_handler = &qep_pci_err_handler
};

/* This is the entry point of the NIC driver.
 * This function will get called on insert of the driver into the kernel
 */
static int __init qep_module_init(void)
{
	int err = 0;
	enum qep_design_support des_idx;


#ifdef QAP_DESIGN
	des_idx = QAP;
#elif defined(QAP_STMN_DESIGN)
	des_idx = QAP_STMN;
#else
	des_idx = QEP;
#endif

	pr_info("%s: Initializing %s Version %s\n", design_name[des_idx],
			DRV_DESC, DRV_VER);

	/* Initialize Debug File System */
	err = qep_debugfs_init();
	if (err != 0) {
		pr_err(" %s: qep_debugfs_init() failed\n", __func__);
		return err;
	}

	/* Initialize QDMA Library */
	err = libqdma_init(0, qep_debugfs_root);
	if (err != 0) {
		pr_err("%s: libqdma_init() failed\n", __func__);
		qep_debugfs_exit();
		return err;
	}

	err = pci_register_driver(&qep_pci_driver);
	if (err < 0) {
		pr_err("%s: PCI registration failed with status %d\n",
			__func__, err);
		libqdma_exit();
		qep_debugfs_exit();
	}

	return err;
}

/* This is the exit point of the NIC driver.
 * This function will get called on remove of the driver from the kernel
 */
static void __exit qep_module_exit(void)
{
	pci_unregister_driver(&qep_pci_driver);
	libqdma_exit();
	qep_debugfs_exit();
	pr_info("%s: Exiting %s Version %s\n", __func__, DRV_DESC, DRV_VER);
}

module_init(qep_module_init);
module_exit(qep_module_exit);

MODULE_AUTHOR("Xilinx");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRV_DESC);
MODULE_VERSION(DRV_VER);
