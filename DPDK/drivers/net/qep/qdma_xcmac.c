/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2019 Xilinx, Inc. All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <unistd.h>

#include "qdma.h"
#include "qdma_xcmac.h"
#include <rte_ethdev_driver.h>
#include <rte_alarm.h>

/*
 *  This function initialize and setup CMAC ip
 */
int xcmac_setup(struct rte_eth_dev *dev)
{
	int ret = 0, retry = 100;
	struct qdma_pci_dev *dma_priv;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	/* reset release of base */
	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
		(QEP_BASE_OFFSET + QEP_BASE_CMAC_RESET_CONTROL_OFFSET), 1);

	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
		(QEP_BASE_OFFSET + QEP_BASE_CMAC_RESET_CONTROL_OFFSET), 0);

	//sleep(.1);
	sleep(.5);

	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
		       (QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_RST_OFFSET),
		       (1 << QEP_CMAC_CTRL_RST_TX_GTWIZ_SHIFT));

	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
		       (QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_RST_OFFSET), 0);

	//sleep(.1);
	sleep(.5);

	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
		       (QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_RST_OFFSET),
		       (1 << QEP_CMAC_CTRL_RST_RX_GTWIZ_SHIFT));

	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
		       (QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_RST_OFFSET), 0);

	while (retry != 0) {
		//sleep(.05);
		sleep(.5);
		ret = qdma_read_reg(
			(uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
			(QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_ERR_OFFSET));
		PMD_DRV_LOG(INFO, "%s: CMAC_err = %x\n", __func__, ret);
		if (ret == 0) {
			PMD_DRV_LOG(INFO, "CMAC Reset done\n");
			break;
		}
		PMD_DRV_LOG(INFO, "CMAC Reset is not done. Retrying !!!\n");
		retry--;

	}

	if (retry == 0)
		PMD_DRV_LOG(INFO, "CMAC Reset is not done.!!!!\n");

	ret = xcmac_initialize(&dma_priv->cmac_instance,
			((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx])
			 + CMAC_100G_IP_BASE));
	if (ret != 0) {
		PMD_DRV_LOG(INFO, "%s: Error in xcmac_initialize\n", __func__);
		return ret;
	}

	ret = xcmac_reset_gt(&dma_priv->cmac_instance);
	if (ret != 0) {
		PMD_DRV_LOG(INFO, "%s: Error in resetting CMAC GT\n", __func__);
		return ret;
	}

	ret = xcmac_reset_core(&dma_priv->cmac_instance, XCMAC_CORE_BOTH);
	if (ret != 0) {
		PMD_DRV_LOG(INFO, "%s: Error in resetting CMAC cores\n",
			    __func__);
		return ret;
	}

#ifdef ENABLE_CMAC_LOOPBACK
	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
		       (QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_GT_OFFSET),
		       (QEP_CMAC_CTRL_GT_LOOPBACK));
#endif //ENABLE_CMAC_LOOPBACK

	if (dma_priv->rsfec) {
		/* Enable RS-FEC */
		qdma_write_reg(
			(uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
			(CMAC_100G_IP_BASE + XCMAC_RSFEC_CONFIG_ENABLE_OFFSET),
			(XCMAC_RSFEC_CONFIG_RX_ENABLE_MASK |
			XCMAC_RSFEC_CONFIG_TX_ENABLE_MASK));
	}

	ret = xcmac_start(dev);
	if (ret != 0) {
		PMD_DRV_LOG(INFO, "%s: Error in starting CMAC cores\n",
			    __func__);
		return ret;
	}

	xcmac_set_mtu(dev, ETHER_MAX_LEN);
	return ret;
}

/* This function enables CMAC IP */
int xcmac_start(struct rte_eth_dev *dev)
{
	int ret = 0;
	uint32_t reg_val = 0;
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;

	if (qdma_dev->is_cmac_on == 1)
		return 0;

	ret = xcmac_enable(&qdma_dev->cmac_instance, XCMAC_CORE_RX);
	if (ret != 0) {
		PMD_DRV_LOG(INFO, "%s: Error in enabling CMAC Rx core\n",
			    __func__);
		return ret;
	}

	ret = xcmac_set_fault_indication(&qdma_dev->cmac_instance, true);
	if (ret != 0) {
		PMD_DRV_LOG(INFO, "%s: Error in setting Tx fault indication\n",
			    __func__);
		return ret;
	}

	ret = xcmac_enable(&qdma_dev->cmac_instance, XCMAC_CORE_TX);
	if (ret != 0) {
		PMD_DRV_LOG(INFO, "%s: Error in enabling CMAC Tx core\n",
			    __func__);
		return ret;
	}

	ret = xcmac_set_fault_indication(&qdma_dev->cmac_instance, false);
	if (ret != 0) {
		PMD_DRV_LOG(INFO, "%s: Error in clearing Tx fault indication\n",
			    __func__);
		return ret;
	}

	/* Start TX output for peer to realize that link is up */
	reg_val = qdma_read_reg(
			(uint64_t)(qdma_dev->bar_addr[qdma_dev->user_bar_idx]) +
			(QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_TX_CTRL_OFFSET));
	reg_val = reg_val & ~(1 << QEP_CMAC_CTRL_TX_CTRL_TX_INHIBIT_SHIFT);
	qdma_write_reg((uint64_t)(qdma_dev->bar_addr[qdma_dev->user_bar_idx]) +
		(QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_TX_CTRL_OFFSET), reg_val);

	qdma_dev->is_cmac_on = 1;

	return ret;
}

/* This function disables CMAC IP */
int xcmac_stop(struct rte_eth_dev *dev)
{
	int ret = 0;
	uint32_t reg_val = 0;
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;

	if (qdma_dev->is_cmac_on == 0)
		return 0;

	qdma_dev->is_cmac_on = 0;

	/* Stop TX output for peer to realize link is down */
	reg_val = qdma_read_reg(
			(uint64_t)(qdma_dev->bar_addr[qdma_dev->user_bar_idx]) +
			(QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_TX_CTRL_OFFSET));
	reg_val = reg_val | (1 << QEP_CMAC_CTRL_TX_CTRL_TX_INHIBIT_SHIFT);
	qdma_write_reg((uint64_t)(qdma_dev->bar_addr[qdma_dev->user_bar_idx]) +
		(QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_TX_CTRL_OFFSET), reg_val);

	ret = xcmac_disable(&qdma_dev->cmac_instance, XCMAC_CORE_RX);
	if (ret != 0) {
		PMD_DRV_LOG(INFO, "%s: Error in disabling CMAC Rx core\n",
			    __func__);
		return ret;
	}

	ret = xcmac_disable(&qdma_dev->cmac_instance, XCMAC_CORE_TX);
	if (ret != 0) {
		PMD_DRV_LOG(INFO, "%s: Error in disabling CMAC Tx core\n",
			    __func__);
		return ret;
	}

	return ret;
}

void xcmac_set_mtu(struct rte_eth_dev *dev, uint16_t new_mtu)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;

	qdma_write_reg((uint64_t)(qdma_dev->bar_addr[qdma_dev->user_bar_idx]) +
			(QEP_CMAC_DRP_OFFSET +
			 QEP_CMAC_CTL_RX_MAX_PACKET_LEN_OFFSET),
			new_mtu);

}
