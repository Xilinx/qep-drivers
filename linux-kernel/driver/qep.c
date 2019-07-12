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
#include <net/busy_poll.h>

#include "qep.h"
#include "qdma_ul_ext_stmn.h"

/* Module parameter debug which enables debug prints based on lavels */
static int debug = -1;
module_param(debug, int, 0660);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

/* Module parameter defines direct or indirect interrupt mode */
unsigned short int ind_intr_mode;
module_param(ind_intr_mode, short, 0660);

static unsigned short int poll_mode;
module_param(poll_mode, short, 0660);

static unsigned short int nb_queues = QEP_NUM_DEFAULT_QUEUES;
module_param(nb_queues, short, 0660);

static unsigned short int rs_fec_en = QEP_DEFAULT_RS_FEC;
module_param(rs_fec_en, short, 0660);

static unsigned short int queue_base = QEP_QUEUE_BASE;
module_param(queue_base, short, 0660);

static unsigned short int ring_size = QEP_DEFAULT_RING_SIZE;
module_param(ring_size, short, 0660);

static unsigned short int loopback_en = QEP_EN_PMA_LOOPBACK;
module_param(loopback_en, short, 0660);

/* Module parameter c2h_byp_mode which defines C2H bypass mode */
#ifdef STMN_ENABLE
static unsigned short int c2h_byp_mode = 2;
#else
static unsigned short int c2h_byp_mode;
#endif
module_param(c2h_byp_mode, short, 0660);

/* Module parameter h2c_byp_mode which defines H2C bypass mode */
#ifdef STMN_ENABLE
static unsigned short int h2c_byp_mode = 1;
#else
static unsigned short int h2c_byp_mode;
#endif
module_param(h2c_byp_mode, short, 0660);
/*****************************************************************************/
/* PCI id for devices  */
static const struct pci_device_id qep_pci_ids[] = {
#ifdef QEP_ONLY_DESIGN
	{
		PCI_DEVICE(0x10ee, 0x903f),
	},
#else
	{
		PCI_DEVICE(0x10ee, 0x7002),
	},
#endif
	{
		0,
	}
};
MODULE_DEVICE_TABLE(pci, qep_pci_ids);

static u8 mac_addr[6] = { 0x00, 0x5D, 0x03, 0x00, 0x00, 0x02 };
static u8 qep_device_count;

static int qep_tx_done(struct qdma_request *, unsigned int bytes_done, int err);
static int qep_rx_poll(struct napi_struct *napi, int quota);
static int qep_rx_pkt_process(unsigned long qhndl, unsigned long quld,
			      unsigned int len, unsigned int sgcnt,
			      struct qdma_sw_sg *sgl, void *udd);
static void qep_isr_tx_bottom_half(unsigned long data);
static void qep_isr_tx_tophalf(unsigned long qhndl, unsigned long uld);
static void qep_isr_rx_tophalf(unsigned long qhndl, unsigned long uld);


void stmn_reg_write(void *dev_hndl, uint32_t reg_offst, uint32_t val)
{
	void __iomem *base = ((struct qep_priv *)dev_hndl)->tm_dev.stm_regs;

	writel(val, base + reg_offst);
}

uint32_t stmn_reg_read(void *dev_hndl, uint32_t reg_offst)
{
	u32 val;
	void __iomem *base = ((struct qep_priv *)dev_hndl)->tm_dev.stm_regs;

	val = readl(base + reg_offst);
	return val;
}

/*
 * Thread function routine which keeps polling of
 * link status in background in every QEP_LINK_CHECK_INTERVAL
 */

enum cmac_link_state { CMAC_LINK_DOWN = 0, CMAC_LINK_UP, CMAC_LINK_READ_ERROR };

static enum cmac_link_state qep_cmac_link_status(struct xcmac *inst)
{
	int ret = 0;
	struct xcmac_rx_lane_am_status lane_status;

	ret = xcmac_get_rx_lane_status(inst, &lane_status);
	if (ret != 0)
		return CMAC_LINK_READ_ERROR;

	if (lane_status.aligned == true)
		return CMAC_LINK_UP;

	return CMAC_LINK_DOWN;
}
static int qep_thread_monitor_link(void *th_arg)
{
	int retry = 0;
	struct qep_priv *xpriv;
	struct net_device *netdev = (struct net_device *)th_arg;
	enum cmac_link_state ls = CMAC_LINK_DOWN;

	if (!netdev)
		return -EINVAL;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return -EINVAL;

	while (!kthread_should_stop()) {
		wait_event_interruptible(xpriv->link_mon_needed,
			((xpriv->dev_state == QEP_DEV_STATE_UP) ||
			(xpriv->dev_state == QEP_DEV_STATE_EXIT)));
		retry = 100;
		while (retry-- && !kthread_should_stop()) {
			ls = qep_cmac_link_status(&xpriv->cmac_instance);
			if (ls == CMAC_LINK_UP)
				break;
			usleep_range(5000, 10000);
		}

		if (ls == CMAC_LINK_UP && !netif_carrier_ok(netdev)) {
			qep_info(link, "%s: Enabling interface....", __func__);
			netif_carrier_on(netdev);
		}

		if (retry <= 0) {
			qep_err(link, "%s: Rx Lane is not Aligned\n", __func__);
			netif_carrier_off(netdev);
			/*TODO log error state */
		}

		if (kthread_should_stop())
			break;
		ssleep(QEP_LINK_CHECK_INTERVAL);
	}

	qep_info(link, "%s: Link Status Thread  Exiting\n", __func__);

	/* do_exit(0); */
	return 0;
}

static int qep_thread_start(struct qep_priv *xpriv)
{
	if (xpriv->link_thread) {
		qep_warn(drv, "%s: Link thread is already present.\n",
			 __func__);
		return 0;
	}

	xpriv->link_thread = kthread_create(qep_thread_monitor_link,
					    xpriv->netdev, "qep_link_mon%d",
					    qep_device_count);
	qep_device_count++;

	if (!xpriv->link_thread) {
		qep_err(link, "%s: Link thread creation failed\n", __func__);
		return -EINVAL;
	}
	// Increments usage counter.
	get_task_struct(xpriv->link_thread);
	// safe to start kthread - exiting from it doesn't destroy its struct.
	wake_up_process(xpriv->link_thread);
	qep_info(link, "%s: Link thread created successfully\n", __func__);
	return 0;
}

static void qep_thread_stop(struct qep_priv *xpriv)
{
	if (!xpriv->link_thread) {
		qep_warn(link, "%s: Link thread is not present.\n", __func__);
		return;
	}

	kthread_stop(xpriv->link_thread);
	put_task_struct(xpriv->link_thread);
	xpriv->link_thread = NULL;
	qep_info(link, "%s: Link Thread stopped\n", __func__);
}

static int qep_cmac_ctrl_reset(struct qep_priv *xpriv)
{
	int ret = 0, retry = 20;
	void __iomem *io_addr = xpriv->bar_base + QEP_BASE_OFFSET;

	/* reset release of base */
	writel(1, io_addr + QEP_BASE_CMAC_RESET_CONTROL_OFFSET);
	writel(0, io_addr + QEP_BASE_CMAC_RESET_CONTROL_OFFSET);

	msleep(100);

	io_addr = xpriv->bar_base + QEP_CMAC_CTRL_BASE;
	writel((1 << QEP_CMAC_CTRL_RST_TX_GTWIZ_SHIFT),
	       io_addr + QEP_CMAC_CTRL_RST_OFFSET);

	writel(0, io_addr + QEP_CMAC_CTRL_RST_OFFSET);

	msleep(100);

	writel((1 << QEP_CMAC_CTRL_RST_RX_GTWIZ_SHIFT),
	       io_addr + QEP_CMAC_CTRL_RST_OFFSET);

	writel(0, io_addr + QEP_CMAC_CTRL_RST_OFFSET);

	while (retry--) {
		msleep(500);
		ret = readl(io_addr + QEP_CMAC_CTRL_ERR_OFFSET);
		if (ret == 0) {
			qep_dbg(xpriv->netdev, "%s: CMAC Reset done\n",
				__func__);
			break;
		}
		qep_info(hw, "%s: CMAC Reset is not done. Retrying\n",
			 __func__);
	}
	if (retry == 0) {
		qep_err(hw, "%s: CMAC Reset is not done\n", __func__);
		return -EINVAL;
	}

	return 0;
}
enum clcr_mode { CLCR_FIXED_MODE = 0, CLCR_ROUND_ROBIN };

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
	};
};

static int qep_clcr_setup(struct qep_priv *xpriv, struct clcr_config *config)
{
	u32 reg_val, qbase = 0;
	void __iomem *io_addr;

	io_addr = xpriv->bar_base + QEP_DMA_USER_C2H_BASE;
	reg_val = readl(io_addr + QEP_DMA_USER_C2H_CLCR_OFFSET);

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
		qbase = config->rr.qbase;
	} else
		return -EINVAL;

	writel(reg_val, io_addr + QEP_DMA_USER_C2H_CLCR_OFFSET);

	/* set the qbase */
	writel(qbase, io_addr + QEP_DMA_USER_C2H_CLCR2_OFFSET);

	return 0;
}

static void qep_cmac_loopback_en(struct qep_priv *xpriv, bool en)
{
	void __iomem *io_addr;
	int val = 0x492;


	io_addr = xpriv->bar_base + QEP_CMAC_CTRL_BASE +
			QEP_CMAC_CTRL_LOOPBACK;

	if (en)
		writel(val, io_addr);
	else
		writel(0, io_addr);
}

/* This function initialize and setup CMAC IP */
static int qep_cmac_setup(struct qep_priv *xpriv)
{
	int ret = 0;
	unsigned int cmac_mtu;
	void __iomem *io_addr;
	struct xcmac *inst = &xpriv->cmac_instance;

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
	io_addr = xpriv->bar_base + QEP_CMAC_DRP_OFFSET +
		  QEP_CMAC_CTL_RX_MAX_PACKET_LEN_OFFSET;
	writel(cmac_mtu, io_addr);

	cmac_mtu = readl(io_addr);
	qep_dbg(xpriv->netdev, "%s: Default MTU size	= %d\n", __func__,
		xpriv->netdev->mtu);
	return ret;
}

/* This function enables CMAC IP */
static int qep_cmac_start(struct qep_priv *xpriv)
{
	int ret = 0;
	u32 reg_val = 0;
	void __iomem *io_addr;
	struct xcmac *inst = &xpriv->cmac_instance;

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
	io_addr = xpriv->bar_base +
		  (QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_TX_CTRL_OFFSET);
	reg_val = readl(io_addr);
	reg_val = reg_val & ~(1 << QEP_CMAC_CTRL_TX_CTRL_TX_INHIBIT_SHIFT);
	writel(reg_val, io_addr);

	xpriv->is_cmac_on = 1;
	return ret;
}

/* This function disables CMAC IP */
static int qep_cmac_stop(struct qep_priv *xpriv)
{
	int ret = 0;
	u32 reg_val = 0;
	void __iomem *io_addr;
	struct xcmac *inst = &xpriv->cmac_instance;

	if (xpriv->is_cmac_on == 0)
		return 0;

	xpriv->is_cmac_on = 0;
	io_addr = xpriv->bar_base +
		  (QEP_CMAC_CTRL_BASE + QEP_CMAC_CTRL_TX_CTRL_OFFSET);

	/* Stop TX output for peer to realize link is down */
	reg_val = readl(io_addr);
	reg_val = reg_val | (1 << QEP_CMAC_CTRL_TX_CTRL_TX_INHIBIT_SHIFT);
	writel(reg_val, io_addr);

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
/* This function gets global CSR and sets the default indexes of the
 * xpriv structure. If the default value set in the driver doesn't matches with
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
			ring_size);
	if (index < 0) {
		qep_err(drv, "%s: Expected ring size %d not found", __func__,
				ring_size);
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

static int qep_qdma_rx_queue_add(struct qep_priv *xpriv, u32 q_no, u8 timer_idx,
				 u8 cnt_th_idx, u8 use_adaptive_rx)
{
	int ret = 0;
	char error_str[QEP_ERROR_STR_BUF_LEN] = { '0' };
	unsigned long q_handle = 0;
	struct qdma_queue_conf qconf;

	memset(&qconf, 0, sizeof(struct qdma_queue_conf));
	qconf.st = 1;
	qconf.c2h = 1;
	qconf.irq_en = 0;
#ifdef STMN_ENABLE
	qconf.adaptive_rx = use_adaptive_rx;
	qconf.pfetch_en = 0;
#else
	qconf.adaptive_rx = use_adaptive_rx;
	qconf.pfetch_en = 1;
#endif
	qconf.latency_optimize = 0;

	switch (c2h_byp_mode) {
	case 1:
		qconf.desc_bypass = 1;
		qconf.pfetch_bypass = 0;
		break;
	case 2:
		qconf.desc_bypass = 1;
		qconf.pfetch_bypass = 1;
		break;
	default:
		qconf.desc_bypass = 0;
		qconf.pfetch_bypass = 0;
	}

	qconf.fetch_credit = 1;
	qconf.cmpl_stat_en = 1;
	qconf.cmpl_desc_sz = DESC_SZ_8B;

	qconf.cmpl_rng_sz_idx = xpriv->cmpl_rng_sz_idx;
	qconf.desc_rng_sz_idx = xpriv->rx_desc_rng_sz_idx;
	qconf.c2h_buf_sz_idx = xpriv->rx_buf_sz_idx;
	qconf.cmpl_timer_idx = timer_idx;
	qconf.cmpl_cnt_th_idx = cnt_th_idx;

	qconf.cmpl_trig_mode = TRIG_MODE_TIMER;
	qconf.cmpl_en_intr = (poll_mode == 0);

	qconf.quld = (unsigned long)xpriv;
	qconf.fp_descq_isr_top = qep_isr_rx_tophalf;
	qconf.fp_descq_c2h_packet = qep_rx_pkt_process;
	qconf.fp_proc_ul_cmpt_entry = qdma_stmn_parse_cmpl_entry;

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
			"%s: qdma_queue_add() failed for queue %d with error %d(%s)\n",
			__func__, qconf.qidx, ret, error_str);
		return ret;
	}

	/* Get base q_handle */
	if (q_no == 0)
		xpriv->base_rx_q_handle = q_handle;

	xpriv->rx_q[q_no].parent = xpriv;
	xpriv->rx_q[q_no].q_handle = q_handle;
	xpriv->rx_q[q_no].queue_id = q_no;
	xpriv->rx_q[q_no].rx_timer_idx = timer_idx;
	xpriv->rx_q[q_no].cnt_th_idx = cnt_th_idx;

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
				"%s: qdma_queue_remove() failed for queue %d with error %d(%s)\n",
				__func__, q_no, ret, error_str);
		}

		tasklet_kill(&xpriv->task_tx_done[q_no]);

		xpriv->tx_q[q_no].q_handle = 0;
		xpriv->tx_q[q_no].parent = NULL;
	}

	kfree(xpriv->tx_q);
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
				"%s: qdma_queue_remove() failed for queue %d with error %d(%s)\n",
				__func__, q_no, ret, error_str);
		}
		netif_napi_del(&xpriv->napi[q_no]);

		xpriv->rx_q[q_no].q_handle = 0;
		xpriv->rx_q[q_no].parent = NULL;
	}

	kfree(xpriv->rx_q);
	kfree(xpriv->napi);
}
/* This function setup Rx queues  */
static int qep_qdma_rx_queue_setup(struct qep_priv *xpriv)
{
	int ret = 0, q_no = 0;

	/* Allocate per Rx queue structure */
	xpriv->rx_q = kzalloc(xpriv->num_queues * sizeof(struct qep_dma_q),
			      GFP_KERNEL);
	if (xpriv->rx_q == NULL) {
		qep_err(drv, "%s: rx_q allocation failed. no of queues = %d\n",
			__func__, xpriv->num_queues);
		return -ENOMEM;
	}

	/* Allocate napi array based on number of Rx queue */
	xpriv->napi = kzalloc(xpriv->num_queues * sizeof(struct napi_struct),
			      GFP_KERNEL);
	if (xpriv->napi == NULL) {
		qep_err(drv, "%s: napi allocation failed. no of queues = %d\n",
			__func__, xpriv->num_queues);
		ret = -ENOMEM;
		goto cleanup_rx_q;
	}

	for (q_no = 0; q_no < xpriv->num_queues; q_no++) {
		ret = qep_qdma_rx_queue_add(xpriv, q_no, xpriv->rx_timer_idx,
					    xpriv->rx_cnt_th_idx, 1);
		if (ret != 0) {
			qep_err(drv,
				"%s: qep_qdma_rx_queue_add() failed for queue %d with error %d\n",
				__func__, q_no, ret);
			goto release_rx_q;
		}

		netif_napi_add(xpriv->netdev, &xpriv->napi[q_no], qep_rx_poll,
			       QEP_NAPI_WEIGHT);
	}
	return 0;

release_rx_q:
	qep_qdma_rx_queue_release(xpriv, q_no);
	kfree(xpriv->napi);
cleanup_rx_q:
	kfree(xpriv->rx_q);
	return ret;
}

/* This function setup Tx queues */
static int qep_qdma_tx_queue_setup(struct qep_priv *xpriv)
{
	int ret = 0, q_no = 0;
	char error_str[QEP_ERROR_STR_BUF_LEN] = { '0' };
	unsigned long q_handle = 0;
	struct qdma_queue_conf qconf;

	xpriv->task_tx_done = kzalloc(
		xpriv->num_queues * sizeof(struct tasklet_struct), GFP_KERNEL);
	if (xpriv->task_tx_done == NULL) {
		qep_err(drv,
			"%s: task_tx_done allocation failed. no of queues = %d\n",
			__func__, xpriv->num_queues);
		return -ENOMEM;
	}

	xpriv->tx_q = kzalloc(xpriv->num_queues * sizeof(struct qep_dma_q),
			      GFP_KERNEL);
	if (xpriv->tx_q == NULL) {
		qep_err(drv, "%s: tx_q allocation failed. no of queues = %d\n",
			__func__, xpriv->num_queues);
		return -ENOMEM;
	}

	for (q_no = 0; q_no < xpriv->num_queues; q_no++) {
		memset(&qconf, 0, sizeof(struct qdma_queue_conf));
		qconf.st = 1;
		qconf.c2h = 0;
		qconf.irq_en = 1;
		qconf.wb_status_en = 1;
		qconf.fetch_credit = 1;
		qconf.cmpl_stat_en = 1;
		qconf.cmpl_status_acc_en = 1;
		qconf.cmpl_status_pend_chk = 1;

		switch (h2c_byp_mode) {
		case 1:
			qconf.desc_bypass = 1;
			break;
		default:
			qconf.desc_bypass = 0;
		}

		qconf.desc_rng_sz_idx = xpriv->tx_desc_rng_sz_idx;
		qconf.fp_descq_isr_top = qep_isr_tx_tophalf;
		qconf.fp_bypass_desc_fill = qdma_stmn_bypass_desc_fill;
		qconf.quld = (unsigned long)xpriv;
		qconf.qidx = q_no;

		ret = qdma_queue_add(xpriv->dev_handle, &qconf, &q_handle,
				     error_str, QEP_ERROR_STR_BUF_LEN);
		if (ret != 0) {
			qep_err(drv,
				"%s: qdma_queue_add() failed for queue %d with error %d(%s)\n",
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
static int qep_qdma_setup(struct qep_priv *xpriv)
{
	int ret, status;
	enum qdma_drv_mode qdma_drv_mode;
#if 0
	void __iomem *io_addr = xpriv->bar_base + QEP_BASE_OFFSET;

	writel(1, io_addr + QEP_BASE_QDMA_RESET_CONTROL_OFFSET);
	msleep(100);
#endif

	memset(&xpriv->qdma_dev_conf, 0, sizeof(struct qdma_dev_conf));
	xpriv->qdma_dev_conf.msix_qvec_max = pci_msix_vec_count(xpriv->pcidev);
	if (!xpriv->qdma_dev_conf.msix_qvec_max) {
		qep_dev_err("MSI-X not supported\n");
		return -EINVAL;
	}

	if (xpriv->qdma_dev_conf.msix_qvec_max > QEP_NUM_IRQ_MAX)
		xpriv->qdma_dev_conf.msix_qvec_max = QEP_NUM_IRQ_MAX;

	xpriv->qdma_dev_conf.master_pf = 0;
	xpriv->qdma_dev_conf.intr_moderation = 1;
	xpriv->qdma_dev_conf.qsets_max = QEP_NUM_MAX_QUEUES;
	xpriv->qdma_dev_conf.qsets_base = queue_base;
	xpriv->qdma_dev_conf.pdev = xpriv->pcidev;
	xpriv->qdma_dev_conf.debugfs_dev_root = xpriv->debugfs_dev_root;
	if (poll_mode)
		qdma_drv_mode = POLL_MODE;
	else {
		if (ind_intr_mode == 1)
			qdma_drv_mode = INDIRECT_INTR_MODE;
		else
			qdma_drv_mode = DIRECT_INTR_MODE;
	}
	xpriv->qdma_dev_conf.qdma_drv_mode = qdma_drv_mode;

	ret = qdma_device_open(DRV_NAME, &xpriv->qdma_dev_conf,
			       &xpriv->dev_handle);
	if (ret != 0) {
		qep_dev_err("%s: qdma_device_open failed, Error Code : %d\n",
			    __func__, ret);
		return -EINVAL;
	}

	status = qep_qdma_csr_index_setup(xpriv);
	if (status != 0) {
		qep_dev_err(
			"%s: qep_qdma_csr_index_setup failed with status %d, CSR not programmed,\n",
			__func__, status);
		ret = -EINVAL;
		goto close_qdma;
	}

	qep_dev_info("%s: %02x:%02x.%02x, done", __func__,
		     xpriv->pcidev->bus->number, PCI_SLOT(xpriv->pcidev->devfn),
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
	char error_str[QEP_ERROR_STR_BUF_LEN] = { '0' };

	for (q = 0; q < rxq; q++) {
		ret = qdma_queue_stop(xpriv->dev_handle,
				      (xpriv->base_rx_q_handle + q), error_str,
				      QEP_ERROR_STR_BUF_LEN);
		if (ret < 0) {
			qep_err(drv,
				"%s: RX queue stop failed for queue %d with error: %d msg: %s\n",
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
				"%s: TX queue stop failed for queue %d with error: %d msg: %s\n",
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

	for (q_no = 0; q_no < xpriv->num_queues; q_no++) {
		ret = qdma_queue_start(xpriv->dev_handle,
				       (xpriv->base_rx_q_handle + q_no),
				       error_str, QEP_ERROR_STR_BUF_LEN);
		if (ret != 0) {
			qep_err(drv,
				"%s: qdma_queue_start() failed for queue %d with error %d(%s)\n",
				__func__, q_no, ret, error_str);
			qep_qdma_stop(xpriv, 0, q_no);
			return ret;
		}
	}

	for (q_no = 0; q_no < xpriv->num_queues; q_no++) {
		ret = qdma_queue_start(xpriv->dev_handle,
				       (xpriv->base_tx_q_handle + q_no),
				       error_str, QEP_ERROR_STR_BUF_LEN);
		if (ret != 0) {
			qep_err(drv,
				"%s: qdma_queue_start() failed for queue %d with error %d(%s)\n",
				__func__, q_no, ret, error_str);
			qep_qdma_stop(xpriv, q_no, xpriv->num_queues);
			return ret;
		}
	}
	return 0;
}
static int qep_stop_data_path(struct qep_priv *xpriv)
{
	int ret = 0, q_no = 0;


	/* Stop CMAC */
	netif_carrier_off(xpriv->netdev);

	for (q_no = 0; q_no < xpriv->num_queues; q_no++)
		napi_disable(&xpriv->napi[q_no]);

	ret = qep_cmac_stop(xpriv);
	if (ret != 0)
		qep_err(drv, "%s: qep_cmac_stop failed with status %d\n",
			__func__, ret);

	ret = qep_qdma_stop(xpriv, xpriv->num_queues, xpriv->num_queues);
	if (ret != 0)
		qep_err(drv, "%s: qep_qdma_stop failed with status %d\n",
			__func__, ret);

	return ret;
}
static int qep_start_data_path(struct qep_priv *xpriv)
{
	int ret = 0, q_no = 0;
	struct clcr_config config;

	ret = qep_qdma_start(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_qdma_start failed with status %d\n",
			__func__, ret);
		return ret;
	}

	for (q_no = 0; q_no < xpriv->num_queues; q_no++)
		napi_enable(&xpriv->napi[q_no]);

	config.mode = CLCR_ROUND_ROBIN;
	config.rr.num_queue = xpriv->num_queues;
	config.rr.qbase = xpriv->qdma_dev_conf.qsets_base;
	ret = qep_clcr_setup(xpriv, &config);
	if (ret != 0) {
		qep_err(drv, "%s: CLCR setup failed %d\n", __func__, ret);
		goto disable_napi;
	}

	msleep(100);

	ret = qep_cmac_start(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_cmac_start failed with status %d\n",
			__func__, ret);
		goto disable_napi;
	}

	if (poll_mode)
		for (q_no = 0; q_no < xpriv->num_queues; q_no++)
			napi_schedule(&xpriv->napi[q_no]);

	/* start up the transmission queue */
	netif_tx_start_all_queues(xpriv->netdev);

	return ret;

disable_napi:
	for (q_no = 0; q_no < xpriv->num_queues; q_no++)
		napi_disable(&xpriv->napi[q_no]);
	qep_qdma_stop(xpriv, xpriv->num_queues, xpriv->num_queues);
	return ret;
}
int qep_qdma_reinit_rx_queue(struct qep_priv *xpriv, u32 queue, u8 cnt_th_idx,
			     u8 rx_timer_idx, u32 use_adaptive_rx)
{
	int ret = 0, q_no = 0;
	char error_str[QEP_ERROR_STR_BUF_LEN] = { '0' };

	ret = qep_stop_data_path(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: Error in stopping data path, Error = %d\n",
			__func__, ret);
		return ret;
	}

	ret = qdma_queue_remove(xpriv->dev_handle,
				(xpriv->base_rx_q_handle + queue), error_str,
				QEP_ERROR_STR_BUF_LEN);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_queue_remove() failed for queue %d with error %d(%s)\n",
			__func__, queue, ret, error_str);
	}

	xpriv->rx_q[queue].q_handle = 0;
	xpriv->rx_q[queue].parent = NULL;

	qep_info(drv, "%s: Removed queue : %d\n", __func__, queue);

	ret = qep_qdma_rx_queue_add(xpriv, queue, rx_timer_idx, cnt_th_idx,
				    use_adaptive_rx);
	if (ret != 0) {
		qep_err(drv,
			"%s: qep_qdma_rx_queue_add() failed for queue %d with error %d\n",
			__func__, q_no, ret);
		goto release_queues;
	}

	ret = qep_start_data_path(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_start_data_path failed with status %d\n",
			__func__, ret);
		goto release_queues;
	}

	return ret;

release_queues:
	qep_qdma_tx_queue_release(xpriv, xpriv->num_queues);
	qep_qdma_rx_queue_release(xpriv, xpriv->num_queues);

	return ret;
}

/* This function gets called when interface gets 'UP' request via 'ifconfig up'
 * In this function, Rx and Tx queues are setup send/receive operation started
 */
int qep_open(struct net_device *netdev)
{
	int ret = 0;
	struct qep_priv *xpriv;

	if (!netdev)
		return -EINVAL;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return -EINVAL;

	spin_lock(&xpriv->config_lock);
	xpriv->dev_state = QEP_DEV_STATE_UP;
	wake_up_interruptible(&xpriv->link_mon_needed);

	ret = qep_qdma_rx_queue_setup(xpriv);
	if (ret != 0) {
		qep_err(drv,
			"%s: qep_qdma_rx_queue_setup failed with status %d\n",
			__func__, ret);
		spin_unlock(&xpriv->config_lock);
		return -EINVAL;
	}

	ret = qep_qdma_tx_queue_setup(xpriv);
	if (ret != 0) {
		qep_err(drv,
			"%s: qep_qdma_tx_queue_setup failed with status %d\n",
			__func__, ret);
		goto release_rx_queues;
	}

	ret = qep_start_data_path(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_start_data_path failed with status %d\n",
			__func__, ret);
		goto release_queues;
	}

	qep_info(drv, "%s : interface(%s) is up", __func__, netdev->name);
	spin_unlock(&xpriv->config_lock);
	return 0;

release_queues:
	qep_qdma_tx_queue_release(xpriv, xpriv->num_queues);
release_rx_queues:
	qep_qdma_rx_queue_release(xpriv, xpriv->num_queues);
	spin_unlock(&xpriv->config_lock);
	return ret;
}
/* This function gets called when interface down request comes
 * In this function, Tx/Rx operations on the queues are stopped
 */
int qep_stop(struct net_device *netdev)
{
	int ret = 0;
	struct qep_priv *xpriv;

	if (!netdev)
		return -EINVAL;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return -EINVAL;

	netif_carrier_off(netdev);
	netif_tx_stop_all_queues(netdev);

	spin_lock(&xpriv->config_lock);
	xpriv->dev_state = QEP_DEV_STATE_DOWN;

	ret = qep_stop_data_path(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: Error in stopping data path, Error = %d\n",
			__func__, ret);
	}

	qep_qdma_rx_queue_release(xpriv, xpriv->num_queues);
	qep_qdma_tx_queue_release(xpriv, xpriv->num_queues);

	spin_unlock(&xpriv->config_lock);
	qep_info(drv, "%s : Interface(%s) is down ", __func__, netdev->name);
	return ret;
}

/* This function is used to identify in which queue packet need to transmit */
static u16 qep_select_queue(struct net_device *netdev, struct sk_buff *skb
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
			    ,
			    void *accel_priv
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
			    ,
			    select_queue_fallback_t fallback
#endif
)
{
	static u16 tx_queue;
	u16 queue_id = 0;

	queue_id = (netdev->real_num_tx_queues - tx_queue - 1);
	if (queue_id == 0)
		tx_queue = 0;
	else
		tx_queue++;

	qep_dbg(netdev,
		"real_num_tx_queues = %d, skb->queue_mapping = %d, queue_id (returned) = %d\n",
		netdev->real_num_tx_queues, skb->queue_mapping, queue_id);

	return queue_id;
}

static int qep_unmap_free_pkt_data(struct qdma_request *req)
{
	u16 nb_frags = 0, frag_index = 0;
	struct qdma_sw_sg *qdma_sgl;
	struct qep_tx_cb_arg *qep_tx_cb;
	struct net_device *netdev;
	struct sk_buff *skb;
	skb_frag_t *frag;

	if (req == NULL)
		return -EINVAL;

	qep_tx_cb = (struct qep_tx_cb_arg *)req->uld_data;
	if (qep_tx_cb == NULL)
		return -EINVAL;

	netdev = qep_tx_cb->netdev;
	skb = qep_tx_cb->skb;

	if (skb_shinfo(skb)->nr_frags)
		nb_frags = skb_shinfo(skb)->nr_frags;

	qdma_sgl = req->sgl;
	dma_unmap_single(netdev->dev.parent, qdma_sgl->dma_addr,
			 skb_headlen(skb), DMA_TO_DEVICE);

	qep_dbg(netdev,
		"%s:skb->len = %d skb_headlen(skb) = %d, dma_addr = %llx nb_frags:%d\n",
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
	struct qep_hw_stats *stats;
	struct qep_tx_cb_arg *qep_cb_args;
	struct qdma_request *qdma_req;
	struct qdma_sw_sg *qdma_sgl;
	struct iphdr *ip_header;
	skb_frag_t *frag;

	if (!netdev)
		return -EINVAL;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return -EINVAL;

	stats = &xpriv->stats;

	if (unlikely(skb->len <= ETH_HLEN)) {
		qep_err(tx_err, "skb->len = %d less then eth header len\n",
			skb->len);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	if (unlikely(skb->len == 0)) {
		qep_err(tx_err, "skb->len = %d\n is zero", skb->len);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	cmac_mtu = netdev->mtu + ETH_HLEN + ETH_FCS_LEN;
	if (unlikely(skb->len > cmac_mtu)) {
		qep_err(tx_err, "skb->len = %d greater then mtu\n", skb->len);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	q_id = skb_get_queue_mapping(skb);
	if (q_id >= xpriv->num_queues) {
		qep_err(tx_err, "%s: Invalid queue mapping. q_id = %d\n",
			__func__, q_id);
		dev_kfree_skb_any(skb);
		return -EINVAL;
	}

	if (!netif_carrier_ok(netdev)) {
		qep_err(tx_err, "%s: packet sent when carrier down\n",
					__func__);
		dev_kfree_skb_any(skb);
		return -EINVAL;
	}

	q_handle = xpriv->base_tx_q_handle + q_id;

	if (skb_shinfo(skb)->nr_frags) {
		qep_dbg(netdev, "%s: Detected %d number of skb fragments\n",
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
		GFP_KERNEL);
	if (qep_cb_args == NULL) {
		qep_err(tx_err, "%s: qep_cb_args allocation failed\n",
			__func__);
		return -ENOMEM;
	}

	qep_cb_args->skb = skb;
	qep_cb_args->netdev = netdev;

	qdma_req = (struct qdma_request *)(qep_cb_args + 1);
	qdma_sgl = (struct qdma_sw_sg *)(qdma_req + 1);
	qdma_req->sgl = qdma_sgl;

#ifndef STMN_ENABLE
	qdma_sgl->flags |= QDMA_SG_F_SOP;
	if (nb_frags == 0)
		qdma_sgl->flags |= QDMA_SG_F_EOP;
#endif

	ip_header = (struct iphdr *)skb_network_header(skb);
	qep_dbg(netdev,
		"%s: Received-SKB: len:%d data_len:%d headlen:%d truesize:%d mac_len:%d Frags:%d Prot:%d\n",
		__func__, skb->len, skb->data_len, skb->truesize, skb->mac_len,
		skb_headlen(skb), nb_frags, ip_header->protocol);

#ifdef DEBUG_5
	dump_skb(xpriv, skb);
#endif

	qdma_sgl->len = skb_headlen(skb);
	qdma_req->count = qdma_sgl->len;

	qdma_sgl->dma_addr = dma_map_single(netdev->dev.parent, skb->data,
					    skb_headlen(skb), DMA_TO_DEVICE);
	if (dma_mapping_error(netdev->dev.parent, qdma_sgl->dma_addr)) {
		qep_err(tx_err, "%s:%d dma_map_single failed\n", __func__,
			__LINE__);
		kfree(qep_cb_args);
		ret = -EFAULT;
		goto free_packet_data;
	}

	qdma_sgl->next = NULL;
	qdma_req->sgcnt++;

	/* DMA mapping for fragments data */
	for (frag_index = 0; frag_index < nb_frags; frag_index++) {
		qdma_sgl->next = (struct qdma_sw_sg *)(qdma_sgl + 1);
		qdma_sgl = qdma_sgl->next;
		qdma_sgl->next = NULL;

		frag = &skb_shinfo(skb)->frags[frag_index];
		qdma_sgl->len = skb_frag_size(frag);
		qdma_req->count += qdma_sgl->len;

		qep_dbg(netdev, "%s: frag no = %d, skb_frag_size = %d\n",
			__func__, frag_index, qdma_sgl->len);

		qdma_sgl->dma_addr = (unsigned long)skb_frag_dma_map(
			netdev->dev.parent, frag, 0, skb_frag_size(frag),
			DMA_TO_DEVICE);
		if (dma_mapping_error(netdev->dev.parent, qdma_sgl->dma_addr)) {
			qep_err(tx_err, "%s:%d dma_map_single failed\n",
				__func__, __LINE__);
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
	count = qdma_queue_packet_write(xpriv->dev_handle, q_handle, qdma_req);
	if (count < 0) {
		qep_err(tx_err,
			"%s: qdma_queue_packet_write failed, count = %d\n",
			__func__, count);
		ret = count;
		goto free_packet_data;
	}

	xpriv->tx_q[q_id].stats.tx_packets++;
	xpriv->tx_q[q_id].stats.tx_bytes += skb->len;

	return 0;

free_packet_data:
	if (qep_unmap_free_pkt_data(qdma_req) != 0)
		qep_err(tx_err, "%s: Error in qep_unmap_free_pkt_data\n",
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
		pr_err("%s: Error in qep_unmap_free_pkt_data\n", __func__);

	pr_debug("%s:bytes_done = %d, error = %d\n", __func__, bytes_done, err);

	return ret;
}

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
 * once packet is written in QDMA queue, relevant interrupt will be generated
 * from this handler generated interrupt get served
 */
static void qep_isr_tx_tophalf(unsigned long qhndl, unsigned long uld)
{
	u32 q_no;
	struct qep_priv *xpriv = (struct qep_priv *)(uld);

	/* If ISR if for Tx queue */
	q_no = (qhndl - xpriv->base_tx_q_handle);
	tasklet_schedule(&xpriv->task_tx_done[q_no]);

	qep_dbg(xpriv->netdev, "%s:Tx interrupt called. Mapped queue no = %d\n",
		__func__, q_no);
}

static void qep_isr_rx_tophalf(unsigned long qhndl, unsigned long uld)
{
	u32 q_no;
	struct qep_priv *xpriv = (struct qep_priv *)(uld);

	/* ISR is for Rx queue */
	q_no = (qhndl - xpriv->base_rx_q_handle);
	napi_schedule(&xpriv->napi[q_no]);

	qep_dbg(xpriv->netdev, "%s:Rx interrupt called. Mapped queue no = %d\n",
		__func__, q_no);
}

/* This is differed NAPI task for processing incoming Rx packet from DMA queue
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

	if (!napi)
		return -EINVAL;

	netdev = napi->dev;
	if (!netdev)
		return -EINVAL;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return -EINVAL;

	queue_id = (int)(napi - xpriv->napi);
	q_handle = (xpriv->base_rx_q_handle + queue_id);

	/* Call queue service for QDMA Core to service queue */
	qdma_queue_service(xpriv->dev_handle, q_handle, quota, true);

	if (xpriv->qdma_dev_conf.intr_moderation)
		qdma_queue_c2h_peek(xpriv->dev_handle, q_handle, &udd_cnt,
				    &pkt_cnt, &data_len);
	napi_complete(napi);

	if (poll_mode || (pkt_cnt > quota))
		napi_reschedule(napi);

	return 0;
}

static int qep_rx_deliver(struct qep_priv *xpriv, u32 q_no, unsigned int len,
			  unsigned int sgcnt, struct qdma_sw_sg *sgl, void *udd)
{
	struct net_device *netdev = xpriv->netdev;
	struct sk_buff *skb = NULL;
	struct qdma_sw_sg *l_sgl = sgl;

	BUG_ON(!sgcnt);

#ifdef DEBUG_5
	dump_qdma_sw_sgl(sgcnt, sgl);
#endif
	if (len <= QEP_RX_COPY_THRES) {
		skb = napi_alloc_skb(&xpriv->napi[q_no], len);
		if (unlikely(!skb)) {
			qep_err(rx_err, "%s: napi_alloc_skb() failed\n",
				__func__);
			return -ENOMEM;
		}

		skb_copy_to_linear_data(skb, page_address(l_sgl->pg), len);
		__skb_put(skb, len);
		__free_pages(l_sgl->pg, 0);
	} else {
		unsigned int nr_frags = 0;
		unsigned int frag_len;
		unsigned int frag_offset;

		skb = napi_alloc_skb(&xpriv->napi[q_no], QEP_RX_PULL_LEN);
		if (unlikely(!skb)) {
			qep_err(rx_err, "%s: napi_alloc_skb() failed\n",
				__func__);
			return -ENOMEM;
		}

		skb_copy_to_linear_data(skb,
					page_address(l_sgl->pg + l_sgl->offset),
					QEP_RX_PULL_LEN);
		__skb_put(skb, QEP_RX_PULL_LEN);

		frag_offset = l_sgl->offset + QEP_RX_PULL_LEN;
		frag_len = l_sgl->len - QEP_RX_PULL_LEN;
		do {
			skb_fill_page_desc(skb, nr_frags, l_sgl->pg,
					   frag_offset, frag_len);
			nr_frags++;
			sgcnt--;
			l_sgl = l_sgl->next;
			frag_len = l_sgl->len;
			frag_offset = l_sgl->offset;
		} while (sgcnt && l_sgl);
		skb->len = len;
		skb->data_len = len - QEP_RX_PULL_LEN;
		skb->truesize += skb->data_len;
	}

#ifdef DEBUG_5
	dump_skb(xpriv, skb);
#endif

	/*skb->ip_summed = CHECKSUM_UNNECESSARY;*/
	skb->protocol = eth_type_trans(skb, netdev);
	skb_record_rx_queue(skb, q_no);
	skb_mark_napi_id(skb, &xpriv->napi[q_no]);

	netif_receive_skb(skb);

	return 0;
}

static int qep_rx_gro(struct qep_priv *xpriv, u32 q_no, unsigned int len,
		      unsigned int sgcnt, struct qdma_sw_sg *sgl, void *udd)
{
	struct sk_buff *skb = NULL;
	struct qdma_sw_sg *l_sgl = sgl;
	unsigned int nr_frags = 0;

	skb = napi_get_frags(&xpriv->napi[q_no]);
	if (unlikely(!skb)) {
		qep_err(rx_err, "%s: napi_alloc_skb() failed\n", __func__);
		return -ENOMEM;
	}

	do {
		skb_fill_page_desc(skb, nr_frags, l_sgl->pg, l_sgl->offset,
				   l_sgl->len);
		l_sgl = l_sgl->next;
		nr_frags++;
		sgcnt--;
	} while (sgcnt);

	skb->len = len;
	skb->data_len = len;
	skb->truesize += skb->data_len;

	/*skb->ip_summed = CHECKSUM_UNNECESSARY;*/
	skb_record_rx_queue(skb, q_no);
	skb_mark_napi_id(skb, &xpriv->napi[q_no]);

	napi_gro_frags(&xpriv->napi[q_no]);

	return 0;
}

static int qep_rx_pkt_process(unsigned long qhndl, unsigned long quld,
			      unsigned int len, unsigned int sgcnt,
			      struct qdma_sw_sg *sgl, void *udd)
{
	u32 q_no;
	int ret = 0;
	struct qdma_sw_sg *l_sgl = sgl;
	struct qep_priv *xpriv = (struct qep_priv *)quld;

	BUG_ON(!sgcnt);

	q_no = (qhndl - xpriv->base_rx_q_handle);
	if (false /*netdev->features & NETIF_F_GRO*/)
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

	xpriv->rx_q[q_no].stats.rx_packets++;
	xpriv->rx_q[q_no].stats.rx_bytes += len;

	qep_dbg(xpriv->netdev,
		"%s: q_no = %u, qhndl = %lu, len = %d processed\n", __func__,
		q_no, qhndl, len);

	return ret;
}

/* This function provides statistics to ifconfig command. */
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
static void qep_get_stats64(struct net_device *netdev,
			    struct rtnl_link_stats64 *stats)
#else
static struct rtnl_link_stats64 *
qep_get_stats64(struct net_device *netdev, struct rtnl_link_stats64 *stats)
#endif
{
	u32 i = 0;
	struct qep_priv *xpriv = netdev_priv(netdev);

	/* Added below check to avoid race condition while removing driver.
	 * Where qep_pci_remove gets call which frees cmac and then
	 * qep_get_stats64 is getting called which results in crash in kernel
	 */
	if ((xpriv != NULL) && (xpriv->cmac_instance.base_address) &&
	    (NULL != xpriv->rx_q) && (xpriv->tx_q != NULL)) {
		for (i = 0; i < xpriv->num_queues; i++) {
			stats->rx_bytes += xpriv->rx_q[i].stats.rx_bytes;
			stats->rx_packets += xpriv->rx_q[i].stats.rx_packets;

			stats->tx_bytes += xpriv->tx_q[i].stats.tx_bytes;
			stats->tx_packets += xpriv->tx_q[i].stats.tx_packets;
		}
		xpriv->stats.rx_bytes = stats->rx_bytes;
		xpriv->stats.rx_packets = stats->rx_packets;
		xpriv->stats.tx_bytes = stats->tx_bytes;
		xpriv->stats.tx_packets = stats->tx_packets;
	}

	/* TODO Read CMAC stats and populate other stats */
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
#else
	return stats;
#endif
}

/* This function sets MAC address to the interface */
static int qep_set_mac_address(struct net_device *netdev, void *p)
{
	int ret = 0;
	struct sockaddr *address = p;

	ether_addr_copy(netdev->dev_addr, address->sa_data);
	memcpy(mac_addr, netdev->dev_addr, netdev->addr_len);
	if (!is_valid_ether_addr(netdev->dev_addr))
		eth_random_addr(netdev->dev_addr);

	return ret;
}

/* This function changes MTU size of the interface */
static int qep_change_mtu(struct net_device *netdev, int new_mtu)
{
	int ret = 0;
	struct qep_priv *xpriv;
	unsigned int cmac_mtu;
	void __iomem *io_addr;
	bool is_dev_up = false;

	if (!netdev)
		return -EINVAL;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return -EINVAL;

	if (netif_running(netdev)) {
		is_dev_up = true;
		netif_trans_update(netdev);
		qep_stop(netdev);
	}

	if (new_mtu > QEP_MAX_MTU) {
		qep_err(rx_err,
			"%s: mtu = %d is greater than MAX MTU supported %d\n",
			__func__, new_mtu, QEP_MAX_MTU);
		return -EINVAL;
	}

	if (new_mtu < QEP_MIN_MTU) {
		qep_err(rx_err,
			"%s: mtu = %d is smaller than minimum supported %d\n",
			__func__, new_mtu, QEP_MIN_MTU);
		return -EINVAL;
	}

	qep_dbg(netdev, "%s: mtu: netdev mtu=%d new_mtu=%d", netdev->name,
		netdev->mtu, new_mtu);
	cmac_mtu = new_mtu + ETH_HLEN + ETH_FCS_LEN;

	io_addr = xpriv->bar_base + QEP_CMAC_DRP_OFFSET +
		  QEP_CMAC_CTL_RX_MAX_PACKET_LEN_OFFSET;
	writel(cmac_mtu, io_addr);
	cmac_mtu = readl(io_addr);

	qep_info(drv, "%s: Netdev MTU:%d  CMAC MTU = %d\n", netdev->name,
		 netdev->mtu, cmac_mtu);

	netdev->mtu = new_mtu;

	if (is_dev_up)
		qep_open(netdev);

	return ret;
}

static const struct net_device_ops qep_netdev_ops = {
	.ndo_open = qep_open,
	.ndo_stop = qep_stop,
	.ndo_start_xmit = qep_start_xmit,
	.ndo_select_queue = qep_select_queue,
	.ndo_get_stats64 = qep_get_stats64,
	//	.ndo_set_rx_mode = qep_set_rx_mode,
	//	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = qep_set_mac_address,
	.ndo_change_mtu = qep_change_mtu,
//	.ndo_set_features = qep_set_features,
#ifdef ENABLE_XDP
	.ndo_xdp = qep_xdp,
#endif
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	netdev->max_mtu = QEP_MAX_MTU;
#else
	netdev->mtu = QEP_MAX_MTU;
#endif

	qep_set_ethtool_ops(netdev);
	//netdev->flags |=	(unsigned int)NETIF_F_VLAN_FEATURES;
}

/* Initializes STMN IP */
static void qep_stmn_init(struct qep_priv *xpriv, unsigned short dev_id)
{
	int ret;

	xpriv->tm_dev.xdev = xpriv;
	xpriv->tm_dev.bar_num = QEP_STMN_BAR;
	xpriv->tm_dev.stm_regs = xpriv->bar_base + QEP_STMN_BAR_OFFSET;
	ret = stmn_initialize(xpriv, &(xpriv->tm_dev), dev_id);
	if (ret < 0)
		qep_warn(drv, "%s, Unable to init STMN on BAR %d.\n", __func__,
			 QEP_STMN_BAR);

	ret = stmn_write_c2h_buf_size(xpriv, QEP_DEFAULT_C2H_BUFFER_SIZE);
	if (ret < 0)
		qep_warn(drv, "%s, Unable to Set STMN C2h BUF Size\n",
			 __func__);

}
/* This is probe function which is called when Linux kernel detects PCIe device
 * with PCI ID and device ID (Mentioned in qep_pci_ids)
 * From this function netdevice and device initialization is done
 */
static int qep_pci_probe(struct pci_dev *pdev,
			 const struct pci_device_id *pci_dev_id)
{
	int ret;
	u64 bar_start, bar_len;
	struct net_device *netdev;
	struct qep_priv *xpriv;

	netdev = alloc_netdev_mqs(sizeof(struct qep_priv), "xeth%d",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
				  NET_NAME_ENUM,
#endif
				  qep_setup, nb_queues,
				  nb_queues);

	if (!netdev)
		return -EINVAL;

	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);

	xpriv = netdev_priv(netdev);
	xpriv->netdev = netdev;
	xpriv->pcidev = pdev;
	xpriv->is_cmac_on = 0;
	xpriv->num_queues = nb_queues;
	xpriv->rs_fec_en = rs_fec_en;
	xpriv->msg_enable = netif_msg_init(debug, QEP_DEFAULT_MSG_ENABLE);

	/* Set netdev flags */
	netdev->features |= NETIF_F_HIGHDMA;
	netdev->features |= NETIF_F_SG;

	netdev->gflags |= IFF_PROMISC;

	memcpy(netdev->dev_addr, mac_addr, netdev->addr_len);

	ret = qep_debugfs_dev_init(xpriv);
	if (ret < 0)
		pr_warn("%s, debugfs init for device failed code:%d\n",
			__func__, ret);

	if (nb_queues > QEP_NUM_MAX_QUEUES) {
		pr_err("Num queue %d geater then max supported %d",
				nb_queues, QEP_NUM_MAX_QUEUES);
		ret = -EINVAL;
		goto Err;
	}

	bar_start = pci_resource_start(pdev, QEP_PCI_BAR_NUM);
	bar_len = pci_resource_len(pdev, QEP_PCI_BAR_NUM);
	xpriv->bar_base = ioremap(bar_start, bar_len);
	if (!xpriv->bar_base) {
		dev_err(&pdev->dev, "%s: ioremap failed for bar%d\n", __func__,
			QEP_PCI_BAR_NUM);
		ret = -EIO;
		goto Err;
	}

	ret = qep_qdma_setup(xpriv);
	if (ret < 0) {
		qep_err(probe, "%s : qdma_setup failed Err:%d", __func__, ret);
		goto Err;
	}

	qep_stmn_init(xpriv, pdev->device);

	ret = qep_cmac_setup(xpriv);
	if ((ret != 0) && (ret != -EAGAIN)) {
		qep_err(probe, "%s: qep_cmac_setup() failed with status %d\n",
			__func__, ret);
		goto close_qdma_device;
	}

	ret = register_netdev(netdev);
	if (ret != 0) {
		qep_err(probe, "%s: Failed to register network driver\n",
			__func__);
		goto close_qdma_device;
	}

	xpriv->dev_state = QEP_DEV_STATE_DOWN;
	netif_carrier_off(xpriv->netdev);

	spin_lock_init(&xpriv->config_lock);
	init_waitqueue_head(&xpriv->link_mon_needed);

	/* Start link thread to monitor link status */
	ret = qep_thread_start(xpriv);
	if (ret != 0) {
		qep_err(drv,
			"%s: qep_start_link_thread failed with status %d\n",
			__func__, ret);
		goto close_qdma_device;
	}
	qep_info(probe, "%s : %s  probed successfully", __func__, netdev->name);
	return 0;

close_qdma_device:
	qdma_device_close(pdev, xpriv->dev_handle);
Err:
	qep_debugfs_dev_exit(xpriv);
	kfree(netdev);
	return ret;
}

/* This function gets called when PCIe device has been removed from the bus
 * This function cleans up all initialized components of driver
 */

static void qep_pci_remove(struct pci_dev *pdev)
{
	int ret;
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct qep_priv *xpriv;
	struct clcr_config config;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return;

	if (loopback_en)
		qep_cmac_loopback_en(xpriv, false);

	/* stop CMAC */
	ret = qep_cmac_stop(xpriv);
	if (ret != 0) {
		qep_err(drv, "%s: qep_cmac_stop failed with status %d\n",
			__func__, ret);
	}

	/* Stop link thread if it is running */
	xpriv->dev_state = QEP_DEV_STATE_EXIT;
	wake_up_interruptible(&xpriv->link_mon_needed);
	qep_thread_stop(xpriv);

	config.mode = CLCR_FIXED_MODE;
	config.fixed.qnum = xpriv->num_queues;
	ret = qep_clcr_setup(xpriv, &config);
	if (ret != 0)
		qep_err(drv, "%s: CLCR setup failed %d\n", __func__, ret);

	ret = xcmac_deintialize(&xpriv->cmac_instance);
	if (ret != 0)
		qep_err(drv, "%s: xcmac_deintialize failed\n", __func__);

	unregister_netdev(netdev);

	qdma_device_close(pdev, xpriv->dev_handle);

	if (xpriv->bar_base)
		iounmap(xpriv->bar_base);

	free_netdev(netdev);
}

/* This function handles PCI bus related errors
 * This function gets called if any PCI related errors detected
 */
static pci_ers_result_t qep_pci_error_detected(struct pci_dev *pdev,
					       pci_channel_state_t state)
{
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);

	rtnl_lock();

	switch (state) {
	case pci_channel_io_normal:
		rtnl_unlock();
		return PCI_ERS_RESULT_CAN_RECOVER;
	case pci_channel_io_frozen:
		qep_dev_warn(
			"dev 0x%p,0x%p, frozen state error, reset controller\n",
			pdev, xpriv);
		/* Mark device as removed */
		netif_device_detach(xpriv->netdev);
		if (netif_running(xpriv->netdev))
			qep_stop(xpriv->netdev);
		pci_disable_device(pdev);
		rtnl_unlock();
		return PCI_ERS_RESULT_NEED_RESET;
	case pci_channel_io_perm_failure:
		qep_dev_warn(
			"dev 0x%p,0x%p, failure state error, req. disconnect\n",
			pdev, xpriv);
		if (netif_running(xpriv->netdev))
			qep_stop(xpriv->netdev);
		rtnl_unlock();
		return PCI_ERS_RESULT_DISCONNECT;
	}

	rtnl_unlock();
	return PCI_ERS_RESULT_NEED_RESET;
}

/* This function handled PCI reset request */
static pci_ers_result_t qep_pci_slot_reset(struct pci_dev *pdev)
{
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);

	if (!xpriv)
		return PCI_ERS_RESULT_DISCONNECT;

	qep_dev_info("0x%p restart after slot reset\n", xpriv);
	if (pci_enable_device_mem(pdev)) {
		qep_dev_info("0x%p failed to renable after slot reset\n",
			     xpriv);
		return PCI_ERS_RESULT_DISCONNECT;
	}

	pci_set_master(pdev);
	pci_restore_state(pdev);
	pci_save_state(pdev);

	return PCI_ERS_RESULT_RECOVERED;
}

/* This function handled PCI resume request */
static void qep_pci_error_resume(struct pci_dev *pdev)
{
	int ret = 0;
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);

	if (!xpriv)
		return;

	qep_dev_info("dev 0x%p,0x%p.\n", pdev, xpriv);
	pci_cleanup_aer_uncorrect_error_status(pdev);

	if (netif_running(xpriv->netdev)) {
		ret = qep_open(xpriv->netdev);
		if (ret) {
			qep_dev_err(
				"%s: Device initialization failed after reset.\n",
				__func__);
			return;
		}
	} else {
		qep_dev_err("%s: Device was not running prior to EEH.\n",
			    __func__);
	}
	netif_device_attach(xpriv->netdev);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
static void qep_pci_reset_prepare(struct pci_dev *pdev)
{
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);

	qep_dev_info("%s pdev 0x%p, xdev 0x%p, hndl 0x%lx.\n",
		     dev_name(&pdev->dev), pdev, xpriv, xpriv->dev_handle);

	qdma_device_offline(pdev, xpriv->dev_handle);
	qdma_device_flr_quirk_set(pdev, xpriv->dev_handle);
	if (netif_running(xpriv->netdev))
		qep_stop(xpriv->netdev);
	qdma_device_flr_quirk_check(pdev, xpriv->dev_handle);
}

static void qep_pci_reset_done(struct pci_dev *pdev)
{
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);

	qep_dev_info("%s pdev 0x%p, xdev 0x%p, hndl 0x%lx.\n",
		     dev_name(&pdev->dev), pdev, xpriv, xpriv->dev_handle);
	qdma_device_online(pdev, xpriv->dev_handle);
	if (netif_running(xpriv->netdev))
		qep_open(xpriv->netdev);
}
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
static void qep_pci_reset_notify(struct pci_dev *pdev, bool prepare)
{
	struct qep_priv *xpriv = dev_get_drvdata(&pdev->dev);

	qep_dev_info("pdev = 0x%p, xpriv = 0x%p, prepare = %d.\n", pdev, xpriv,
		     prepare);

	if (prepare) {
		qdma_device_offline(pdev, xpriv->dev_handle);
		qdma_device_flr_quirk_set(pdev, xpriv->dev_handle);
		if (netif_running(xpriv->netdev))
			qep_stop(xpriv->netdev);
		qdma_device_flr_quirk_check(pdev, xpriv->dev_handle);
	} else {
		qdma_device_online(pdev, xpriv->dev_handle);
		if (netif_running(xpriv->netdev))
			qep_open(xpriv->netdev);
	}
}
#endif

static const struct pci_error_handlers qep_pci_err_handler = {
	.error_detected = qep_pci_error_detected,
	.slot_reset = qep_pci_slot_reset,
	.resume = qep_pci_error_resume,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
	.reset_prepare = qep_pci_reset_prepare,
	.reset_done = qep_pci_reset_done,
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
	.reset_notify = qep_pci_reset_notify,
#endif
};

static struct pci_driver qep_pci_driver = {
		.name = DRV_NAME,
		.id_table = qep_pci_ids,
		.probe = qep_pci_probe,
		.remove = qep_pci_remove,
		.err_handler = &qep_pci_err_handler
};

/* This is the entry point of the NIC driver.
 * This function will get called on insert of the driver into the kernel
 */
static int __init qep_module_init(void)
{
	int err = 0;

	pr_info("Initializing %s Version %s\n", DRV_DESC, DRV_VER);

	err = qep_debugfs_init();
	if (err != 0) {
		pr_err("qep debugfs init failed\n");
		return -EINVAL;
	}

	err = libqdma_init(0, qep_debugfs_root);
	if (err != 0) {
		pr_err("libqdma_init failed\n");
		qep_debugfs_exit();
		return -EINVAL;
	}

	err = pci_register_driver(&qep_pci_driver);
	if (err < 0) {
		pr_err("qep: PCI registration failed err=%d\n", err);
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
	pr_info("Exiting %s Version %s\n", DRV_DESC, DRV_VER);
}

module_init(qep_module_init);
module_exit(qep_module_exit);

MODULE_INFO(intree, "Y");
MODULE_AUTHOR("Xilinx");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRV_DESC);
MODULE_VERSION(DRV_VER);
