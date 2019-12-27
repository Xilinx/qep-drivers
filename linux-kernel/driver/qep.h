/*
 * Copyright (c) 2019 Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#ifndef QEP_H_
#define QEP_H_

#include <linux/netdevice.h>
#include "libqdma_export.h"
#include "xcmac.h"
#include "stmn.h"

#define DRV_NAME "qep_drv"
#define DRV_DESC "Xilinx QDMA Ethernet Platform Driver"

#define QEP_VERSION_MAJOR 1
#define QEP_VERSION_MINOR 2
#define QEP_VERSION_PATCH 0

#define QEP_JUMBO_DISABLE (0)
#define QEP_LOOPBACK_EN (0)
#define QEP_RS_FEC_EN (1)
#define QEP_DBG_DUMP_EN (0)

#define QEP_NUM_MAX_QUEUES (256)

#define DRV_VER QEP_VERSION_STR
#define QEP_ERROR_STR_BUF_LEN (512)

#define QEP_PCI_CONFIG_BAR (0)
#if defined(QAP_DESIGN) || defined(QAP_STMN_DESIGN)
#define QEP_PCI_USR_BAR (1)
#define QEP_NUM_IRQ_MAX (8)
#define QEP_QUEUE_BASE (0)
#else
#define QEP_PCI_USR_BAR (2)
#define QEP_NUM_IRQ_MAX (32)
#define QEP_QUEUE_BASE (256)
#endif

#define QEP_MIN_MTU (64)
#if  (QEP_JUMBO_DISABLE)
#define QEP_MAX_MTU (1500)
#else
#define QEP_MAX_MTU (9600)
#endif

#define QEP_MIN_RING_SIZE (512)
#define QEP_DEFAULT_RING_SIZE (512)
#define QEP_DEFAULT_C2H_TIMER_COUNT (5)
#define QEP_DEFAULT_C2H_COUNT_THRESHOLD (64)
#define QEP_DEFAULT_C2H_BUFFER_SIZE (4096)

#define QEP_RX_PKT_SKB_LEN (512)
#define QEP_RX_COPY_THRES (256)
#define QEP_RX_PULL_LEN (128)

#define QEP_NAPI_WEIGHT (66)
#define QEP_LINK_CHECK_INTERVAL (100)

#define QEP_VERSION_STR                                                   \
	__stringify(QEP_VERSION_MAJOR) "." __stringify(                       \
		QEP_VERSION_MINOR) "." __stringify(QEP_VERSION_PATCH)

#define QEP_VERSION                                                       \
	((LIBQDMA_VERSION_MAJOR)*1000 + (LIBQDMA_VERSION_MINOR)*100 +         \
	 LIBQDMA_VERSION_PATCH)

#define QEP_DEFAULT_MSG_ENABLE                                            \
	(NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK | NETIF_MSG_TIMER | \
	 NETIF_MSG_IFDOWN | NETIF_MSG_IFUP | NETIF_MSG_RX_ERR |               \
	 NETIF_MSG_TX_ERR)

#define qep_dbg(netdev, format, arg...) netdev_dbg(netdev, format, ##arg)
#define qep_err(msglvl, format, arg...)                                   \
	netif_err(xpriv, msglvl, xpriv->netdev, format, ##arg)
#define qep_info(msglvl, format, arg...)                                  \
	netif_info(xpriv, msglvl, xpriv->netdev, format, ##arg)
#define qep_warn(msglvl, format, arg...)                                  \
	netif_warn(xpriv, msglvl, xpriv->netdev, format, ##arg)
#define qep_notice(msglvl, format, arg...)                                \
	netif_notice(xpriv, msglvl, xpriv->netdev, format, ##arg)

#define qep_dev_info(format, arg...)                                      \
	dev_info(&xpriv->pcidev->dev, format, ##arg)
#define qep_dev_warn(format, arg...)                                      \
	dev_warn(&xpriv->pcidev->dev, format, ##arg)
#define qep_dev_err(format, arg...) dev_err(&xpriv->pcidev->dev, format, ##arg)


#define QEP_BASE_OFFSET (0x0)
#define QEP_BASE_CMAC_RESET_CONTROL_OFFSET (0x00000020)
#define QEP_BASE_QDMA_RESET_CONTROL_OFFSET (0x00000024)
#define QEP_BASE_DMA_USER_H2C_OFFSET (0x00000028)
#define QEP_BASE_DMA_USER_C2H_OFFSET (0x0000002C)

#define QEP_CMAC_CTRL_BASE (0x00010000)
#define QEP_CMAC_CTRL_LOOPBACK (0x10)
#define QEP_CMAC_CTRL_RST_OFFSET (0x00000014)
#define QEP_CMAC_CTRL_RST_RX_GTWIZ_SHIFT (5)
#define QEP_CMAC_CTRL_RST_TX_GTWIZ_SHIFT (4)
#define QEP_CMAC_CTRL_ERR_OFFSET (0x0000001C)
#define QEP_CMAC_CTRL_TX_CTRL_OFFSET (0x00000308)
#define QEP_CMAC_CTRL_TX_CTRL_TX_INHIBIT_SHIFT (12)

#define QEP_CMAC_100G_IP_BASE (0x00020000)
#define QEP_CMAC_DRP_OFFSET (0xA4000)
/* Actual address is 0xAF, need to multiply by 4 */
#define QEP_CMAC_CTL_RX_MAX_PACKET_LEN_OFFSET (0x2BC)

#define QEP_DMA_USER_C2H_BASE (0x00050000)
#define QEP_DMA_USER_C2H_CLCR_OFFSET (0x00000010)
#define QEP_DMA_USER_C2H_CLCR2_OFFSET (0x00000014)
#define QEP_DMA_USER_C2H_ETH_ADD_SEL_MASK (0x10000)
#define QEP_DMA_USER_C2H_ETH_ADD_SEL_BIT (16)
#define QEP_DMA_USER_C2H_QCTRL_MASK (0x3000)
#define QEP_DMA_USER_C2H_QCTRL_BIT (12)
#define QEP_DMA_USER_C2H_QNUM_MASK (0x7FF)
#define QEP_DMA_USER_C2H_QNUM_BIT (0)
#define QEP_DMA_USER_C2H_QBURST_MASK (0x07)
#define QEP_DMA_USER_C2H_QBURST_BIT (20)

#define QEP_STMN_BAR_OFFSET (0x000D0000)

#define QEP_VERSION_REG (0x110000)
#define QEP_USR_RX_META_LOW_R (0x200000)
#define QEP_USR_RX_META_HIGH_R (0x201000)

/***************** Type Definitions ***********************/
/* Hardware statistics structure */
struct qep_hw_stats {
	u64 rx_bytes;
	u64 rx_mac_bytes;
	u64 rx_good_bytes;
	u64 rx_packets;
	u64 rx_mac_packets;
	u64 rx_good_pkts;
	u64 rx_unicast_pkts;
	u64 rx_multicast_pkts;
	u64 rx_broadcast_pkts;
	u64 rx_pause_pkts;
	u64 rx_vlan_tagged_pkts;
	u64 rx_priority_pause_pkts;

	u64 rx_bad_fcs_pkts;
	u64 rx_bad_code_pkts;
	u64 rx_too_long_pkts;
	u64 rx_oversized_pkts;
	u64 rx_jabber_pkts;
	u64 rx_undersized_pkts;
	u64 rx_frag_pkts;
	u64 rx_stomped_fcs_pkts;

	u64 rx_range_error_pkts;
	u64 rx_truncated_pkts;
	u64 rx_hist_less_64_pkts;
	u64 rx_hist_64_pkts;
	u64 rx_hist_65_127_pkts;
	u64 rx_hist_128_255_pkts;
	u64 rx_hist_256_511_pkts;
	u64 rx_hist_512_1023_pkts;
	u64 rx_hist_1024_1518_pkts;
	u64 rx_hist_1519_1522_pkts;
	u64 rx_hist_1523_1548_pkts;
	u64 rx_hist_1549_2047_pkts;
	u64 rx_hist_2048_4095_pkts;
	u64 rx_hist_4096_8191_pkts;
	u64 rx_hist_8192_9215_pkts;
	u64 rx_hist_greter_eq_9216_pkts;

	u64 tx_bytes;
	u64 tx_mac_bytes;
	u64 tx_good_bytes;
	u64 tx_packets;
	u64 tx_mac_packets;
	u64 tx_good_pkts;
	u64 tx_unicast_pkts;
	u64 tx_multicast_pkts;
	u64 tx_broadcast_pkts;
	u64 tx_pause_pkts;
	u64 tx_vlan_tagged_pkts;
	u64 tx_priority_pause_pkts;

	u64 tx_bad_fcs_pkts;
	u64 tx_bad_code_pkts;
	u64 tx_too_long_pkts;
	u64 tx_oversized_pkts;
	u64 tx_jabber_pkts;
	u64 tx_undersized_pkts;
	u64 tx_frag_pkts;
	u64 tx_stomped_fcs_pkts;

	u64 tx_frame_error_pkts;
	u64 tx_hist_less_64_pkts;
	u64 tx_hist_64_pkts;
	u64 tx_hist_65_127_pkts;
	u64 tx_hist_128_255_pkts;
	u64 tx_hist_256_511_pkts;
	u64 tx_hist_512_1023_pkts;
	u64 tx_hist_1024_1518_pkts;
	u64 tx_hist_1519_1522_pkts;
	u64 tx_hist_1523_1548_pkts;
	u64 tx_hist_1549_2047_pkts;
	u64 tx_hist_2048_4095_pkts;
	u64 tx_hist_4096_8191_pkts;
	u64 tx_hist_8192_9215_pkts;
	u64 tx_hist_greter_eq_9216_pkts;
};

struct qep_tx_cb_arg {
	struct sk_buff *skb;
	struct net_device *netdev;
};

/*Per queue structure*/
struct qep_dma_q {
	u8 cnt_th_idx;
	u8 rx_timer_idx;
	u32 queue_id;
	unsigned long q_handle;
	struct qep_priv *parent;
	struct rtnl_link_stats64 stats;
};

enum cmac_link_state {
	CMAC_LINK_DOWN = 0,
	CMAC_LINK_UP,
	CMAC_LINK_READ_ERROR
};

enum qep_dev_state {
	QEP_DEV_STATE_DOWN = 0,
	QEP_DEV_STATE_UP,
	QEP_DEV_STATE_EXIT,
};

/* QEP Net device private structure */
struct qep_priv {
	u8 rx_desc_rng_sz_idx;
	u8 tx_desc_rng_sz_idx;
	u8 rx_buf_sz_idx;
	u8 rx_timer_idx;
	u8 rx_cnt_th_idx;
	u8 cmpl_rng_sz_idx;

	u8 is_cmac_on;
	u8 rs_fec_en;
	u8 lnk_state;
	u16 msg_enable;
	u16 tx_q_idx;
	u16 num_msix;

	void __iomem *bar_base;
	unsigned long base_rx_q_handle, base_tx_q_handle;
	unsigned long dev_handle;

	struct net_device *netdev;
	struct pci_dev *pcidev;
	struct napi_struct *napi;
	struct tasklet_struct *task_tx_done;
	struct qdma_dev_conf qdma_dev_conf;
	struct qep_dma_q *tx_q, *rx_q;

	struct task_struct *link_thread;
	struct dentry *debugfs_dev_root;

	struct xcmac cmac_instance;
	struct stmn_dev tm_dev;
	struct qep_hw_stats stats;

	spinlock_t stats_lock;
	spinlock_t config_lock;
	spinlock_t lnk_state_lock;
	wait_queue_head_t link_mon_needed;
	enum qep_dev_state dev_state;
};

/* Global Variables  */
extern struct dentry *qep_debugfs_root;

/************* Function Definitions *************************/
int qep_open(struct net_device *netdev);
int qep_stop(struct net_device *netdev);
int qep_arr_find(unsigned int *arr, int n, int element);
void qep_set_ethtool_ops(struct net_device *netdev);
void qep_update_stats(struct net_device *netdev);
int qep_qdma_reinit_rx_queue(struct qep_priv *xpriv, u32 queue, u8 cnt_th_idx,
			     u8 rx_timer_idx, u32 use_adaptive_rx);
int qep_init_reta(struct qep_priv *xpriv, u32 start, u32 q);
void qep_debugfs_exit(void);
int qep_debugfs_init(void);
int qep_debugfs_dev_init(struct qep_priv *xpriv);
void qep_debugfs_dev_exit(struct qep_priv *xpriv);
void qep_get_cmac_stats(struct net_device *netdev, u32 len, void *msg);
#if (QEP_DBG_DUMP_EN)
void dump_qdma_sw_sgl(unsigned int sgcnt, struct qdma_sw_sg *sgl);
void dump_skb(struct qep_priv *xpriv, struct sk_buff *skb);
#endif


#endif /* QEP_H_ */
