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

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/version.h>

#include "qep.h"

#define QEP_REGS_LEN				(80)

#define OBJ_SIZE(m) (sizeof(((struct qep_priv *)0)->m))
#define OBJ_OFFSET(m) (offsetof(struct qep_priv, m))

/* Stats register */
struct qep_stats {
	char stat_string[ETH_GSTRING_LEN];
	u32 stat_size;
	u32 stat_offset;
};

/* CMAC Stats and Statistics registers */
static const struct qep_stats qep_gstrings_stats[] = {
	{ "rx_bytes", OBJ_SIZE(stats.rx_bytes), OBJ_OFFSET(stats.rx_bytes) },
	{ "rx_mac_bytes", OBJ_SIZE(stats.rx_mac_bytes),
			OBJ_OFFSET(stats.rx_mac_bytes) },
	{ "rx_good_bytes", OBJ_SIZE(stats.rx_good_bytes),
			OBJ_OFFSET(stats.rx_good_bytes) },
	{ "rx_packets", OBJ_SIZE(stats.rx_packets),
			OBJ_OFFSET(stats.rx_packets) },
	{ "rx_mac_packets", OBJ_SIZE(stats.rx_mac_packets),
			OBJ_OFFSET(stats.rx_mac_packets) },
	{ "rx_good_pkts", OBJ_SIZE(stats.rx_good_pkts),
			OBJ_OFFSET(stats.rx_good_pkts) },
	{ "rx_unicast_pkts", OBJ_SIZE(stats.rx_unicast_pkts),
			OBJ_OFFSET(stats.rx_unicast_pkts) },
	{ "rx_multicast_pkts", OBJ_SIZE(stats.rx_multicast_pkts),
			OBJ_OFFSET(stats.rx_multicast_pkts) },
	{ "rx_broadcast_pkts", OBJ_SIZE(stats.rx_broadcast_pkts),
			OBJ_OFFSET(stats.rx_broadcast_pkts) },
	{ "rx_pause_pkts", OBJ_SIZE(stats.rx_pause_pkts),
			OBJ_OFFSET(stats.rx_pause_pkts) },
	{ "rx_vlan_tagged_pkts", OBJ_SIZE(stats.rx_vlan_tagged_pkts),
			OBJ_OFFSET(stats.rx_vlan_tagged_pkts) },
	{ "rx_priority_pause_pkts", OBJ_SIZE(stats.rx_priority_pause_pkts),
			OBJ_OFFSET(stats.rx_priority_pause_pkts) },

	{ "rx_bad_fcs_pkts", OBJ_SIZE(stats.rx_bad_fcs_pkts),
			OBJ_OFFSET(stats.rx_bad_fcs_pkts) },
	{ "rx_bad_code_pkts", OBJ_SIZE(stats.rx_bad_code_pkts),
			OBJ_OFFSET(stats.rx_bad_code_pkts) },
	{ "rx_too_long_pkts", OBJ_SIZE(stats.rx_too_long_pkts),
			OBJ_OFFSET(stats.rx_too_long_pkts) },
	{ "rx_oversized_pkts", OBJ_SIZE(stats.rx_oversized_pkts),
			OBJ_OFFSET(stats.rx_oversized_pkts) },
	{ "rx_jabber_pkts", OBJ_SIZE(stats.rx_jabber_pkts),
			OBJ_OFFSET(stats.rx_jabber_pkts) },
	{ "rx_undersized_pkts", OBJ_SIZE(stats.rx_undersized_pkts),
			OBJ_OFFSET(stats.rx_undersized_pkts) },
	{ "rx_frag_pkts", OBJ_SIZE(stats.rx_frag_pkts),
			OBJ_OFFSET(stats.rx_frag_pkts) },
	{ "rx_stomped_fcs_pkts", OBJ_SIZE(stats.rx_stomped_fcs_pkts),
			OBJ_OFFSET(stats.rx_stomped_fcs_pkts) },

	{ "rx_range_error_pkts", OBJ_SIZE(stats.rx_range_error_pkts),
			OBJ_OFFSET(stats.rx_range_error_pkts) },
	{ "rx_truncated_pkts", OBJ_SIZE(stats.rx_truncated_pkts),
			OBJ_OFFSET(stats.rx_truncated_pkts) },
	{ "rx_hist_less_64_pkts", OBJ_SIZE(stats.rx_hist_less_64_pkts),
			OBJ_OFFSET(stats.rx_hist_less_64_pkts) },
	{ "rx_hist_64_pkts", OBJ_SIZE(stats.rx_hist_64_pkts),
			OBJ_OFFSET(stats.rx_hist_64_pkts) },
	{ "rx_hist_65_127_pkts", OBJ_SIZE(stats.rx_hist_65_127_pkts),
			OBJ_OFFSET(stats.rx_hist_65_127_pkts) },
	{ "rx_hist_128_255_pkts", OBJ_SIZE(stats.rx_hist_128_255_pkts),
			OBJ_OFFSET(stats.rx_hist_128_255_pkts) },
	{ "rx_hist_256_511_pkts", OBJ_SIZE(stats.rx_hist_256_511_pkts),
			OBJ_OFFSET(stats.rx_hist_256_511_pkts) },
	{ "rx_hist_512_1023_pkts", OBJ_SIZE(stats.rx_hist_512_1023_pkts),
			OBJ_OFFSET(stats.rx_hist_512_1023_pkts) },
	{ "rx_hist_1024_1518_pkts", OBJ_SIZE(stats.rx_hist_1024_1518_pkts),
			OBJ_OFFSET(stats.rx_hist_1024_1518_pkts) },
	{ "rx_hist_1519_1522_pkts", OBJ_SIZE(stats.rx_hist_1519_1522_pkts),
			OBJ_OFFSET(stats.rx_hist_1519_1522_pkts) },
	{ "rx_hist_1523_1548_pkts", OBJ_SIZE(stats.rx_hist_1523_1548_pkts),
			OBJ_OFFSET(stats.rx_hist_1523_1548_pkts) },
	{ "rx_hist_1549_2047_pkts", OBJ_SIZE(stats.rx_hist_1549_2047_pkts),
			OBJ_OFFSET(stats.rx_hist_1549_2047_pkts) },
	{ "rx_hist_2048_4095_pkts", OBJ_SIZE(stats.rx_hist_2048_4095_pkts),
			OBJ_OFFSET(stats.rx_hist_2048_4095_pkts) },
	{ "rx_hist_4096_8191_pkts", OBJ_SIZE(stats.rx_hist_4096_8191_pkts),
			OBJ_OFFSET(stats.rx_hist_4096_8191_pkts) },
	{ "rx_hist_8192_9215_pkts", OBJ_SIZE(stats.rx_hist_8192_9215_pkts),
			OBJ_OFFSET(stats.rx_hist_8192_9215_pkts) },
	{ "rx_hist_greter_eq_9216_pkts",
	  OBJ_SIZE(stats.rx_hist_greter_eq_9216_pkts),
	  OBJ_OFFSET(stats.rx_hist_greter_eq_9216_pkts) },

	{ "tx_bytes", OBJ_SIZE(stats.tx_bytes),
			OBJ_OFFSET(stats.tx_bytes) },
	{ "tx_mac_bytes", OBJ_SIZE(stats.tx_mac_bytes),
			OBJ_OFFSET(stats.tx_mac_bytes) },
	{ "tx_good_bytes", OBJ_SIZE(stats.tx_good_bytes),
			OBJ_OFFSET(stats.tx_good_bytes) },
	{ "tx_packets", OBJ_SIZE(stats.tx_packets),
			OBJ_OFFSET(stats.tx_packets) },
	{ "tx_mac_packets", OBJ_SIZE(stats.tx_mac_packets),
			OBJ_OFFSET(stats.tx_mac_packets) },
	{ "tx_good_pkts", OBJ_SIZE(stats.tx_good_pkts),
			OBJ_OFFSET(stats.tx_good_pkts) },
	{ "tx_unicast_pkts", OBJ_SIZE(stats.tx_unicast_pkts),
			OBJ_OFFSET(stats.tx_unicast_pkts) },
	{ "tx_multicast_pkts", OBJ_SIZE(stats.tx_multicast_pkts),
			OBJ_OFFSET(stats.tx_multicast_pkts) },
	{ "tx_broadcast_pkts", OBJ_SIZE(stats.tx_broadcast_pkts),
			OBJ_OFFSET(stats.tx_broadcast_pkts) },
	{ "tx_pause_pkts", OBJ_SIZE(stats.tx_pause_pkts),
			OBJ_OFFSET(stats.tx_pause_pkts) },
	{ "tx_vlan_tagged_pkts", OBJ_SIZE(stats.tx_vlan_tagged_pkts),
			OBJ_OFFSET(stats.tx_vlan_tagged_pkts) },
	{ "tx_priority_pause_pkts", OBJ_SIZE(stats.tx_priority_pause_pkts),
			OBJ_OFFSET(stats.tx_priority_pause_pkts) },

	{ "tx_bad_fcs_pkts", OBJ_SIZE(stats.tx_bad_fcs_pkts),
			OBJ_OFFSET(stats.tx_bad_fcs_pkts) },
	{ "tx_frame_error_pkts", OBJ_SIZE(stats.tx_frame_error_pkts),
			OBJ_OFFSET(stats.tx_frame_error_pkts) },
	{ "tx_hist_less_64_pkts", OBJ_SIZE(stats.tx_hist_less_64_pkts),
			OBJ_OFFSET(stats.tx_hist_less_64_pkts) },
	{ "tx_hist_64_pkts", OBJ_SIZE(stats.tx_hist_64_pkts),
			OBJ_OFFSET(stats.tx_hist_64_pkts) },
	{ "tx_hist_65_127_pkts", OBJ_SIZE(stats.tx_hist_65_127_pkts),
			OBJ_OFFSET(stats.tx_hist_65_127_pkts) },
	{ "tx_hist_128_255_pkts", OBJ_SIZE(stats.tx_hist_128_255_pkts),
			OBJ_OFFSET(stats.tx_hist_128_255_pkts) },
	{ "tx_hist_256_511_pkts", OBJ_SIZE(stats.tx_hist_256_511_pkts),
			OBJ_OFFSET(stats.tx_hist_256_511_pkts) },
	{ "tx_hist_512_1023_pkts", OBJ_SIZE(stats.tx_hist_512_1023_pkts),
			OBJ_OFFSET(stats.tx_hist_512_1023_pkts) },
	{ "tx_hist_1024_1518_pkts", OBJ_SIZE(stats.tx_hist_1024_1518_pkts),
			OBJ_OFFSET(stats.tx_hist_1024_1518_pkts) },
	{ "tx_hist_1519_1522_pkts", OBJ_SIZE(stats.tx_hist_1519_1522_pkts),
			OBJ_OFFSET(stats.tx_hist_1519_1522_pkts) },
	{ "tx_hist_1523_1548_pkts", OBJ_SIZE(stats.tx_hist_1523_1548_pkts),
			OBJ_OFFSET(stats.tx_hist_1523_1548_pkts) },
	{ "tx_hist_1549_2047_pkts", OBJ_SIZE(stats.tx_hist_1549_2047_pkts),
			OBJ_OFFSET(stats.tx_hist_1549_2047_pkts) },
	{ "tx_hist_2048_4095_pkts", OBJ_SIZE(stats.tx_hist_2048_4095_pkts),
			OBJ_OFFSET(stats.tx_hist_2048_4095_pkts) },
	{ "tx_hist_4096_8191_pkts", OBJ_SIZE(stats.tx_hist_4096_8191_pkts),
			OBJ_OFFSET(stats.tx_hist_4096_8191_pkts) },
	{ "tx_hist_8192_9215_pkts", OBJ_SIZE(stats.tx_hist_8192_9215_pkts),
			OBJ_OFFSET(stats.tx_hist_8192_9215_pkts) },
	{ "tx_hist_greter_eq_9216_pkts",
			OBJ_SIZE(stats.tx_hist_greter_eq_9216_pkts),
			OBJ_OFFSET(stats.tx_hist_greter_eq_9216_pkts) },
};
#define QEP_GLOBAL_STATS_LEN ARRAY_SIZE(qep_gstrings_stats)

/* This is a ethtool callback and prints driver info */
static void qep_get_drvinfo(struct net_device *netdev,
			    struct ethtool_drvinfo *drvinfo)
{
	struct qep_priv *xpriv = netdev_priv(netdev);

	strlcpy(drvinfo->driver, DRV_NAME, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, DRV_VER, sizeof(drvinfo->version));
	strlcpy(drvinfo->bus_info, pci_name(xpriv->pcidev),
		sizeof(drvinfo->bus_info));
}

/* This is a ethtool callback and register dump length/size */
static int qep_get_regs_len(struct net_device *netdev)
{
	return QEP_REGS_LEN * sizeof(u32);
}

/* This is a ethtool callback and prints register dump */
static void qep_get_regs(struct net_device *netdev, struct ethtool_regs *regs,
			 void *p)
{
	u32 *regs_buff = p;
	unsigned int i = 0, ret = 0, reg_idx = 0;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);

	memset(p, 0, QEP_REGS_LEN * sizeof(u32));

	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CMAC_VERSION_REG_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIGURATION_TX_REG1_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIGURATION_RX_REG1_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_REG1_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG1_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG2_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG3_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG4_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG5_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG1_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG2_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG3_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG4_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG5_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET);
	regs_buff[reg_idx++] =
		xcmac_in32((u64)(xpriv->bar_base) + QEP_CMAC_100G_IP_BASE +
			   CONFIGURATION_RX_FLOW_CONTROL_REG2_OFFSET);

	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
			QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return;
	}

	for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
		regs_buff[reg_idx + i + 0] = csr_conf.ring_sz[i];
		regs_buff[reg_idx + i + 16] = csr_conf.c2h_buf_sz[i];
		regs_buff[reg_idx + i + 32] = csr_conf.c2h_timer_cnt[i];
		regs_buff[reg_idx + i + 48] = csr_conf.c2h_cnt_th[i];
	}

	/*Update the QEP_REGS_LEN macro if adding more registers*/
}

static unsigned int qep_arr_max(unsigned int *arr, int n)
{
	unsigned int temp;
	int i;

	temp = *arr;
	for (i = 0; i < n; i++) {
		if (*(arr + i) > temp)
			temp = *(arr + i);
	}

	return temp;
}

int qep_arr_find(unsigned int *arr, int n, int element)
{
	int i = 0;

	for (i = 0; i < n; i++) {
		if (*(arr + i) == element)
			return i;
	}
	return -1;
}

/* This is a ethtool callback and gets msg level */
static u32 qep_get_msglevel(struct net_device *netdev)
{
	struct qep_priv *xpriv = netdev_priv(netdev);

	return xpriv->msg_enable;
}

/* This is a ethtool callback and sets msg level */
static void qep_set_msglevel(struct net_device *netdev, u32 data)
{
	struct qep_priv *xpriv = netdev_priv(netdev);

	xpriv->msg_enable = data;
}

/* This is a ethtool callback and gets ring size of queues */
static void qep_get_ringparam(struct net_device *netdev,
			      struct ethtool_ringparam *ring)
{
	int ret = 0;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (!xpriv)
		return;

	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return;
	}

	/* Setting unused parameters to 0 */
	ring->rx_mini_max_pending = 0;
	ring->rx_jumbo_max_pending = 0;
	ring->rx_mini_pending = 0;
	ring->rx_jumbo_pending = 0;

	ring->rx_max_pending = qep_arr_max(csr_conf.ring_sz,
					   QDMA_GLOBAL_CSR_ARRAY_SZ);
	ring->rx_pending = csr_conf.ring_sz[xpriv->rx_desc_rng_sz_idx];

	ring->tx_max_pending = qep_arr_max(csr_conf.ring_sz,
					   QDMA_GLOBAL_CSR_ARRAY_SZ);
	ring->tx_pending = csr_conf.ring_sz[xpriv->tx_desc_rng_sz_idx];
}

/* This is a ethtool callback and sets ring size of queues */
static int qep_set_ringparam(struct net_device *netdev,
			     struct ethtool_ringparam *ring)
{
	int ret = 0, tx_idx, rx_idx = 0;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (!xpriv)
		return -EINVAL;

	if (ring->rx_jumbo_pending) {
		netdev_info(xpriv->netdev, "%s: jumbo_pending not supported\n",
			    __func__);
		return -EINVAL;
	}
	if (ring->rx_mini_pending) {
		netdev_info(xpriv->netdev, "%s: mini_pending not supported\n",
			    __func__);
		return -EINVAL;
	}

	if (ring->rx_pending < QEP_MIN_RING_SIZE
			|| ring->tx_pending < QEP_MIN_RING_SIZE) {
		netdev_info(xpriv->netdev, "%s: ring size more then min (%d)\n",
			    __func__, QEP_MIN_RING_SIZE);
		return -EINVAL;
	}


	/* Get the global CSR , get the index of the ring size array and then
	 * modify the ring size
	 */
	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	tx_idx = qep_arr_find(csr_conf.ring_sz,
			     QDMA_GLOBAL_CSR_ARRAY_SZ, ring->tx_pending);
	if (tx_idx < 0) {
		qep_err(drv, "%s: Invalid tx_pending = %d\n", __func__,
			ring->tx_pending);
		return -EINVAL;
	}


	rx_idx = qep_arr_find(csr_conf.ring_sz,
			     QDMA_GLOBAL_CSR_ARRAY_SZ, ring->rx_pending);
	if (rx_idx < 0) {
		qep_err(drv, "%s: Invalid rx_pending = %d\n", __func__,
			ring->rx_pending);
		return -EINVAL;
	}

	xpriv->tx_desc_rng_sz_idx = tx_idx;
	xpriv->rx_desc_rng_sz_idx = rx_idx;
	xpriv->cmpl_rng_sz_idx = rx_idx;

	if (!netif_running(netdev))
		return 0;

	ret = qep_stop(netdev); /* Stop the interface */
	if (ret != 0) {
		qep_err(drv, "%s qep_stop  failed err=%d",
				__func__, ret);
		return ret;
	}

	usleep_range(1000, 2000);
	ret = qep_open(netdev);
	if (ret != 0) {
		qep_err(drv, "%s qep_open failed err=%d",
			__func__, ret);
		return ret;
	}

	return 0;
}

/* This is a ethtool callback and gets number of queues */
static void qep_get_channels(struct net_device *netdev,
			     struct ethtool_channels *ch)
{
	struct qep_priv *xpriv = netdev_priv(netdev);

	ch->max_rx = QEP_NUM_MAX_QUEUES;
	ch->max_tx = QEP_NUM_MAX_QUEUES;
	ch->max_other = 0;
	ch->max_combined = 0;
	ch->rx_count = xpriv->netdev->real_num_rx_queues;
	ch->tx_count = xpriv->netdev->real_num_tx_queues;
	ch->other_count = 0;
	ch->combined_count = 0;
}

/* This is a ethtool callback and sets number of queues */
static int qep_set_channels(struct net_device *netdev,
			    struct ethtool_channels *ch)
{
	int ret = 0;
	struct qep_priv *xpriv = netdev_priv(netdev);
	u8 iface_up = 0;

	if ((ch->rx_count == 0) || (ch->tx_count == 0)) {
		qep_err(drv,
			"%s: invalid channel count passed, Rx = %d, Tx = %d\n",
			__func__, ch->rx_count, ch->tx_count);
		return -EINVAL;
	}

	if ((ch->rx_count > QEP_NUM_MAX_QUEUES) ||
	    (ch->tx_count > QEP_NUM_MAX_QUEUES)) {
		qep_err(drv, "%s: channel count passed %d %d more then max supported %d\n",
				__func__, ch->tx_count,
				ch->rx_count, QEP_NUM_MAX_QUEUES);
		return -EINVAL;
	}

	qep_dbg(netdev, "%s: rx_count = %d, tx_count = %d, combined = %d\n",
		__func__, ch->rx_count, ch->tx_count, ch->combined_count);

	if (netif_running(netdev)) {
		iface_up = 1;
		ret = qep_stop(netdev); /* Stop the interface */
		if (ret != 0) {
			qep_err(drv, "%s qep_stop failed err=%d",
					__func__, ret);
			return ret;
		}
	}

	netif_set_real_num_tx_queues(netdev, ch->tx_count);
	netif_set_real_num_rx_queues(netdev, ch->rx_count);

	if (xpriv->netdev->features & NETIF_F_RXHASH) {
		ret = qep_init_reta(xpriv, 0, ch->rx_count);
		if (ret != 0) {
			qep_err(drv, "%s qep_init_reta failed err=%d",
					__func__, ret);
			return ret;
		}
	}

	if (!iface_up)
		return 0;

	usleep_range(1000, 2000);
	ret = qep_open(netdev); /* Open the interface */
	if (ret != 0) {
		qep_err(drv, "%s qep_open failed err=%d",
				__func__, ret);
		return ret;
	}

	return 0;
}

/* This is a ethtool callback and populates  stats as strings*/
static void qep_get_strings(struct net_device *netdev, u32 stringset, u8 *data)
{
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_STATS: {
		for (i = 0; i < QEP_GLOBAL_STATS_LEN; i++) {
			memcpy(p, qep_gstrings_stats[i].stat_string,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	}
	default:
		break;
	}
}

/* This is a ethtool callback and populates  stats */
static void qep_get_ethtool_stats(struct net_device *netdev,
				  struct ethtool_stats *stats, u64 *data)
{
	u32 count = 0;
	struct qep_priv *xpriv = netdev_priv(netdev);
	const struct qep_stats *stat = qep_gstrings_stats;
	char *p = NULL;

	qep_update_stats(netdev);

	for (count = 0; count < QEP_GLOBAL_STATS_LEN; count++) {
		p = (char *)xpriv + stat->stat_offset;

		if (stat->stat_size == sizeof(u64))
			data[count] = *(u64 *)p;
		else
			data[count] = *(u32 *)p;

		stat++;
	}
}

/* This is a ethtool callback and gets stats length */
static int qep_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return QEP_GLOBAL_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

/* This function return CMAC stats as string */
void qep_get_cmac_stats(struct net_device *netdev, u32 len, void *msg)
{
	u8 *stat_string;
	u8 *p;
	u64 val[QEP_GLOBAL_STATS_LEN] = {0};
	int i;

	stat_string = kcalloc(QEP_GLOBAL_STATS_LEN, ETH_GSTRING_LEN,
				GFP_KERNEL);
	if (!stat_string)
		return;

	qep_get_strings(netdev, ETH_SS_STATS, stat_string);
	qep_get_ethtool_stats(netdev, NULL, val);
	p = stat_string;

	for (i = 0; i < QEP_GLOBAL_STATS_LEN; i++) {
		snprintf((u8 *)msg + strlen(msg), len - strlen(msg),
			 "%-32s = %llu\n", p, val[i]);
		p += ETH_GSTRING_LEN;
	}

	kfree(stat_string);
}

/* This is a ethtool callback and gets global coalesce options */
static int qep_get_coalesce(struct net_device *netdev,
			    struct ethtool_coalesce *ec)
{
	int ret = 0;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (!xpriv)
		return -EINVAL;

	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	ec->rx_max_coalesced_frames =
		csr_conf.c2h_cnt_th[xpriv->rx_cnt_th_idx];
	ec->rx_coalesce_usecs =
		csr_conf.c2h_timer_cnt[xpriv->rx_timer_idx];

	return 0;
}

/* This is a ethtool callback and sets global coalesce options */
static int qep_set_coalesce(struct net_device *netdev,
			    struct ethtool_coalesce *ec)
{
	u8 rx_count_threshold_index = 0;
	int ret = 0, index = 0;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (!xpriv)
		return -EINVAL;

	/* Get the global CSR , get the index of the ring size array and
	 * then modify the ring size
	 */
	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	index = qep_arr_find(csr_conf.c2h_cnt_th,
			     QDMA_GLOBAL_CSR_ARRAY_SZ,
			     ec->rx_max_coalesced_frames);
	if (index < 0) {
		qep_err(drv, "%s: Invalid rx_max_coalesced_frames = %d\n",
			__func__, ec->rx_max_coalesced_frames);
		return -EINVAL;
	}
	rx_count_threshold_index = xpriv->rx_cnt_th_idx;
	xpriv->rx_cnt_th_idx = index;

	index = qep_arr_find(csr_conf.c2h_timer_cnt,
			     QDMA_GLOBAL_CSR_ARRAY_SZ, ec->rx_coalesce_usecs);
	if (index < 0) {
		qep_err(drv, "%s: Invalid rx_coalesce_usecs = %d\n", __func__,
			ec->rx_coalesce_usecs);
		xpriv->rx_cnt_th_idx = rx_count_threshold_index;
		return -EINVAL;
	}
	xpriv->rx_timer_idx = index;

	/* Stop the interface */
	ret = qep_stop(netdev);
	if (ret != 0) {
		qep_err(drv, "%s qep_stop failed err=%d",
				__func__, ret);
		return ret;
	}

	/* Open the interface */
	ret = qep_open(netdev);
	if (ret != 0) {
		qep_err(drv, "%s qep_open failed err=%d",
				__func__, ret);
		return ret;
	}

	return 0;
}

#if KERNEL_VERSION(4, 5, 0) < LINUX_VERSION_CODE
/* This is a ethtool callback and gets per queue coalesce options */
static int qep_get_per_queue_coalesce(struct net_device *netdev, u32 queue,
				      struct ethtool_coalesce *ec)
{
	int ret = 0;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);

	qep_info(drv, "%s: queue = %d\n", __func__, queue);

	if (xpriv == NULL)
		return -EINVAL;

	if (queue >= xpriv->netdev->real_num_rx_queues) {
		qep_err(drv, "%s: Invalid queue id entered = %d\n", __func__,
			queue);
		return -EINVAL;
	}

	if (xpriv->is_cmac_on == 0) {
		qep_err(drv,
			"%s: Interface is down. First bring interface up\n",
			__func__);
		return -EINVAL;
	}

	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				 QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	ec->rx_max_coalesced_frames =
		csr_conf.c2h_cnt_th[xpriv->rx_q[queue].cnt_th_idx];
	ec->rx_coalesce_usecs =
		csr_conf.c2h_timer_cnt[xpriv->rx_q[queue].rx_timer_idx];
	return ret;
}

/* This is a ethtool callback and sets per queue coalesce options */
static int qep_set_per_queue_coalesce(struct net_device *netdev, u32 queue,
				      struct ethtool_coalesce *ec)
{
	int ret = 0, index = 0;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (xpriv == NULL)
		return -EINVAL;

	qep_info(drv, "%s: queue = %d, adaptive_rx = %d\n", __func__, queue,
		 ec->use_adaptive_rx_coalesce);

	if (queue >= xpriv->netdev->real_num_rx_queues) {
		qep_err(drv, "%s: Invalid queue id entered = %d\n", __func__,
			queue);
		return -EINVAL;
	}

	if (ec->rx_max_coalesced_frames == 0 && ec->rx_coalesce_usecs == 0) {
		qep_err(drv,
			"%s: Invalid parameters entered supported only rx_max_coalesced_frames and/or rx_coalesce_usecs\n",
			__func__);
		return -EINVAL;
	}

	/* Get the global CSR , get the index of the ring size array and
	 * then modify the ring size
	 */
	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	/* Check if user wants to set coalesced frames */
	if (ec->rx_max_coalesced_frames) {
		if (ec->rx_max_coalesced_frames !=
		    xpriv->rx_q[queue].cnt_th_idx) {
			index = qep_arr_find(csr_conf.c2h_cnt_th,
					     QDMA_GLOBAL_CSR_ARRAY_SZ,
					     ec->rx_max_coalesced_frames);
			if (index < 0) {
				qep_err(drv,
					"%s: Invalid rx_max_coalesced_frames = %d\n",
					__func__, ec->rx_max_coalesced_frames);
				return -EINVAL;
			}
			xpriv->rx_q[queue].cnt_th_idx = index;
		} else
			qep_info(
				drv,
				"%s: Trying to set same rx_max_coalesced_frames = %d, Ignored!\n",
				__func__, ec->rx_max_coalesced_frames);
	}

	/* Check if user wants to set coalesced usecs */
	if (ec->rx_coalesce_usecs) {
		if (ec->rx_coalesce_usecs != xpriv->rx_q[queue].rx_timer_idx) {
			index = qep_arr_find(csr_conf.c2h_timer_cnt,
					     QDMA_GLOBAL_CSR_ARRAY_SZ,
					     ec->rx_coalesce_usecs);
			if (index < 0) {
				qep_err(drv,
					"%s: Invalid rx_coalesce_usecs = %d\n",
					__func__, ec->rx_coalesce_usecs);
				return -EINVAL;
			}
			xpriv->rx_q[queue].rx_timer_idx = index;
		} else
			qep_info(
				drv,
				"%s: Trying to set same rx_coalesce_usecs = %d, Ignored!\n",
				__func__, ec->rx_coalesce_usecs);
	}

	ret = qep_qdma_reinit_rx_queue(xpriv, queue,
				       xpriv->rx_q[queue].cnt_th_idx,
				       xpriv->rx_q[queue].rx_timer_idx,
				       ec->use_adaptive_rx_coalesce);
	if (ret != 0) {
		qep_err(drv,
			"%s: qep_qdma_reinit_rx_queue() failed with error %d\n",
			__func__, ret);
	}

	return ret;
}
#endif

#if KERNEL_VERSION(4, 13, 0) < LINUX_VERSION_CODE
/* This is a ethtool callback and fet FEC params */
static int qep_get_fecparam(struct net_device *netdev,
			    struct ethtool_fecparam *fec)
{
	struct qep_priv *xpriv;

	xpriv = netdev_priv(netdev);
	if (xpriv == NULL)
		return -EINVAL;

	fec->fec |= ETHTOOL_FEC_OFF;
	fec->fec |= ETHTOOL_FEC_RS;
	fec->fec |= ETHTOOL_FEC_AUTO;

	if (xpriv->rs_fec_en == 1)
		fec->active_fec = ETHTOOL_FEC_RS;
	else
		fec->active_fec = ETHTOOL_FEC_OFF;

	return 0;
}

/* This is a ethtool callback and sets FEC params as AUTO, RS or OFF */
static int qep_set_fecparam(struct net_device *netdev,
			    struct ethtool_fecparam *fec)
{
	int ret = 0;
	struct qep_priv *xpriv;
	unsigned short int rs_fec_new = 0;

	xpriv = netdev_priv(netdev);
	if (xpriv == NULL)
		return -EINVAL;

	if (fec->fec & ETHTOOL_FEC_OFF) {
		rs_fec_new = 0;
	} else if ((fec->fec & ETHTOOL_FEC_AUTO) ||
		   (fec->fec & ETHTOOL_FEC_RS)) {
		rs_fec_new = 1;
	} else {
		qep_err(drv, "%s: Unsupported FEC option\n", __func__);
		return -EINVAL;
	}

	if (rs_fec_new == xpriv->rs_fec_en) {
		qep_info(drv, "%s: Existing FEC mode is same as requested\n",
			 __func__);
	} else {
		xpriv->rs_fec_en = rs_fec_new;
		/* Configure RS-FEC */
		ret = xcmac_config_rsfec(&xpriv->cmac_instance,
					 xpriv->rs_fec_en);
		if (ret != 0) {
			qep_err(hw,
				"%s: Error in configuring RS-FEC in CMAC Core\n",
				__func__);
			return ret;
		}
		qep_info(drv, "%s: FEC mode is set to %d\n", __func__,
			 xpriv->rs_fec_en);
	}

	return ret;
}
#endif

/* This function reads CMAC stats */
void qep_update_stats(struct net_device *netdev)
{
	int ret = 0;
	unsigned long flags;
	struct qep_priv *xpriv;
	struct qep_hw_stats *stats;
	struct xcmac_packet_count_statistics tx_hist, rx_hist;
	struct xcmac_malformed_statistics rx_malformed, tx_malformed;
	struct xcmac_packet_type_statistics rx_pkt_types, tx_pkt_types;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return;

	stats = &xpriv->stats;

	memset(&tx_hist, 0, sizeof(tx_hist));
	memset(&rx_hist, 0, sizeof(rx_hist));
	memset(&tx_malformed, 0, sizeof(tx_malformed));
	memset(&rx_malformed, 0, sizeof(rx_malformed));
	memset(&tx_pkt_types, 0, sizeof(tx_pkt_types));
	memset(&rx_pkt_types, 0, sizeof(rx_pkt_types));

	spin_lock_irqsave(&xpriv->stats_lock, flags);

	/* Set the Tick register so that statistics are
	 * captured in readable registers
	 */
	ret = xcmac_latch_statistics(&xpriv->cmac_instance);
	if (ret != 0)
		qep_err(hw, "%s: Error in setting Tick register\n", __func__);

	/* Get Rx Packet count statistics */
	ret = xcmac_get_rx_packet_statistics(&xpriv->cmac_instance, &rx_hist);
	if (ret != 0) {
		qep_err(hw,
			"%s: Error in retrieving Rx packet count statistics\n",
			__func__);
	}

	/* Get Tx Packet count statistics */
	ret = xcmac_get_tx_packet_statistics(&xpriv->cmac_instance, &tx_hist);
	if (ret != 0) {
		qep_err(hw,
			"%s: Error in retrieving Tx packet count statistics\n",
			__func__);
	}

	ret = xcmac_get_tx_malformed_statistics(&xpriv->cmac_instance,
						&tx_malformed);
	if (ret != 0) {
		qep_err(hw, "%s: Error in retrieving Tx Malformed statistics\n",
			__func__);
	}

	ret = xcmac_get_rx_malformed_statistics(&xpriv->cmac_instance,
						&rx_malformed);
	if (ret != 0) {
		qep_err(hw, "%s: Error in retrieving Rx Malformed statistics\n",
			__func__);
	}

	stats->rx_mac_bytes += rx_hist.total_bytes;
	stats->rx_good_bytes += rx_hist.total_good_bytes;
	stats->rx_mac_packets += rx_hist.total_packets;
	stats->rx_good_pkts += rx_hist.total_good_packets;
	stats->rx_unicast_pkts += rx_pkt_types.packets_unicast;
	stats->rx_multicast_pkts += rx_pkt_types.packets_multicast;
	stats->rx_broadcast_pkts += rx_pkt_types.packets_broadcast;
	stats->rx_pause_pkts += rx_pkt_types.packets_pause;
	stats->rx_vlan_tagged_pkts += rx_pkt_types.packets_vlan;
	stats->rx_priority_pause_pkts += rx_pkt_types.packets_priority_pause;

	stats->rx_bad_fcs_pkts += rx_malformed.bad_fcs_count;
	stats->rx_bad_code_pkts +=
		xcmac_get_rx_bad_64b66b_code_count(&xpriv->cmac_instance);
	stats->rx_too_long_pkts += rx_malformed.toolong_count;
	stats->rx_oversized_pkts += rx_malformed.oversize_count;
	stats->rx_jabber_pkts += rx_malformed.jabber_count;
	stats->rx_undersized_pkts += rx_malformed.under_size_count;
	stats->rx_frag_pkts += rx_malformed.fragment_count;
	stats->rx_stomped_fcs_pkts += rx_malformed.stomped_fcs_count;

	stats->rx_range_error_pkts +=
		xcmac_get_rx_range_error_count(&xpriv->cmac_instance);
	stats->rx_truncated_pkts +=
		xcmac_get_rx_truncated_count(&xpriv->cmac_instance);
	stats->rx_hist_less_64_pkts += rx_hist.packets_small;
	stats->rx_hist_64_pkts += rx_hist.packets_0_to_64_bytes;
	stats->rx_hist_65_127_pkts += rx_hist.packets_65_to_127_bytes;
	stats->rx_hist_128_255_pkts += rx_hist.packets_128_to_255_bytes;
	stats->rx_hist_256_511_pkts += rx_hist.packets_256_to_511_bytes;
	stats->rx_hist_512_1023_pkts += rx_hist.packets_512_to_1023_bytes;
	stats->rx_hist_1024_1518_pkts += rx_hist.packets_1024_to_1518_bytes;
	stats->rx_hist_1519_1522_pkts += rx_hist.packets_1519_to_1522_bytes;
	stats->rx_hist_1523_1548_pkts += rx_hist.packets_1523_to_1548_bytes;
	stats->rx_hist_1549_2047_pkts += rx_hist.packets_1549_to_2047_bytes;
	stats->rx_hist_2048_4095_pkts += rx_hist.packets_2048_to_4095_bytes;
	stats->rx_hist_4096_8191_pkts += rx_hist.packets_4096_to_8191_bytes;
	stats->rx_hist_8192_9215_pkts += rx_hist.packets_8192_to_9215_bytes;
	stats->rx_hist_greter_eq_9216_pkts += rx_hist.packets_large;

	stats->tx_mac_bytes += tx_hist.total_bytes;
	stats->tx_good_bytes += tx_hist.total_good_bytes;
	stats->tx_mac_packets += tx_hist.total_packets;
	stats->tx_good_pkts += tx_hist.total_good_packets;
	stats->tx_unicast_pkts += tx_pkt_types.packets_unicast;
	stats->tx_multicast_pkts += tx_pkt_types.packets_multicast;
	stats->tx_broadcast_pkts += tx_pkt_types.packets_broadcast;
	stats->tx_pause_pkts += tx_pkt_types.packets_pause;
	stats->tx_vlan_tagged_pkts += tx_pkt_types.packets_vlan;
	stats->tx_priority_pause_pkts += tx_pkt_types.packets_priority_pause;
	stats->tx_bad_fcs_pkts += tx_malformed.bad_fcs_count;
	stats->tx_frame_error_pkts +=
		xcmac_get_tx_frame_error_count(&xpriv->cmac_instance);
	stats->tx_hist_less_64_pkts += tx_hist.packets_small;
	stats->tx_hist_64_pkts += tx_hist.packets_0_to_64_bytes;
	stats->tx_hist_65_127_pkts += tx_hist.packets_65_to_127_bytes;
	stats->tx_hist_128_255_pkts += tx_hist.packets_128_to_255_bytes;
	stats->tx_hist_256_511_pkts += tx_hist.packets_256_to_511_bytes;
	stats->tx_hist_512_1023_pkts += tx_hist.packets_512_to_1023_bytes;
	stats->tx_hist_1024_1518_pkts += tx_hist.packets_1024_to_1518_bytes;
	stats->tx_hist_1519_1522_pkts += tx_hist.packets_1519_to_1522_bytes;
	stats->tx_hist_1523_1548_pkts += tx_hist.packets_1523_to_1548_bytes;
	stats->tx_hist_1549_2047_pkts += tx_hist.packets_1549_to_2047_bytes;
	stats->tx_hist_2048_4095_pkts += tx_hist.packets_2048_to_4095_bytes;
	stats->tx_hist_4096_8191_pkts += tx_hist.packets_4096_to_8191_bytes;
	stats->tx_hist_8192_9215_pkts += tx_hist.packets_8192_to_9215_bytes;
	stats->tx_hist_greter_eq_9216_pkts += tx_hist.packets_large;

	spin_unlock_irqrestore(&xpriv->stats_lock, flags);
}

#define QEP_RSS_INDR_TBL_SIZE 256
#define QEP_RSS_HKEY_SIZE 40
#define QEP_RSS_OFFSET 0x00300000
#define QEP_RSS_INDR_DATA_MASK 0x7ff

#if KERNEL_VERSION(4, 5, 0) < LINUX_VERSION_CODE
static u32 qep_get_rxfh_key_size(struct net_device *netdev)
{
	return QEP_RSS_HKEY_SIZE;
}
#endif

static u32 qep_rss_indir_size(struct net_device *netdev)
{
	return QEP_RSS_INDR_TBL_SIZE;
}

static void qep_get_reta(struct qep_priv *xpriv, u32 *indir)
{
	int i, reta_size = QEP_RSS_INDR_TBL_SIZE;
	void __iomem *io_addr;
	unsigned long flags;

	io_addr = (void __iomem *)((u64)xpriv->bar_base +
			QEP_RSS_OFFSET);

	spin_lock_irqsave(&xpriv->stats_lock, flags);
	for (i = 0; i < reta_size; i++)
		indir[i] = readl((void __iomem *)((u32 *)io_addr + i)) &
					QEP_RSS_INDR_DATA_MASK;

	spin_unlock_irqrestore(&xpriv->stats_lock, flags);

}


static int qep_set_reta(struct qep_priv *xpriv, const u32 *indir)
{
	int i, reta_size = QEP_RSS_INDR_TBL_SIZE;
	void __iomem *io_addr;
	unsigned long flags;

	io_addr = (void __iomem *)((u64)xpriv->bar_base +
			QEP_RSS_OFFSET);

	for (i = 0; i < reta_size; i++) {
		if (indir[i] > xpriv->netdev->real_num_rx_queues) {
			pr_err("Invalid Indirection Table");
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&xpriv->stats_lock, flags);
	for (i = 0; i < reta_size; i++)
		writel(indir[i], (void __iomem *)((u32 *)io_addr + i));

	spin_unlock_irqrestore(&xpriv->stats_lock, flags);
	return 0;
}

int qep_init_reta(struct qep_priv *xpriv, u32 start, u32 q)
{
	u32 *indir = NULL;
	u32 i;
	int ret = 0;

	indir = kcalloc(QEP_RSS_INDR_TBL_SIZE, sizeof(u32), GFP_KERNEL);
	if (!indir)
		return -ENOMEM;

	for (i = 0; i < QEP_RSS_INDR_TBL_SIZE; i++)
		indir[i] = start + (i % q);

	ret = qep_set_reta(xpriv, indir);
	kfree(indir);

	return ret;
}
static int qep_get_rxfh(struct net_device *netdev, u32 *indir, u8 *key,
			  u8 *hfunc)
{

	struct qep_priv *xpriv;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return -EINVAL;

	if (hfunc)
		*hfunc = ETH_RSS_HASH_TOP;

	if (indir)
		qep_get_reta(xpriv, indir);

	return 0;
}


static int qep_set_rxfh(struct net_device *netdev, const u32 *indir,
			  const u8 *key, const u8 hfunc)
{
	struct qep_priv *xpriv;
	int ret = -EOPNOTSUPP;

	xpriv = netdev_priv(netdev);
	if (!xpriv)
		return -EINVAL;

	if (indir) {
		qep_set_reta(xpriv, indir);
		ret = 0;
	}

	if (key)
		ret = -EOPNOTSUPP;

	if (hfunc)
		ret = -EOPNOTSUPP;

	return ret;
}

static int qep_get_rxnfc(struct net_device *dev, struct ethtool_rxnfc *cmd,
			   u32 *rule_locs)
{
	int ret = -EOPNOTSUPP;

	switch (cmd->cmd) {
	case ETHTOOL_GRXRINGS:
		cmd->data = dev->real_num_rx_queues;
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static const struct ethtool_ops qep_ethtool_ops = {
	.get_drvinfo = qep_get_drvinfo,
	.get_regs_len = qep_get_regs_len,
	.get_regs = qep_get_regs,
	.get_link = ethtool_op_get_link,
	.get_msglevel = qep_get_msglevel,
	.set_msglevel = qep_set_msglevel,
	.get_ringparam = qep_get_ringparam,
	.set_ringparam = qep_set_ringparam,
	.get_channels = qep_get_channels,
	.set_channels = qep_set_channels,
	.get_strings = qep_get_strings,
	.get_ethtool_stats = qep_get_ethtool_stats,
	.get_coalesce = qep_get_coalesce,
	.set_coalesce = qep_set_coalesce,
#if KERNEL_VERSION(4, 5, 0) < LINUX_VERSION_CODE
	.get_per_queue_coalesce = qep_get_per_queue_coalesce,
	.set_per_queue_coalesce = qep_set_per_queue_coalesce,
	.get_fecparam = qep_get_fecparam,
	.set_fecparam = qep_set_fecparam,
	.get_rxfh_key_size	= qep_get_rxfh_key_size,
#endif
	.get_rxfh_indir_size	= qep_rss_indir_size,
	.get_rxfh		= qep_get_rxfh,
	.set_rxfh		= qep_set_rxfh,
	.get_rxnfc		= qep_get_rxnfc,
	//.set_rxnfc		= qep_set_rxnfc,
	.get_sset_count = qep_get_sset_count
};

void qep_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &qep_ethtool_ops;
}
