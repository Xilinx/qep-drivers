/*
 * Copyright (c) 2019-2020 Xilinx, Inc.
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

#define OBJ_SIZE(m) (sizeof(((struct qep_drv_stats *)0)->m))
#define OBJ_OFFSET(m) (offsetof(struct qep_drv_stats, m))

static const struct qep_stats qep_drv_stats[] = {
	{ "rx_packets", OBJ_SIZE(rx_packets), OBJ_OFFSET(rx_packets) },
	{ "tx_packets", OBJ_SIZE(tx_packets), OBJ_OFFSET(tx_packets) },
	{ "rx_bytes", OBJ_SIZE(rx_bytes), OBJ_OFFSET(rx_bytes)},
	{ "tx_bytes", OBJ_SIZE(tx_bytes), OBJ_OFFSET(tx_bytes) },
	{ "rx_pkt_l3_chksum_valid",
			OBJ_SIZE(rx_pkt_l3_csum),
			OBJ_OFFSET(rx_pkt_l3_csum) },
	{ "rx_pkt_l4_chksum_valid",
			OBJ_SIZE(rx_pkt_l4_csum),
			OBJ_OFFSET(rx_pkt_l4_csum)},
	{ "tx_pkt_l3_chksumed",
			OBJ_SIZE(tx_pkt_l3_csum),
			OBJ_OFFSET(tx_pkt_l3_csum) },
	{ "tx_pkt_l4_chksumed",
			OBJ_SIZE(tx_pkt_l4_csum),
			OBJ_OFFSET(tx_pkt_l4_csum)},
	{ "tx_timeout_count",
			OBJ_SIZE(tx_timeout_count),
			OBJ_OFFSET(tx_timeout_count)},
};

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

int qep_drv_stats_get_size(void)
{
	return ARRAY_SIZE(qep_drv_stats);
}

int qep_drv_stats_get_string(char *buf, int len)
{
	int i = 0;
	int count = qep_drv_stats_get_size();

	if (!buf) {
		pr_err("%s: buf is NULL\n", __func__);
		return -EINVAL;
	}

	if (len < count * QEP_CMAC_STATS_NAME_MAX_LEN) {
		pr_err("%s: buf length %d is less than required %d\n",
			__func__, len, count);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		memcpy(buf, qep_drv_stats[i].name, QEP_CMAC_STATS_NAME_MAX_LEN);
		buf += QEP_CMAC_STATS_NAME_MAX_LEN;
	}

	return 0;
}

int qep_drv_stats_get_data(struct qep_drv_stats *stats, u64 *data, int len)
{
	char *p = NULL;
	int count;
	int stats_len = qep_drv_stats_get_size();

	if (!stats || !data) {
		pr_err("%s: invalid args\n", __func__);
		return -EINVAL;
	}

	if (len < stats_len) {
		pr_err("%s: buf length %d is less then required %d\n",
			__func__, len, stats_len);
		return -EINVAL;
	}

	for (count = 0; count < stats_len; count++) {
		p = (char *)stats + qep_drv_stats[count].offset;

		if (qep_drv_stats[count].size == sizeof(u64))
			data[count] = *(u64 *)p;
		else
			data[count] = *(u32 *)p;
	}

	return 0;
}

int qep_drv_stats_snprintf(struct qep_drv_stats *stats, char *msg, int len)
{
	u8 *p;
	u8 *stat_string;
	u64 *stat_data;
	int ret = 0;
	int i;
	int stats_len = qep_drv_stats_get_size();

	if (!stats || !msg) {
		pr_err("%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	stat_string = kcalloc(stats_len, QEP_CMAC_STATS_NAME_MAX_LEN,
			GFP_KERNEL);
	if (!stat_string)
		return -ENOMEM;

	stat_data = kcalloc(stats_len,  sizeof(u64), GFP_KERNEL);
	if (!stat_data) {
		kfree(stat_string);
		return -ENOMEM;
	}

	ret = qep_drv_stats_get_string(stat_string,
			stats_len * QEP_CMAC_STATS_NAME_MAX_LEN);
	if (ret) {
		pr_err("%s: qep_drv_stats_get_string() failed\n", __func__);
		goto fn_exit;
	}

	ret = qep_drv_stats_get_data(stats, stat_data, stats_len);
	if (ret) {
		pr_err("%s: qep_drv_stats_get_data() failed\n", __func__);
		goto fn_exit;
	}

	p = stat_string;
	for (i = 0; i < stats_len; i++) {
		snprintf((u8 *)msg + strlen(msg), len - strlen(msg),
			 "%-32s = %llu\n", p, stat_data[i]);
		p += QEP_CMAC_STATS_NAME_MAX_LEN;
	}

fn_exit:
	kfree(stat_string);
	kfree(stat_data);
	return ret;
}

/* This is a ethtool callback and register dump length/size */
static int qep_get_regs_len(struct net_device *netdev)
{
	return QEP_REGS_LEN * sizeof(u32);
}

/* This is a ethtool callback and gets stats length */
static int qep_get_sset_count(struct net_device *netdev, int sset)
{
	int len;

	switch (sset) {
	case ETH_SS_STATS:
		len = qep_cmac_stats_get_size() +
			qep_drv_stats_get_size();
		return len;
	default:
		return -EOPNOTSUPP;
	}
}

/* This is a ethtool callback and prints driver info */
static void qep_get_drvinfo(struct net_device *netdev,
			    struct ethtool_drvinfo *drvinfo)
{
	struct qep_priv *xpriv = netdev_priv(netdev);

	strlcpy(drvinfo->driver, DRV_NAME, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, DRV_VER, sizeof(drvinfo->version));
	strlcpy(drvinfo->bus_info, pci_name(xpriv->pcidev),
		sizeof(drvinfo->bus_info));
	drvinfo->regdump_len = qep_get_regs_len(netdev);
	drvinfo->n_stats = qep_get_sset_count(netdev, ETH_SS_STATS);
}


/* This is a ethtool callback and prints register dump */
static void qep_get_regs(struct net_device *netdev, struct ethtool_regs *regs,
			 void *p)
{
	u32 *regs_buff = p;
	unsigned int i = 0, ret = 0, reg_idx = 0;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);
	u32 offset;
	u32 cmac_regs[] = {
		XCMAC_CMAC_VERSION_REG_OFFSET,
		XCMAC_CONFIGURATION_TX_REG1_OFFSET,
		XCMAC_CONFIGURATION_RX_REG1_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_REG1_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG1_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG2_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG3_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG4_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG5_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG1_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG2_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG3_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG4_OFFSET,
		XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG5_OFFSET,
		CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET,
		CONFIGURATION_RX_FLOW_CONTROL_REG2_OFFSET
	};

	memset(p, 0, QEP_REGS_LEN * sizeof(u32));

	offset = qep_fdt_get_netip_offset(xpriv->pinfo, QEP_NETPF_CMAC_IP);
	for (i = 0; i < sizeof(cmac_regs) / sizeof(u32); i++) {
		regs_buff[reg_idx++] =  xcmac_in32((u64)(xpriv->bar_base) +
			offset + cmac_regs[i]);
	}

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

	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return;
	}

	if (!ring) {
		qep_err(drv, "%s: ring is NULL\n", __func__);
		return;
	}

	memset(&csr_conf, 0, sizeof(struct global_csr_conf));

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

	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (!ring) {
		qep_err(drv, "%s: ring is NULL\n", __func__);
		return -EINVAL;
	}

	if (ring->rx_jumbo_pending) {
		qep_err(drv, "%s: rx_jumbo_pending not supported\n",
			    __func__);
		return -EINVAL;
	}
	if (ring->rx_mini_pending) {
		qep_err(drv, "%s: rx_mini_pending not supported\n",
			    __func__);
		return -EINVAL;
	}

	if (ring->rx_pending < QEP_MIN_RING_SIZE
			|| ring->tx_pending < QEP_MIN_RING_SIZE) {
		qep_err(drv, "%s: ring size more than min (%d)\n",
			    __func__, QEP_MIN_RING_SIZE);
		return -EINVAL;
	}

	memset(&csr_conf, 0, sizeof(struct global_csr_conf));
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
		qep_err(drv, "%s: qep_stop()  failed with status %d\n",
				__func__, ret);
		return ret;
	}

	usleep_range(1000, 2000);
	ret = qep_open(netdev);
	if (ret != 0) {
		qep_err(drv, "%s: qep_open() failed with status %d\n",
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

	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return;
	}

	if (!ch) {
		qep_err(drv, "%s: ethtool_channels is NULL\n", __func__);
		return;
	}

	ch->max_rx = xpriv->pinfo->stmn_queue_max;
	ch->max_tx =  xpriv->pinfo->stmn_queue_max;
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

	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (!ch) {
		qep_err(drv, "%s: ethtool_channels is NULL\n", __func__);
		return -EINVAL;
	}

	if ((ch->rx_count == 0) || (ch->tx_count == 0)) {
		qep_err(drv,
			"%s: invalid channel count passed, Rx = %d, Tx = %d\n",
			__func__, ch->rx_count, ch->tx_count);
		return -EINVAL;
	}

	if ((ch->rx_count >  xpriv->pinfo->stmn_queue_max) ||
	    (ch->tx_count >  xpriv->pinfo->stmn_queue_max)) {
		qep_err(drv, "%s: Tx Ch count %d Rx Ch count %d passed more than max supported %d\n",
				__func__, ch->tx_count,
				ch->rx_count,  xpriv->pinfo->stmn_queue_max);
		return -EINVAL;
	}

	qep_dbg(netdev, "%s: rx_count = %d, tx_count = %d, combined = %d\n",
		__func__, ch->rx_count, ch->tx_count, ch->combined_count);

	if (netif_running(netdev)) {
		iface_up = 1;
		ret = qep_stop(netdev); /* Stop the interface */
		if (ret != 0) {
			qep_err(drv, "%s: qep_stop() failed with status %d\n",
					__func__, ret);
			return ret;
		}
	}

	netif_set_real_num_tx_queues(netdev, ch->tx_count);
	netif_set_real_num_rx_queues(netdev, ch->rx_count);

	if (xpriv->netdev->features & NETIF_F_RXHASH) {
		ret = qep_init_reta(xpriv, 0, ch->rx_count);
		if (ret != 0) {
			qep_err(drv, "%s: qep_init_reta() failed with status %d\n",
					__func__, ret);
			return ret;
		}
	}

	if (!iface_up)
		return 0;

	usleep_range(1000, 2000);
	ret = qep_open(netdev); /* Open the interface */
	if (ret != 0) {
		qep_err(drv, "%s: qep_open() failed with status %d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

/* This is a ethtool callback and populates  stats as strings*/
static void qep_get_strings(struct net_device *netdev, u32 stringset, u8 *data)
{
	char *buf = (char *)data;
	int ret;
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return;
	}

	if (!data) {
		qep_err(drv, "%s: data is NULL\n", __func__);
		return;
	}

	switch (stringset) {
	case ETH_SS_STATS: {
		int drv_buf_len = qep_drv_stats_get_size() * ETH_GSTRING_LEN;
		int cmac_buf_len = qep_cmac_stats_get_size() *  ETH_GSTRING_LEN;

		ret = qep_drv_stats_get_string(buf, drv_buf_len);
		if (ret) {
			qep_err(drv, "%s: drv_stats_get_string() failed with status %d\n"
					, __func__, ret);
			return;
		}
		ret = qep_cmac_stats_get_strings(buf + drv_buf_len,
				cmac_buf_len,
				QEP_CMAC_STATS_INCR_32B);
		if (ret) {
			qep_err(drv, "%s: cmac_stats_get_string() failed with status %d\n",
					__func__, ret);
			return;
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
	int ret, dlen = 0, clen = 0;
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return;
	}

	if (!data) {
		qep_err(drv, "%s: data is NULL\n", __func__);
		return;
	}

	if (!stats) {
		qep_err(drv, "%s: ethtool_stats is NULL\n", __func__);
		return;
	}

	clen = qep_cmac_stats_get_size();
	dlen = qep_drv_stats_get_size();

	ret = qep_cmac_stats_get(&xpriv->cmac_dev, xpriv->mac_stats);
	if (ret) {
		qep_err(drv, "%s: qep_cmac_stats_get() failed with status %d\n",
				__func__, ret);
		return;
	}

	ret = qep_drv_stats_get_data(xpriv->drv_stats, data, dlen);
	if (ret) {
		qep_err(drv, "%s: qep_drv_stats_get_data() failed with status %d\n",
				__func__, ret);
		return;
	}
	data = data + dlen;
	ret = qep_cmac_stats_get_data(xpriv->mac_stats, data, clen);
	if (ret) {
		qep_err(drv, "%s: qep_cmac_stats_get_data() failed with status %d\n",
				__func__, ret);
		return;
	}

}

/* This is a ethtool callback and gets global coalesce options */
static int qep_get_coalesce(struct net_device *netdev,
			    struct ethtool_coalesce *ec)
{
	int ret = 0;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (!ec) {
		qep_err(drv, "%s: ethtool_coalesce is NULL\n", __func__);
		return -EINVAL;
	}

	memset(&csr_conf, 0, sizeof(struct global_csr_conf));
	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return ret;
	}
	ec->use_adaptive_rx_coalesce = xpriv->rx_q[0].adaptive_update;
	ec->use_adaptive_tx_coalesce = xpriv->tx_q[0].adaptive_update;
	/* assign default values initially */
	ec->rx_max_coalesced_frames =
		csr_conf.c2h_cnt_th[xpriv->rx_cnt_th_idx];
	ec->rx_coalesce_usecs =
		csr_conf.c2h_timer_cnt[xpriv->rx_timer_idx];
	if (!ec->use_adaptive_rx_coalesce) { /* assign configured values */
		/* common for all queues */
		ec->rx_max_coalesced_frames =
				xpriv->rx_q[0].coalesce_frames;
		ec->rx_coalesce_usecs =
				xpriv->rx_q[0].coalesce_usecs;
	}
	ec->tx_max_coalesced_frames = xpriv->tx_q[0].coalesce_frames;
	ec->tx_coalesce_usecs = xpriv->tx_q[0].coalesce_usecs;

	return 0;
}

/* This is a ethtool callback and sets global coalesce options */
static int qep_set_coalesce(struct net_device *netdev,
			    struct ethtool_coalesce *ec)
{
	int ret = 0;
	int index;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);
	unsigned int i;

	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (!ec) {
		qep_err(drv, "%s: ethtool_coalesce is NULL\n", __func__);
		return -EINVAL;
	}

	/* Get the global CSR , get the index of the ring size array and
	 * then modify the ring size
	 */
	memset(&csr_conf, 0, sizeof(struct global_csr_conf));
	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	if (!ec->use_adaptive_rx_coalesce && ec->rx_max_coalesced_frames) {
		index = qep_arr_find(csr_conf.c2h_cnt_th,
				     QDMA_GLOBAL_CSR_ARRAY_SZ,
				     ec->rx_max_coalesced_frames);
		if (index < 0) {
			qep_err(drv,
				"%s: Invalid rx_max_coalesced_frames = %d\n",
				__func__, ec->rx_max_coalesced_frames);
			return -EINVAL;
		}
	}
	if (!ec->use_adaptive_rx_coalesce && ec->rx_coalesce_usecs) {
		index = qep_arr_find(csr_conf.c2h_timer_cnt,
				     QDMA_GLOBAL_CSR_ARRAY_SZ,
				     ec->rx_coalesce_usecs);
		if (index < 0) {
			qep_err(drv,
				"%s: Invalid rx_coalesce_usecs = %d\n",
				__func__, ec->rx_coalesce_usecs);
			return -EINVAL;
		}
	}
	for (i = 0; i < xpriv->netdev->real_num_rx_queues; i++) {
		if (ec->use_adaptive_rx_coalesce)
			xpriv->rx_q[i].adaptive_update = 1;
		else {
			if (ec->rx_max_coalesced_frames)
				xpriv->rx_q[i].coalesce_frames =
						ec->rx_max_coalesced_frames;
			if (ec->rx_coalesce_usecs)
				xpriv->rx_q[i].coalesce_usecs =
						ec->rx_coalesce_usecs;
		}
	}
	for (i = 0; i < xpriv->netdev->real_num_tx_queues; i++) {
		if (ec->use_adaptive_tx_coalesce) /* ignore, not supported */
			xpriv->tx_q[i].adaptive_update = 0;
		else {
			if (ec->tx_max_coalesced_frames)
				xpriv->tx_q[i].coalesce_frames =
						ec->tx_max_coalesced_frames;
			if (ec->tx_coalesce_usecs)
				xpriv->tx_q[i].coalesce_usecs =
						ec->tx_coalesce_usecs;
		}
	}
	if (netif_running(netdev)) {
		/* Stop the interface */
		ret = qep_stop(netdev);
		if (ret != 0) {
			qep_err(drv, "%s: qep_stop() failed with status %d\n",
					__func__, ret);
			return ret;
		}
		/* Open the interface */
		ret = qep_open(netdev);
		if (ret != 0) {
			qep_err(drv, "%s: qep_open() failed with status %d\n",
					__func__, ret);
			return ret;
		}
	}

	return 0;
}

#if KERNEL_VERSION(4, 5, 0) < LINUX_VERSION_CODE
/* This is a ethtool callback and gets per queue coalesce options */
static int qep_get_per_queue_coalesce(struct net_device *netdev, u32 queue,
				      struct ethtool_coalesce *ec)
{
	int ret = 0;
	struct qep_priv *xpriv = netdev_priv(netdev);

	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (!ec) {
		qep_err(drv, "%s: ethtool_coalesce is NULL\n", __func__);
		return -EINVAL;
	}

	if ((queue >= xpriv->netdev->real_num_rx_queues) &&
			(queue >= xpriv->netdev->real_num_tx_queues)) {
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

	if (queue < xpriv->netdev->real_num_rx_queues) {
		ec->use_adaptive_rx_coalesce =
				xpriv->rx_q[queue].adaptive_update;
		ec->rx_max_coalesced_frames =
				xpriv->rx_q[queue].coalesce_frames;
		ec->rx_coalesce_usecs =
			xpriv->rx_q[queue].coalesce_usecs;
	}
	if (queue < xpriv->netdev->real_num_tx_queues) {
		ec->use_adaptive_tx_coalesce =
				xpriv->tx_q[queue].adaptive_update;
		ec->tx_max_coalesced_frames =
				xpriv->tx_q[queue].coalesce_frames;
		ec->tx_coalesce_usecs = xpriv->tx_q[queue].coalesce_usecs;
	}

	return ret;
}

/* This is a ethtool callback and sets per queue coalesce options */
static int qep_set_per_queue_coalesce(struct net_device *netdev, u32 queue,
				      struct ethtool_coalesce *ec)
{
	int ret = 0, index = 0;
	struct global_csr_conf csr_conf;
	struct qep_priv *xpriv = netdev_priv(netdev);
	u8 valid_rx_q = (queue < xpriv->netdev->real_num_rx_queues) ? 1 : 0;
	u8 valid_tx_q = (queue < xpriv->netdev->real_num_tx_queues) ? 1 : 0;

	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (!ec) {
		qep_err(drv, "%s: ethtool_coalesce is NULL\n", __func__);
		return -EINVAL;
	}


	if (!valid_rx_q && !valid_tx_q) {
		qep_err(drv, "%s: Invalid queue id entered = %d\n", __func__,
			queue);
		return -EINVAL;
	}

	if (valid_rx_q && !ec->use_adaptive_rx_coalesce &&
			(ec->rx_max_coalesced_frames == 0) &&
			(ec->rx_coalesce_usecs == 0)) {
		qep_err(drv,
			"%s: Invalid parameters entered supported only rx_max_coalesced_frames and/or rx_coalesce_usecs\n",
			__func__);
		return -EINVAL;
	}
	if (valid_tx_q && !ec->use_adaptive_tx_coalesce &&
			(ec->tx_max_coalesced_frames == 0) &&
			(ec->tx_coalesce_usecs == 0)) {
		qep_err(drv,
			"%s: Invalid parameters entered supported only rx_max_coalesced_frames and/or rx_coalesce_usecs\n",
			__func__);
		return -EINVAL;
	}

	/* Get the global CSR , get the index of the ring size array and
	 * then modify the ring size
	 */
	memset(&csr_conf, 0, sizeof(struct global_csr_conf));
	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &csr_conf);
	if (ret != 0) {
		qep_err(drv,
			"%s: qdma_global_csr_get() failed with status %d\n",
			__func__, ret);
		return ret;
	}

	/* Check if user wants to set coalesced frames */
	if (valid_rx_q && !ec->use_adaptive_rx_coalesce &&
			ec->rx_max_coalesced_frames) {
		if (ec->rx_max_coalesced_frames !=
		    xpriv->rx_q[queue].coalesce_frames) {
			index = qep_arr_find(csr_conf.c2h_cnt_th,
					     QDMA_GLOBAL_CSR_ARRAY_SZ,
					     ec->rx_max_coalesced_frames);
			if (index < 0) {
				qep_err(drv,
					"%s: Invalid rx_max_coalesced_frames = %d\n",
					__func__, ec->rx_max_coalesced_frames);
				return -EINVAL;
			}
		} else
			qep_info(
				drv,
				"%s: Trying to set same rx_max_coalesced_frames = %d, Ignored!\n",
				__func__, ec->rx_max_coalesced_frames);
	}

	/* Check if user wants to set coalesced usecs */
	if (valid_rx_q && !ec->use_adaptive_rx_coalesce &&
			ec->rx_coalesce_usecs) {
		if (ec->rx_coalesce_usecs !=
				xpriv->rx_q[queue].coalesce_frames) {
			index = qep_arr_find(csr_conf.c2h_timer_cnt,
					     QDMA_GLOBAL_CSR_ARRAY_SZ,
					     ec->rx_coalesce_usecs);
			if (index < 0) {
				qep_err(drv,
					"%s: Invalid rx_coalesce_usecs = %d\n",
					__func__, ec->rx_coalesce_usecs);
				return -EINVAL;
			}
		} else
			qep_info(
				drv,
				"%s: Trying to set same rx_coalesce_usecs = %d, Ignored!\n",
				__func__, ec->rx_coalesce_usecs);
	}
	if (valid_rx_q && !ec->use_adaptive_rx_coalesce) {
		if (ec->rx_max_coalesced_frames)
			xpriv->rx_q[queue].coalesce_frames =
					ec->rx_max_coalesced_frames;
		if (ec->rx_coalesce_usecs)
			xpriv->rx_q[queue].coalesce_usecs =
					ec->rx_coalesce_usecs;
	}
	if (valid_tx_q && !ec->use_adaptive_tx_coalesce) {
		if (ec->tx_max_coalesced_frames)
			xpriv->tx_q[queue].coalesce_frames =
					ec->tx_max_coalesced_frames;
		if (ec->tx_coalesce_usecs)
			xpriv->tx_q[queue].coalesce_usecs =
					ec->tx_coalesce_usecs;
	}
	if (netif_running(netdev)) {
		/* Stop the interface */
		ret = qep_stop(netdev);
		if (ret != 0) {
			qep_err(drv, "%s: qep_stop() failed with status %d\n",
					__func__, ret);
			return ret;
		}
		/* Open the interface */
		ret = qep_open(netdev);
		if (ret != 0) {
			qep_err(drv, "%s: qep_open() failed with status %d\n",
					__func__, ret);
			return ret;
		}
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
	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (!fec) {
		qep_err(drv, "%s: ethtool_fecparam is NULL\n", __func__);
		return -EINVAL;
	}


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
	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (!fec) {
		qep_err(drv, "%s: ethtool_fecparam is NULL\n", __func__);
		return -EINVAL;
	}

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
		ret = xcmac_config_rsfec(&xpriv->cmac_dev,
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

	ret = xcmac_reset_gt(&xpriv->cmac_dev);
	if (ret)
		return ret;

	return ret;
}
#endif

/* This function reads CMAC stats */



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
		qep_fdt_get_netip_offset(xpriv->pinfo, QEP_NETPF_RSS));

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
		qep_fdt_get_netip_offset(xpriv->pinfo, QEP_NETPF_RSS));

	for (i = 0; i < reta_size; i++) {
		if (indir[i] > xpriv->netdev->real_num_rx_queues) {
			pr_err("%s: Invalid Indirection Table\n", __func__);
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
	u8 hw_key[] = {0x6d, 0x5a, 0x56, 0xda, 0x25, 0x5b, 0x0e, 0xc2,
			0x41, 0x67, 0x25, 0x3d, 0x43, 0xa3, 0x8f, 0xb0,
			0xd0, 0xca, 0x2b, 0xcb, 0xae, 0x7b, 0x30, 0xb4,
			0x77, 0xcb, 0x2d, 0xa3, 0x80, 0x30, 0xf2, 0x0c,
			0x6a, 0x42, 0xb7, 0x3b, 0xbe, 0xac, 0x01, 0xfa
		};

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	if (hfunc)
		*hfunc = ETH_RSS_HASH_TOP;

	if (indir)
		qep_get_reta(xpriv, indir);

	if (key)
		memcpy(key, hw_key, QEP_RSS_HKEY_SIZE);

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

static int get_lp_advertized(struct xcmac *cmac,
		struct ethtool_link_ksettings *link_ksettings)
{
	int ret;
	struct xcmac_autoneg_status sts_an;
	struct xcmac_autoneg_ability lp_adv;

	memset(&sts_an, 0, sizeof(struct xcmac_autoneg_status));
	memset(&lp_adv, 0, sizeof(struct xcmac_autoneg_ability));
	ethtool_link_ksettings_zero_link_mode(link_ksettings, lp_advertising);

	ret = xcmac_get_autoneg_status(cmac, &sts_an);
	if (ret) {
		pr_err("%s: xcmac_get_autoneg_status() failed with status %d\n",
				__func__, ret);
		return ret;
	}

	ret = xcmac_get_autoneg_ability(cmac, &lp_adv);
	if (ret) {
		pr_err("%s: xcmac_get_autoneg_ability() failed with status %d\n",
				__func__, ret);
		return ret;
	}

	if (!sts_an.lp_ability_valid)
		return 0;

	if (sts_an.lp_autoneg_able)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				lp_advertising, Autoneg);

	if (sts_an.lp_pause && !sts_an.lp_asm_dir)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				lp_advertising, Pause);

	if (sts_an.lp_asm_dir)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				lp_advertising, Asym_Pause);

	if (sts_an.lp_fec_25g_rs_ability)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				lp_advertising, FEC_RS);

	else if ((sts_an.lp_fec_10g_ability && sts_an.lp_fec_10g_request)
			|| sts_an.lp_fec_25g_baser_request)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				lp_advertising, FEC_BASER);
	else
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				lp_advertising, FEC_NONE);

	if (lp_adv.ctl_autoneg_ability_100gbase_cr10
			|| lp_adv.ctl_autoneg_ability_100gbase_cr4)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				lp_advertising, 100000baseCR4_Full);
	if (lp_adv.ctl_autoneg_ability_100gbase_kr4
			|| lp_adv.ctl_autoneg_ability_100gbase_kp4)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				lp_advertising, 100000baseKR4_Full);
	return 0;
}

static int get_advertized(struct xcmac *cmac,
		struct ethtool_link_ksettings *link_ksettings)
{
	int ret;
	struct xcmac_autoneg_config an_cfg;
	struct xcmac_autoneg_ability we_adv;

	memset(&an_cfg, 0, sizeof(struct xcmac_autoneg_config));
	memset(&we_adv, 0, sizeof(struct xcmac_autoneg_ability));

	ethtool_link_ksettings_zero_link_mode(link_ksettings, advertising);

	ret = xcmac_get_config_autoneg(cmac, &an_cfg);
	if (ret) {
		pr_err("%s: xcmac_get_config_autoneg() failed with status %d\n",
				__func__, ret);
		return ret;
	}
	ret = xcmac_get_config_autoneg_ability(cmac, &we_adv);
	if (ret) {
		pr_err("%s: xcmac_get_config_autoneg_ability() failed with status %d\n",
				__func__, ret);
		return ret;
	}

	if (an_cfg.enable && !an_cfg.bypass)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				advertising, Autoneg);

	if (an_cfg.pause && !an_cfg.asmdir)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				advertising, Pause);
	if (an_cfg.asmdir)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				advertising, Asym_Pause);

	if (an_cfg.cl91_fec_ability && an_cfg.cl91_fec_request)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				advertising, FEC_RS);
	else
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				advertising, FEC_NONE);

	if (we_adv.ctl_autoneg_ability_100gbase_cr4
			|| we_adv.ctl_autoneg_ability_100gbase_cr10)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				advertising, 100000baseCR4_Full);

	if (we_adv.ctl_autoneg_ability_100gbase_kr4
			|| we_adv.ctl_autoneg_ability_100gbase_kp4)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				advertising, 100000baseKR4_Full);

	return 0;
}

static int get_supported(struct xcmac *cmac,
		struct ethtool_link_ksettings *link_ksettings)
{
	int ret;
	struct xcmac_autoneg_status sts_an;
	struct xcmac_autoneg_link_ctrl link_ctl;

	memset(&sts_an, 0, sizeof(struct xcmac_autoneg_status));
	memset(&link_ctl, 0, sizeof(struct xcmac_autoneg_link_ctrl));
	ethtool_link_ksettings_zero_link_mode(link_ksettings, supported);

	ethtool_link_ksettings_add_link_mode(link_ksettings,
			supported, FIBRE);

	ret = xcmac_get_autoneg_status(cmac, &sts_an);
	if (ret) {
		pr_err("%s: xcmac_get_autoneg_status() failed with status %d\n",
				__func__, ret);
		return ret;
	}

	ret = xcmac_get_autoneg_link_ctrl(cmac, &link_ctl);
	if (ret) {
		pr_err("%s: xcmac_get_autoneg_link_ctrl() failed with status %d\n",
				__func__, ret);
		return ret;
	}

	if (sts_an.autoneg_complete)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				supported, Autoneg);

	if (sts_an.rx_pause_enable && sts_an.rx_pause_enable)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				supported, Pause);
	else if (sts_an.rx_pause_enable || sts_an.rx_pause_enable)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				supported, Asym_Pause);

	if (sts_an.rs_fec_enable)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				supported, FEC_RS);
	else if (sts_an.fec_enable)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				supported, FEC_BASER);
	else
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				supported, FEC_NONE);

	if (link_ctl.stat_an_link_cntl_100gbase_cr4
			|| link_ctl.stat_an_link_cntl_100gbase_cr10)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				supported, 100000baseCR4_Full);
	if (link_ctl.stat_an_link_cntl_100gbase_kr4
			|| link_ctl.stat_an_link_cntl_100gbase_kp4)
		ethtool_link_ksettings_add_link_mode(link_ksettings,
				supported, 100000baseKR4_Full);

	return 0;
}
static int qep_get_link_ksettings(struct net_device *netdev,
		struct ethtool_link_ksettings *link_ksettings)
{
	int ret;
	struct qep_priv *xpriv;
	struct xcmac *cmac;

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}
	cmac = &xpriv->cmac_dev;

	link_ksettings->base.duplex = DUPLEX_FULL;
	link_ksettings->base.speed = SPEED_100000;
	link_ksettings->base.port = PORT_FIBRE;
	link_ksettings->base.transceiver = XCVR_INTERNAL;
	link_ksettings->base.phy_address = 0;
	link_ksettings->base.mdio_support = 0;
	link_ksettings->base.eth_tp_mdix = ETH_TP_MDI_INVALID;
	link_ksettings->base.eth_tp_mdix_ctrl = ETH_TP_MDI_INVALID;

	/* 802.3-2016 clause 73 */
	ret = get_advertized(cmac, link_ksettings);
	if (ret)
		qep_err(hw, "%s: get_advertized() failed\n", __func__);

	ret = get_lp_advertized(cmac, link_ksettings);
	if (ret)
		qep_err(hw, "%s: get_lp_advertized() failed\n", __func__);

	ret = get_supported(cmac, link_ksettings);
	if (ret)
		qep_err(hw, "%s: get_supported() failed\n", __func__);


	if (ethtool_link_ksettings_test_link_mode(link_ksettings,
			supported, Autoneg))
		link_ksettings->base.autoneg = AUTONEG_ENABLE;
	else
		link_ksettings->base.autoneg = AUTONEG_DISABLE;

	return 0;
}

struct autoneg_config {
	bool en;
	bool rsfec;
	bool tx_pause;
	bool rx_pause;
	bool link_mode;
	bool link_training;
};


static void qep_enable_link_training(struct xcmac *cmac, bool en)
{

	if (!en) {
		xcmac_out32(cmac->base_address
				+ XCMAC_CONFIG_LT_CONTROL_REG1_OFFSET,
				0x0);
		return;
	}

	/* Update default seed and coefficient for link training */
	xcmac_out32(cmac->base_address
			+ XCMAC_CONFIG_LT_CONTROL_REG1_OFFSET,
			0x1);
	xcmac_out32(cmac->base_address
			+ XCMAC_CONFIG_LT_TRAINED_REG_OFFSET,
			0xf);
	xcmac_out32(cmac->base_address
			+ XCMAC_CONFIG_LT_PRESET_REG_OFFSET,
			0xf);
	xcmac_out32(cmac->base_address
			+ XCMAC_CONFIG_LT_INIT_REG_OFFSET,
			0xf);
	xcmac_out32(cmac->base_address
			+ XCMAC_CONFIG_LT_SEED_REG0_OFFSET,
			0x06070605);
	xcmac_out32(cmac->base_address
			+ XCMAC_CONFIG_LT_SEED_REG1_OFFSET,
			0x06110609);
	xcmac_out32(cmac->base_address
			+ XCMAC_CONFIG_LT_COEFFICIENT_REG0_OFFSET,
			0x55555555);
	xcmac_out32(cmac->base_address
			+ XCMAC_CONFIG_LT_COEFFICIENT_REG1_OFFSET,
			0x55555555);
}

static int qep_enable_autoneg(struct xcmac *cmac, struct autoneg_config *an_cfg)
{
	int ret;
	struct xcmac_autoneg_config config;
	struct xcmac_autoneg_ability autoneg_ability;
	u8 random;

	memset(&config, 0, sizeof(struct xcmac_autoneg_config));
	memset(&autoneg_ability, 0, sizeof(struct xcmac_autoneg_ability));

	if (an_cfg->en) {
		config.enable = 1;
		config.bypass = 0;
		get_random_bytes(&random, sizeof(u8));
		config.nonce_seed = random;
		config.pseudo_sel = 1;
		config.loc_np = 0;
		config.lp_np_ack = 1;

		if (an_cfg->rsfec) {
			config.cl91_fec_ability  = 1;
			config.cl91_fec_request = 1;
		}

		if (an_cfg->tx_pause && an_cfg->rx_pause) {
			config.pause = 1;
			config.asmdir = 0;
		} else if (an_cfg->tx_pause && !an_cfg->rx_pause) {
			config.asmdir = 1;
			config.pause = 1;
		} else if (!an_cfg->tx_pause && an_cfg->rx_pause) {
			config.asmdir = 1;
			config.pause = 0;
		} else {
			config.asmdir = 0;
			config.pause = 0;
		}

		if (an_cfg->link_mode) {
			autoneg_ability.ctl_autoneg_ability_100gbase_cr4 = 1;
			autoneg_ability.ctl_autoneg_ability_100gbase_cr10 = 0;
			autoneg_ability.ctl_autoneg_ability_100gbase_kp4 = 0;
			autoneg_ability.ctl_autoneg_ability_100gbase_kr4 = 0;
		}

	} else {
		config.enable = 0;
		config.bypass = 1;
		config.restart = 0;
		config.nonce_seed = 0;
		config.pseudo_sel = 0;
	}

	ret = xcmac_config_autoneg_ability(cmac, &autoneg_ability);
	if (ret) {
		pr_err("%s: xcmac_config_autoneg_ability() failed with status %d\n",
				__func__, ret);
		return ret;
	}

	ret = xcmac_config_autoneg(cmac, &config);
	if (ret) {
		pr_err("%s: xcmac_config_autoneg failed() with status %d\n",
						__func__, ret);
		return ret;
	}

	if (an_cfg->link_training)
		qep_enable_link_training(cmac, an_cfg->en);

	ret = xcmac_reset_gt(cmac);
	if (ret) {
		pr_err("%s: xcmac_reset_gt() failed with status %d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

static int qep_set_link_ksettings(struct net_device *netdev,
		const struct ethtool_link_ksettings *link_ksettings)
{
	int ret;
	struct qep_priv *xpriv;
	struct xcmac *cmac;
	struct autoneg_config an_cfg;

	memset(&an_cfg, 0, sizeof(struct autoneg_config));

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	cmac = &xpriv->cmac_dev;

	netif_tx_stop_all_queues(netdev);

	if (link_ksettings->base.autoneg)
		an_cfg.en = true;
	else
		an_cfg.en = false;

	if (xpriv->rs_fec_en)
		an_cfg.rsfec = true;

	an_cfg.link_mode = true;
	if (link_ksettings->link_modes.advertising)
		pr_warn("%s: Ignoring advertised input\n", __func__);

	an_cfg.rx_pause = false;
	an_cfg.tx_pause = false;
	an_cfg.link_training = true;

	ret = qep_enable_autoneg(cmac, &an_cfg);
	if (ret != 0) {
		qep_err(drv, "%s: Autoneg failed, disable autoneg\n", __func__);
		return ret;
	}

	netif_tx_start_all_queues(netdev);

	return 0;
}

u32 qep_get_link(struct net_device *netdev)
{

	enum cmac_link_state link_state;
	struct qep_priv *xpriv;
	struct xcmac *cmac;

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	cmac = &xpriv->cmac_dev;
	link_state = qep_cmac_link_status(cmac);
	if (link_state != CMAC_LINK_UP)
		return 0;

	return 1;
}

int qep_nway_reset(struct net_device *netdev)
{
	int ret;
	u32 reg;
	struct qep_priv *xpriv;
	struct xcmac *cmac;
	struct xcmac_autoneg_config config;

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	cmac = &xpriv->cmac_dev;
	memset(&config, 0, sizeof(struct autoneg_config));
	ret = xcmac_get_config_autoneg(cmac, &config);
	if (ret)
		return ret;

	if (!config.enable) {
		pr_err("%s: autoneg not enabled\n", __func__);
		return -EINVAL;
	}

	netif_tx_stop_all_queues(netdev);

	config.restart = 1;
	ret = xcmac_config_autoneg(cmac, &config);
	if (ret) {
		qep_err(drv, "%s: xcmac_config_autoneg() failed with status %d\n",
				__func__, ret);
		return ret;
	}

	reg = xcmac_in32(cmac->base_address
			+ XCMAC_CONFIG_LT_CONTROL_REG1_OFFSET);
	reg |= 0x2;
	xcmac_out32(cmac->base_address
			+ XCMAC_CONFIG_LT_CONTROL_REG1_OFFSET,
			reg);

	ret = xcmac_reset_gt(cmac);
	if (ret) {
		qep_err(drv, "%s: xcmac_reset_gt() failed with status %d\n",
				__func__, ret);
		return ret;
	}

	netif_tx_start_all_queues(netdev);

	return 0;

}

int qep_reset(struct net_device *netdev, u32 *reset)
{
	int ret;
	struct qep_priv *xpriv;
	struct xcmac *cmac;

	xpriv = netdev_priv(netdev);
	if (!xpriv) {
		qep_err(drv, "%s: xpriv is NULL\n", __func__);
		return -EINVAL;
	}

	cmac = &xpriv->cmac_dev;

	if (*reset & ETH_RESET_PHY) {
		ret = xcmac_reset_gt(cmac);
		if (ret)
			return ret;

		*reset = *reset & ~ETH_RESET_PHY;
	}

	if (*reset & ETH_RESET_MAC) {
		ret = xcmac_reset_core(cmac, XCMAC_CORE_BOTH);
		if (ret != 0) {
			qep_err(hw, "%s: Error in resetting CMAC cores\n",
			__func__);
			return ret;
		}

		ret = qep_cmac_ctrl_reset(xpriv);
		if (ret) {
			qep_err(hw, "%s: CMAC reset failed\n", __func__);
			return -EINVAL;
		}

		*reset = *reset & ~ETH_RESET_MAC;
	}

	if (*reset & ETH_RESET_DMA) {
		void __iomem *io_addr = (void __iomem *)((u64)xpriv->bar_base +
				qep_fdt_get_netip_offset(xpriv->pinfo,
					QEP_NETPF_QAP_BASE));
		writel(1, (void __iomem *)((u64)io_addr
				+ QEP_BASE_QDMA_RESET_CONTROL_OFFSET));

		/* reset h2c and c2h stmn cores */
		writel(1, (void __iomem *)((u64)io_addr +
				QEP_BASE_DMA_USER_H2C_OFFSET));
		writel(0, (void __iomem *)((u64)io_addr +
				QEP_BASE_DMA_USER_H2C_OFFSET));

		writel(1, (void __iomem *)((u64)io_addr +
				QEP_BASE_DMA_USER_C2H_OFFSET));
		writel(0, (void __iomem *)((u64)io_addr +
				QEP_BASE_DMA_USER_C2H_OFFSET));

		msleep(100);

		*reset = *reset & ~ETH_RESET_DMA;
	}

	return 0;
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
	.get_sset_count = qep_get_sset_count,
	.get_ts_info = ethtool_op_get_ts_info,
	.get_link_ksettings = qep_get_link_ksettings,
	.set_link_ksettings = qep_set_link_ksettings,
	.nway_reset = qep_nway_reset,
	.reset = qep_reset
};

void qep_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &qep_ethtool_ops;
}
