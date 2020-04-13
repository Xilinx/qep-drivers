/*
 * Copyright(c) 2020 Xilinx, Inc. All rights reserved.
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

#include "qep_cmac.h"

#ifdef __KERNEL__
#define cmac_err pr_err
#include <linux/kernel.h>
#include <linux/slab.h>
#define PRIu64 "llu"
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#define cmac_err printf
#endif

#ifndef offsetof
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#endif

#define QEP_MAX_NUM_DIGIT_U64	(24)
#define CMAC_SIZEOF(m)    sizeof(((struct qep_cmac_stats *)0)->m)
#define CMAC_OFFSET(m)  offsetof(struct qep_cmac_stats, m)

/* CMAC stats and statistics registers */
static const struct qep_stats cmac_stats[] = {
	{ "rx_mac_bytes", CMAC_SIZEOF(rx_mac_bytes),
			CMAC_OFFSET(rx_mac_bytes) },
	{ "rx_good_bytes", CMAC_SIZEOF(rx_good_bytes),
			CMAC_OFFSET(rx_good_bytes) },
	{ "rx_mac_packets", CMAC_SIZEOF(rx_mac_packets),
			CMAC_OFFSET(rx_mac_packets) },
	{ "rx_good_pkts", CMAC_SIZEOF(rx_good_pkts),
			CMAC_OFFSET(rx_good_pkts) },
	{ "rx_unicast_pkts", CMAC_SIZEOF(rx_unicast_pkts),
			CMAC_OFFSET(rx_unicast_pkts) },
	{ "rx_multicast_pkts", CMAC_SIZEOF(rx_multicast_pkts),
			CMAC_OFFSET(rx_multicast_pkts) },
	{ "rx_broadcast_pkts", CMAC_SIZEOF(rx_broadcast_pkts),
			CMAC_OFFSET(rx_broadcast_pkts) },
	{ "rx_pause_pkts", CMAC_SIZEOF(rx_pause_pkts),
			CMAC_OFFSET(rx_pause_pkts) },
	{ "rx_vlan_tagged_pkts", CMAC_SIZEOF(rx_vlan_tagged_pkts),
			CMAC_OFFSET(rx_vlan_tagged_pkts) },
	{ "rx_priority_pause_pkts", CMAC_SIZEOF(rx_priority_pause_pkts),
			CMAC_OFFSET(rx_priority_pause_pkts) },
	{ "rx_bad_fcs_pkts", CMAC_SIZEOF(rx_bad_fcs_pkts),
			CMAC_OFFSET(rx_bad_fcs_pkts) },
	{ "rx_bad_code_pkts", CMAC_SIZEOF(rx_bad_code_pkts),
			CMAC_OFFSET(rx_bad_code_pkts) },
	{ "rx_too_long_pkts", CMAC_SIZEOF(rx_too_long_pkts),
			CMAC_OFFSET(rx_too_long_pkts) },
	{ "rx_oversized_pkts", CMAC_SIZEOF(rx_oversized_pkts),
			CMAC_OFFSET(rx_oversized_pkts) },
	{ "rx_jabber_pkts", CMAC_SIZEOF(rx_jabber_pkts),
			CMAC_OFFSET(rx_jabber_pkts) },
	{ "rx_undersized_pkts", CMAC_SIZEOF(rx_undersized_pkts),
			CMAC_OFFSET(rx_undersized_pkts) },
	{ "rx_frag_pkts", CMAC_SIZEOF(rx_frag_pkts),
			CMAC_OFFSET(rx_frag_pkts) },
	{ "rx_stomped_fcs_pkts", CMAC_SIZEOF(rx_stomped_fcs_pkts),
			CMAC_OFFSET(rx_stomped_fcs_pkts) },
	{ "rx_range_error_pkts", CMAC_SIZEOF(rx_range_error_pkts),
			CMAC_OFFSET(rx_range_error_pkts) },
	{ "rx_truncated_pkts", CMAC_SIZEOF(rx_truncated_pkts),
			CMAC_OFFSET(rx_truncated_pkts) },
	{ "rx_hist_less_64_pkts", CMAC_SIZEOF(rx_hist_less_64_pkts),
			CMAC_OFFSET(rx_hist_less_64_pkts) },
	{ "rx_hist_64_pkts", CMAC_SIZEOF(rx_hist_64_pkts),
			CMAC_OFFSET(rx_hist_64_pkts) },
	{ "rx_hist_65_127_pkts", CMAC_SIZEOF(rx_hist_65_127_pkts),
			CMAC_OFFSET(rx_hist_65_127_pkts) },
	{ "rx_hist_128_255_pkts", CMAC_SIZEOF(rx_hist_128_255_pkts),
			CMAC_OFFSET(rx_hist_128_255_pkts) },
	{ "rx_hist_256_511_pkts", CMAC_SIZEOF(rx_hist_256_511_pkts),
			CMAC_OFFSET(rx_hist_256_511_pkts) },
	{ "rx_hist_512_1023_pkts", CMAC_SIZEOF(rx_hist_512_1023_pkts),
			CMAC_OFFSET(rx_hist_512_1023_pkts) },
	{ "rx_hist_1024_1518_pkts", CMAC_SIZEOF(rx_hist_1024_1518_pkts),
			CMAC_OFFSET(rx_hist_1024_1518_pkts) },
	{ "rx_hist_1519_1522_pkts", CMAC_SIZEOF(rx_hist_1519_1522_pkts),
			CMAC_OFFSET(rx_hist_1519_1522_pkts) },
	{ "rx_hist_1523_1548_pkts", CMAC_SIZEOF(rx_hist_1523_1548_pkts),
			CMAC_OFFSET(rx_hist_1523_1548_pkts) },
	{ "rx_hist_1549_2047_pkts", CMAC_SIZEOF(rx_hist_1549_2047_pkts),
			CMAC_OFFSET(rx_hist_1549_2047_pkts) },
	{ "rx_hist_2048_4095_pkts", CMAC_SIZEOF(rx_hist_2048_4095_pkts),
			CMAC_OFFSET(rx_hist_2048_4095_pkts) },
	{ "rx_hist_4096_8191_pkts", CMAC_SIZEOF(rx_hist_4096_8191_pkts),
			CMAC_OFFSET(rx_hist_4096_8191_pkts) },
	{ "rx_hist_8192_9215_pkts", CMAC_SIZEOF(rx_hist_8192_9215_pkts),
			CMAC_OFFSET(rx_hist_8192_9215_pkts) },
	{ "rx_hist_greter_eq_9216_pkts",
			CMAC_SIZEOF(rx_hist_greter_eq_9216_pkts),
			CMAC_OFFSET(rx_hist_greter_eq_9216_pkts) },
	{ "tx_mac_bytes", CMAC_SIZEOF(tx_mac_bytes),
			CMAC_OFFSET(tx_mac_bytes) },
	{ "tx_good_bytes", CMAC_SIZEOF(tx_good_bytes),
			CMAC_OFFSET(tx_good_bytes) },
	{ "tx_mac_packets", CMAC_SIZEOF(tx_mac_packets),
			CMAC_OFFSET(tx_mac_packets) },
	{ "tx_good_pkts", CMAC_SIZEOF(tx_good_pkts),
			CMAC_OFFSET(tx_good_pkts) },
	{ "tx_unicast_pkts", CMAC_SIZEOF(tx_unicast_pkts),
			CMAC_OFFSET(tx_unicast_pkts) },
	{ "tx_multicast_pkts", CMAC_SIZEOF(tx_multicast_pkts),
			CMAC_OFFSET(tx_multicast_pkts) },
	{ "tx_broadcast_pkts", CMAC_SIZEOF(tx_broadcast_pkts),
			CMAC_OFFSET(tx_broadcast_pkts) },
	{ "tx_pause_pkts", CMAC_SIZEOF(tx_pause_pkts),
			CMAC_OFFSET(tx_pause_pkts) },
	{ "tx_vlan_tagged_pkts", CMAC_SIZEOF(tx_vlan_tagged_pkts),
			CMAC_OFFSET(tx_vlan_tagged_pkts) },
	{ "tx_priority_pause_pkts", CMAC_SIZEOF(tx_priority_pause_pkts),
			CMAC_OFFSET(tx_priority_pause_pkts) },
	{ "tx_bad_fcs_pkts", CMAC_SIZEOF(tx_bad_fcs_pkts),
			CMAC_OFFSET(tx_bad_fcs_pkts) },
	{ "tx_frame_error_pkts", CMAC_SIZEOF(tx_frame_error_pkts),
			CMAC_OFFSET(tx_frame_error_pkts) },
	{ "tx_hist_less_64_pkts", CMAC_SIZEOF(tx_hist_less_64_pkts),
			CMAC_OFFSET(tx_hist_less_64_pkts) },
	{ "tx_hist_64_pkts", CMAC_SIZEOF(tx_hist_64_pkts),
			CMAC_OFFSET(tx_hist_64_pkts) },
	{ "tx_hist_65_127_pkts", CMAC_SIZEOF(tx_hist_65_127_pkts),
			CMAC_OFFSET(tx_hist_65_127_pkts) },
	{ "tx_hist_128_255_pkts", CMAC_SIZEOF(tx_hist_128_255_pkts),
			CMAC_OFFSET(tx_hist_128_255_pkts) },
	{ "tx_hist_256_511_pkts", CMAC_SIZEOF(tx_hist_256_511_pkts),
			CMAC_OFFSET(tx_hist_256_511_pkts) },
	{ "tx_hist_512_1023_pkts", CMAC_SIZEOF(tx_hist_512_1023_pkts),
			CMAC_OFFSET(tx_hist_512_1023_pkts) },
	{ "tx_hist_1024_1518_pkts", CMAC_SIZEOF(tx_hist_1024_1518_pkts),
			CMAC_OFFSET(tx_hist_1024_1518_pkts) },
	{ "tx_hist_1519_1522_pkts", CMAC_SIZEOF(tx_hist_1519_1522_pkts),
			CMAC_OFFSET(tx_hist_1519_1522_pkts) },
	{ "tx_hist_1523_1548_pkts", CMAC_SIZEOF(tx_hist_1523_1548_pkts),
			CMAC_OFFSET(tx_hist_1523_1548_pkts) },
	{ "tx_hist_1549_2047_pkts", CMAC_SIZEOF(tx_hist_1549_2047_pkts),
			CMAC_OFFSET(tx_hist_1549_2047_pkts) },
	{ "tx_hist_2048_4095_pkts", CMAC_SIZEOF(tx_hist_2048_4095_pkts),
			CMAC_OFFSET(tx_hist_2048_4095_pkts) },
	{ "tx_hist_4096_8191_pkts", CMAC_SIZEOF(tx_hist_4096_8191_pkts),
			CMAC_OFFSET(tx_hist_4096_8191_pkts) },
	{ "tx_hist_8192_9215_pkts", CMAC_SIZEOF(tx_hist_8192_9215_pkts),
			CMAC_OFFSET(tx_hist_8192_9215_pkts) },
	{ "tx_hist_greter_eq_9216_pkts",
			CMAC_SIZEOF(tx_hist_greter_eq_9216_pkts),
			CMAC_OFFSET(tx_hist_greter_eq_9216_pkts) },
};

void *qep_calloc(size_t n, size_t size)
{
#ifdef __KERNEL__
	return kcalloc(n, size, GFP_KERNEL);
#else
	return calloc(n, size);
#endif
}

void qep_free(void *mem)
{
#ifdef __KERNEL__
	kfree(mem);
#else
	free(mem);
#endif
}

int qep_cmac_stats_add(struct qep_cmac_stats *to, struct qep_cmac_stats *new)
{
	if (!to || !new) {
		cmac_err("%s: Invalid input param", __func__);
		return -EINVAL;
	}

	to->rx_mac_bytes += new->rx_mac_bytes;
	to->rx_good_bytes += new->rx_good_bytes;
	to->rx_mac_packets += new->rx_mac_packets;
	to->rx_good_pkts += new->rx_good_pkts;
	to->rx_unicast_pkts += new->rx_unicast_pkts;
	to->rx_multicast_pkts += new->rx_multicast_pkts;
	to->rx_broadcast_pkts += new->rx_broadcast_pkts;
	to->rx_pause_pkts += new->rx_pause_pkts;
	to->rx_vlan_tagged_pkts += new->rx_vlan_tagged_pkts;
	to->rx_priority_pause_pkts += new->rx_priority_pause_pkts;
	to->rx_bad_fcs_pkts += new->rx_bad_fcs_pkts;
	to->rx_bad_code_pkts += new->rx_bad_code_pkts;
	to->rx_too_long_pkts += new->rx_too_long_pkts;
	to->rx_oversized_pkts += new->rx_oversized_pkts;
	to->rx_jabber_pkts += new->rx_jabber_pkts;
	to->rx_undersized_pkts += new->rx_undersized_pkts;
	to->rx_frag_pkts += new->rx_frag_pkts;
	to->rx_stomped_fcs_pkts += new->rx_stomped_fcs_pkts;
	to->rx_range_error_pkts += new->rx_range_error_pkts;
	to->rx_truncated_pkts += new->rx_truncated_pkts;
	to->rx_hist_less_64_pkts += new->rx_hist_less_64_pkts;
	to->rx_hist_64_pkts += new->rx_hist_64_pkts;
	to->rx_hist_65_127_pkts += new->rx_hist_65_127_pkts;
	to->rx_hist_128_255_pkts += new->rx_hist_128_255_pkts;
	to->rx_hist_256_511_pkts += new->rx_hist_256_511_pkts;
	to->rx_hist_512_1023_pkts += new->rx_hist_512_1023_pkts;
	to->rx_hist_1024_1518_pkts += new->rx_hist_1024_1518_pkts;
	to->rx_hist_1519_1522_pkts += new->rx_hist_1519_1522_pkts;
	to->rx_hist_1523_1548_pkts += new->rx_hist_1523_1548_pkts;
	to->rx_hist_1549_2047_pkts += new->rx_hist_1549_2047_pkts;
	to->rx_hist_2048_4095_pkts += new->rx_hist_2048_4095_pkts;
	to->rx_hist_4096_8191_pkts += new->rx_hist_4096_8191_pkts;
	to->rx_hist_8192_9215_pkts += new->rx_hist_8192_9215_pkts;
	to->rx_hist_greter_eq_9216_pkts += new->rx_hist_greter_eq_9216_pkts;
	to->tx_mac_bytes += new->tx_mac_bytes;
	to->tx_good_bytes += new->tx_good_bytes;
	to->tx_mac_packets += new->tx_mac_packets;
	to->tx_good_pkts += new->tx_good_pkts;
	to->tx_unicast_pkts += new->tx_unicast_pkts;
	to->tx_multicast_pkts += new->tx_multicast_pkts;
	to->tx_broadcast_pkts += new->tx_broadcast_pkts;
	to->tx_pause_pkts += new->tx_pause_pkts;
	to->tx_vlan_tagged_pkts += new->tx_vlan_tagged_pkts;
	to->tx_priority_pause_pkts += new->tx_priority_pause_pkts;
	to->tx_bad_fcs_pkts += new->tx_bad_fcs_pkts;
	to->tx_frame_error_pkts += new->tx_frame_error_pkts;
	to->tx_hist_less_64_pkts += new->tx_hist_less_64_pkts;
	to->tx_hist_64_pkts += new->tx_hist_64_pkts;
	to->tx_hist_65_127_pkts += new->tx_hist_65_127_pkts;
	to->tx_hist_128_255_pkts += new->tx_hist_128_255_pkts;
	to->tx_hist_256_511_pkts += new->tx_hist_256_511_pkts;
	to->tx_hist_512_1023_pkts += new->tx_hist_512_1023_pkts;
	to->tx_hist_1024_1518_pkts += new->tx_hist_1024_1518_pkts;
	to->tx_hist_1519_1522_pkts += new->tx_hist_1519_1522_pkts;
	to->tx_hist_1523_1548_pkts += new->tx_hist_1523_1548_pkts;
	to->tx_hist_1549_2047_pkts += new->tx_hist_1549_2047_pkts;
	to->tx_hist_2048_4095_pkts += new->tx_hist_2048_4095_pkts;
	to->tx_hist_4096_8191_pkts += new->tx_hist_4096_8191_pkts;
	to->tx_hist_8192_9215_pkts += new->tx_hist_8192_9215_pkts;
	to->tx_hist_greter_eq_9216_pkts += new->tx_hist_greter_eq_9216_pkts;

	return 0;
}

int qep_cmac_stats_get_size(void)
{
	return sizeof(cmac_stats)/sizeof(struct qep_stats);
}

int qep_cmac_stats_get_strings(char *buf, int max_buf_len, int incr)
{
	int i = 0;
	int req_buf;
	int stat_len = qep_cmac_stats_get_size();

	if (!buf) {
		cmac_err("%s: buf pointer null", __func__);
		return -EINVAL;
	}

	if (incr != QEP_CMAC_STATS_INCR_32B &&
		incr != QEP_CMAC_STATS_INCR_64B) {
		cmac_err("%s: Unknown increment for stats string", __func__);
		return -EINVAL;
	}

	req_buf = stat_len * incr;
	if (req_buf > max_buf_len) {
		cmac_err("%s: Max buffer len less then required", __func__);
		return -EINVAL;
	}

	for (i = 0; i < stat_len; i++) {
		memcpy(buf, cmac_stats[i].name, QEP_CMAC_STATS_NAME_MAX_LEN);
		buf += incr;
	}

	return 0;
}

int qep_cmac_stats_get_data(struct qep_cmac_stats *stats, uint64_t *data,
	int len)
{
	char *p = NULL;
	int count;
	int stats_len = qep_cmac_stats_get_size();

	if (!stats || !data) {
		cmac_err("%s: Invalid input param", __func__);
		return -EINVAL;
	}

	if (len <  stats_len) {
		cmac_err("%s: data buffer less then required", __func__);
		return -EINVAL;
	}

	for (count = 0; count < stats_len; count++) {
		p = (char *)stats + cmac_stats[count].offset;

		if (cmac_stats[count].size == sizeof(uint64_t))
			data[count] = *(uint64_t *)p;
		else
			data[count] = *(uint32_t *)p;
	}

	return 0;
}

int qep_cmac_stats_get_buf_len(void)
{
	int sz = qep_cmac_stats_get_size();

	return sz * (QEP_MAX_NUM_DIGIT_U64 + QEP_CMAC_STATS_NAME_MAX_LEN);
}

int qep_cmac_stats_snprintf(struct qep_cmac_stats *stats, char *msg,
	uint32_t len)
{
	char *stat_string;
	uint8_t *p;
	uint64_t *stat_data;
	int ret, i;
	int stats_len = qep_cmac_stats_get_size();

	if (!stats || !msg) {
		cmac_err("%s: Input args null", __func__);
		return -EINVAL;
	}

	stat_string = qep_calloc(stats_len, QEP_CMAC_STATS_NAME_MAX_LEN);
	if (!stat_string) {
		cmac_err("%s: OOM for stat string", __func__);
		return -ENOMEM;
	}

	stat_data = qep_calloc(stats_len, sizeof(uint64_t));
	if (!stat_data) {
		cmac_err("%s: OOM for stat data", __func__);
		qep_free(stat_string);
		return -ENOMEM;
	}

	ret = qep_cmac_stats_get_strings(stat_string,
			stats_len * QEP_CMAC_STATS_NAME_MAX_LEN,
			QEP_CMAC_STATS_INCR_32B);
	if (ret) {
		cmac_err("%s: get string failed", __func__);
		goto fn_exit;
	}

	ret = qep_cmac_stats_get_data(stats, stat_data, stats_len);
	if (ret) {
		cmac_err("%s: get data failed", __func__);
		goto fn_exit;
	}

	p = (uint8_t *)stat_string;
	for (i = 0; i < stats_len; i++) {
		snprintf(msg + strlen(msg), len - strlen(msg),
			 "%-32s = %" PRIu64 "\n", p, stat_data[i]);
		p += QEP_CMAC_STATS_NAME_MAX_LEN;
	}

fn_exit:
	qep_free(stat_string);
	qep_free(stat_data);

	return 0;
}

int qep_cmac_stats_read(struct xcmac *cmac, struct qep_cmac_stats *stats)
{
	int ret;
	struct xcmac_packet_count_statistics tx_hist, rx_hist;
	struct xcmac_malformed_statistics rx_malformed, tx_malformed;
	struct xcmac_packet_type_statistics rx_pkt_types, tx_pkt_types;

	if (!cmac || !stats) {
		cmac_err("%s: Invalid args", __func__);
		return -EINVAL;
	}

	memset(&tx_hist, 0, sizeof(tx_hist));
	memset(&rx_hist, 0, sizeof(rx_hist));
	memset(&tx_malformed, 0, sizeof(tx_malformed));
	memset(&rx_malformed, 0, sizeof(rx_malformed));
	memset(&tx_pkt_types, 0, sizeof(tx_pkt_types));
	memset(&rx_pkt_types, 0, sizeof(rx_pkt_types));


	/* Get Rx Packet count statistics */
	ret = xcmac_get_rx_packet_statistics(cmac, &rx_hist);
	if (ret != 0) {
		cmac_err("%s: Error in retrieving Rx packet count statistics\n",
			__func__);
		return ret;
	}

	/* Get Tx Packet count statistics */
	ret = xcmac_get_tx_packet_statistics(cmac, &tx_hist);
	if (ret != 0) {
		cmac_err("%s: Error in retrieving Tx packet count statistics\n",
			__func__);
		return ret;
	}

	ret = xcmac_get_tx_malformed_statistics(cmac,
						&tx_malformed);
	if (ret != 0) {
		cmac_err("%s: Error in retrieving Tx Malformed statistics\n",
			__func__);
		return ret;
	}

	ret = xcmac_get_rx_malformed_statistics(cmac,
						&rx_malformed);
	if (ret != 0) {
		cmac_err("%s: Error in retrieving Rx Malformed statistics\n",
			__func__);
		return ret;
	}

	ret = xcmac_get_rx_packet_type_statistics(cmac, &rx_pkt_types);
	if (ret != 0) {
		cmac_err("%s: Error in retrieving Rx Pkt type statistics\n",
			__func__);
		return ret;
	}

	ret = xcmac_get_tx_packet_type_statistics(cmac, &tx_pkt_types);
	if (ret != 0) {
		cmac_err("%s: Error in retrieving Rx Pkt type statistics\n",
			__func__);
		return ret;
	}

	stats->rx_mac_bytes = rx_hist.total_bytes;
	stats->rx_good_bytes = rx_hist.total_good_bytes;
	stats->rx_mac_packets = rx_hist.total_packets;
	stats->rx_good_pkts = rx_hist.total_good_packets;
	stats->rx_unicast_pkts = rx_pkt_types.packets_unicast;
	stats->rx_multicast_pkts = rx_pkt_types.packets_multicast;
	stats->rx_broadcast_pkts = rx_pkt_types.packets_broadcast;
	stats->rx_pause_pkts = rx_pkt_types.packets_pause;
	stats->rx_vlan_tagged_pkts = rx_pkt_types.packets_vlan;
	stats->rx_priority_pause_pkts = rx_pkt_types.packets_priority_pause;

	stats->rx_bad_fcs_pkts = rx_malformed.bad_fcs_count;
	stats->rx_bad_code_pkts =
		xcmac_get_rx_bad_64b66b_code_count(cmac);
	stats->rx_too_long_pkts = rx_malformed.toolong_count;
	stats->rx_oversized_pkts = rx_malformed.oversize_count;
	stats->rx_jabber_pkts = rx_malformed.jabber_count;
	stats->rx_undersized_pkts = rx_malformed.under_size_count;
	stats->rx_frag_pkts = rx_malformed.fragment_count;
	stats->rx_stomped_fcs_pkts = rx_malformed.stomped_fcs_count;

	stats->rx_range_error_pkts =
		xcmac_get_rx_range_error_count(cmac);
	stats->rx_truncated_pkts =
		xcmac_get_rx_truncated_count(cmac);
	stats->rx_hist_less_64_pkts = rx_hist.packets_small;
	stats->rx_hist_64_pkts = rx_hist.packets_0_to_64_bytes;
	stats->rx_hist_65_127_pkts = rx_hist.packets_65_to_127_bytes;
	stats->rx_hist_128_255_pkts = rx_hist.packets_128_to_255_bytes;
	stats->rx_hist_256_511_pkts = rx_hist.packets_256_to_511_bytes;
	stats->rx_hist_512_1023_pkts = rx_hist.packets_512_to_1023_bytes;
	stats->rx_hist_1024_1518_pkts = rx_hist.packets_1024_to_1518_bytes;
	stats->rx_hist_1519_1522_pkts = rx_hist.packets_1519_to_1522_bytes;
	stats->rx_hist_1523_1548_pkts = rx_hist.packets_1523_to_1548_bytes;
	stats->rx_hist_1549_2047_pkts = rx_hist.packets_1549_to_2047_bytes;
	stats->rx_hist_2048_4095_pkts = rx_hist.packets_2048_to_4095_bytes;
	stats->rx_hist_4096_8191_pkts = rx_hist.packets_4096_to_8191_bytes;
	stats->rx_hist_8192_9215_pkts = rx_hist.packets_8192_to_9215_bytes;
	stats->rx_hist_greter_eq_9216_pkts = rx_hist.packets_large;

	stats->tx_mac_bytes = tx_hist.total_bytes;
	stats->tx_good_bytes = tx_hist.total_good_bytes;
	stats->tx_mac_packets = tx_hist.total_packets;
	stats->tx_good_pkts = tx_hist.total_good_packets;
	stats->tx_unicast_pkts = tx_pkt_types.packets_unicast;
	stats->tx_multicast_pkts = tx_pkt_types.packets_multicast;
	stats->tx_broadcast_pkts = tx_pkt_types.packets_broadcast;
	stats->tx_pause_pkts = tx_pkt_types.packets_pause;
	stats->tx_vlan_tagged_pkts = tx_pkt_types.packets_vlan;
	stats->tx_priority_pause_pkts = tx_pkt_types.packets_priority_pause;
	stats->tx_bad_fcs_pkts = tx_malformed.bad_fcs_count;
	stats->tx_frame_error_pkts =
		xcmac_get_tx_frame_error_count(cmac);
	stats->tx_hist_less_64_pkts = tx_hist.packets_small;
	stats->tx_hist_64_pkts = tx_hist.packets_0_to_64_bytes;
	stats->tx_hist_65_127_pkts = tx_hist.packets_65_to_127_bytes;
	stats->tx_hist_128_255_pkts = tx_hist.packets_128_to_255_bytes;
	stats->tx_hist_256_511_pkts = tx_hist.packets_256_to_511_bytes;
	stats->tx_hist_512_1023_pkts = tx_hist.packets_512_to_1023_bytes;
	stats->tx_hist_1024_1518_pkts = tx_hist.packets_1024_to_1518_bytes;
	stats->tx_hist_1519_1522_pkts = tx_hist.packets_1519_to_1522_bytes;
	stats->tx_hist_1523_1548_pkts = tx_hist.packets_1523_to_1548_bytes;
	stats->tx_hist_1549_2047_pkts = tx_hist.packets_1549_to_2047_bytes;
	stats->tx_hist_2048_4095_pkts = tx_hist.packets_2048_to_4095_bytes;
	stats->tx_hist_4096_8191_pkts = tx_hist.packets_4096_to_8191_bytes;
	stats->tx_hist_8192_9215_pkts = tx_hist.packets_8192_to_9215_bytes;
	stats->tx_hist_greter_eq_9216_pkts = tx_hist.packets_large;

	return 0;

}

int qep_cmac_stats_get(struct xcmac *cmac, struct qep_cmac_stats *stats)
{
	int ret = 0;
	struct qep_cmac_stats new_stat;

	if (!cmac || !stats) {
		cmac_err("%s: Invalid args", __func__);
		return -EINVAL;
	}

	memset(&new_stat, 0, sizeof(struct qep_cmac_stats));

	/* Set the Tick register so that statistics are
	 * captured in readable registers
	 */
	ret = xcmac_latch_statistics(cmac);
	if (ret != 0) {
		cmac_err("%s: Error in setting Tick register\n",
				__func__);
		return ret;
	}

	ret = qep_cmac_stats_read(cmac, &new_stat);
	if (ret != 0) {
		cmac_err("%s: qep_cmac_stats_read failed\n",
				__func__);
		return ret;
	}

	ret = qep_cmac_stats_add(stats, &new_stat);
	if (ret != 0) {
		cmac_err("%s: qep_cmac_stats_add failed\n",
				__func__);
		return ret;
	}

	return 0;
}


int qep_cmac_status_get_buf_len(void)
{
	return QEP_CMAC_STATUS_BUF_LEN;
}

int qep_cmac_status_snprintf(struct xcmac *mac, char *buf, int len)
{
	int ret, i = 0;
	struct xcmac_rx_lane_am_status lane;
	struct xcmac_rx_fault_status fault;
	struct xcmac_rx_packet_status packet_status;
	struct xcmac_rx_vldemux_status vldemux_status;
	struct xcmac_tx_status tx_status;
	struct xcmac_bip7_config bip7_config;

	if (!mac || !buf) {
		cmac_err("%s: Invalid args", __func__);
		return -EINVAL;
	}

	memset(&lane, 0, sizeof(lane));
	memset(&fault, 0, sizeof(fault));
	memset(&packet_status, 0, sizeof(packet_status));
	memset(&vldemux_status, 0, sizeof(vldemux_status));
	memset(&tx_status, 0, sizeof(tx_status));
	memset(&bip7_config, 0, sizeof(bip7_config));

	/* Get PCS lane status */
	ret = xcmac_get_rx_lane_status(mac, &lane);
	if (ret != 0) {
		cmac_err("Error in retrieving Rx lane status\n");
		return ret;
	}

	/* Get Ethernet Packet status */
	ret = xcmac_get_rx_packet_status(mac, &packet_status);
	if (ret != 0) {
		cmac_err("Error in retrieving Rx packet ret\n");
		return ret;
	}

	/* Get Fault status */
	ret = xcmac_get_rx_fault_status(mac, &fault);
	if (ret != 0) {
		cmac_err("Error in retrieving Rx fault ret\n");
		return ret;
	}
	/* Get Tx status */
	ret = xcmac_get_tx_status(mac, &tx_status);
	if (ret != 0) {
		cmac_err("Error in retrieving Tx ret\n");
		return ret;
	}

	/* Get Virtual lane demux status */
	ret = xcmac_get_rx_vldemux_status(mac, &vldemux_status);
	if (ret != 0) {
		cmac_err("Error in retrieving Rx VlDemux ret\n");
		return ret;
	}

	ret = xcmac_get_bip7_config(mac, &bip7_config);
	if (ret != 0) {
		cmac_err("Error in retrieving Rx BIP7 Value\n");
		return ret;
	}

	snprintf(buf + strlen(buf), len - strlen(buf),
		 "PCS Status=0x%x Aligned=0x%x Misaligned=0x%x Aligned error=0x%x\n",
			lane.pcs_status, lane.aligned,
			lane.mis_aligned, lane.aligned_error);

	snprintf(buf + strlen(buf), len - strlen(buf),
		 "HiBer=0x%x Bad Preamble=0x%x Bad Sfd=0x%x Got Signal OS=0x%x\n",
			packet_status.hi_ber,
			packet_status.bad_preamble, packet_status.bad_sfd,
			packet_status.got_signal_os);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"Remote Fault=0x%x Local Fault=0x%x Internal Local Fault=0x%x Received Local Fault=0x%x\n",
		fault.remote_fault,
		fault.local_fault, fault.internal_local_fault,
		fault.received_local_fault);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"Local Fault=0x%x PTP FIFO Read Error=0x%x PTP FIFO Write Error=0x%x\n",
		tx_status.local_fault, tx_status.ptp_fifo_read_error,
		tx_status.ptp_fifo_write_error);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"Rx BIP Override Value=0x%x Valid=0x%x\n",
		bip7_config.bip7_value, bip7_config.bip7_valid);


	snprintf(buf + strlen(buf), len - strlen(buf), "Lane:         ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf), " %2d ", i);

	snprintf(buf + strlen(buf), len - strlen(buf), "\nBlock Lock    ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf), "0x%x ",
			lane.block_lock[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nSynced Err    ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			"0x%x ", lane.synced_error[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nLM Err        ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			"0x%x ", lane.lane_marker_error[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nLM len Err    ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			"0x%x ",
			lane.lane_marker_len_error[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nLM Repeat Err ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			"0x%x ",
			lane.lane_marker_repeat_error[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nVlDemuxed     ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf),
			len - strlen(buf), "0x%x ",
			vldemux_status.vldemuxed[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nVlNumber      ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			 "0x%x ", vldemux_status.vlnumber[i]);

	snprintf(buf + strlen(buf), len - strlen(buf), "\n");

	return ret;
}


