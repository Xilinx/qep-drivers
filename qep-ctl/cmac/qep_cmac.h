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

#ifndef QEP_CMAC_H
#define QEP_CMAC_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif
#include "xcmac.h"

#define QEP_CMAC_STATS_NAME_MAX_LEN	(32)
#define QEP_CMAC_STATUS_BUF_LEN		(1152)

#define QEP_CMAC_STATS_INCR_32B		(32)
#define QEP_CMAC_STATS_INCR_64B		(64)

/* Struct to define an element of statistics */
struct qep_stats {
	char name[QEP_CMAC_STATS_NAME_MAX_LEN];
	uint32_t size;
	uint32_t offset;
};
/* Hardware statistics structure */
struct qep_cmac_stats {
	uint64_t rx_mac_bytes;
	uint64_t rx_good_bytes;
	uint64_t rx_mac_packets;
	uint64_t rx_good_pkts;
	uint64_t rx_unicast_pkts;
	uint64_t rx_multicast_pkts;
	uint64_t rx_broadcast_pkts;
	uint64_t rx_pause_pkts;
	uint64_t rx_vlan_tagged_pkts;
	uint64_t rx_priority_pause_pkts;

	uint64_t rx_bad_fcs_pkts;
	uint64_t rx_bad_code_pkts;
	uint64_t rx_too_long_pkts;
	uint64_t rx_oversized_pkts;
	uint64_t rx_jabber_pkts;
	uint64_t rx_undersized_pkts;
	uint64_t rx_frag_pkts;
	uint64_t rx_stomped_fcs_pkts;

	uint64_t rx_range_error_pkts;
	uint64_t rx_truncated_pkts;
	uint64_t rx_hist_less_64_pkts;
	uint64_t rx_hist_64_pkts;
	uint64_t rx_hist_65_127_pkts;
	uint64_t rx_hist_128_255_pkts;
	uint64_t rx_hist_256_511_pkts;
	uint64_t rx_hist_512_1023_pkts;
	uint64_t rx_hist_1024_1518_pkts;
	uint64_t rx_hist_1519_1522_pkts;
	uint64_t rx_hist_1523_1548_pkts;
	uint64_t rx_hist_1549_2047_pkts;
	uint64_t rx_hist_2048_4095_pkts;
	uint64_t rx_hist_4096_8191_pkts;
	uint64_t rx_hist_8192_9215_pkts;
	uint64_t rx_hist_greter_eq_9216_pkts;

	uint64_t tx_mac_bytes;
	uint64_t tx_good_bytes;
	uint64_t tx_mac_packets;
	uint64_t tx_good_pkts;
	uint64_t tx_unicast_pkts;
	uint64_t tx_multicast_pkts;
	uint64_t tx_broadcast_pkts;
	uint64_t tx_pause_pkts;
	uint64_t tx_vlan_tagged_pkts;
	uint64_t tx_priority_pause_pkts;

	uint64_t tx_bad_fcs_pkts;
	uint64_t tx_frame_error_pkts;
	uint64_t tx_hist_less_64_pkts;
	uint64_t tx_hist_64_pkts;
	uint64_t tx_hist_65_127_pkts;
	uint64_t tx_hist_128_255_pkts;
	uint64_t tx_hist_256_511_pkts;
	uint64_t tx_hist_512_1023_pkts;
	uint64_t tx_hist_1024_1518_pkts;
	uint64_t tx_hist_1519_1522_pkts;
	uint64_t tx_hist_1523_1548_pkts;
	uint64_t tx_hist_1549_2047_pkts;
	uint64_t tx_hist_2048_4095_pkts;
	uint64_t tx_hist_4096_8191_pkts;
	uint64_t tx_hist_8192_9215_pkts;
	uint64_t tx_hist_greter_eq_9216_pkts;
};
/* Returns number of entry in private CMAC stats */
int qep_cmac_stats_get_size(void);
/* Populates the provided buf with name of stats string separated by @incr */
int qep_cmac_stats_get_strings(char *buf, int len, int incr);
/* Issues PM tick to takes a snapshot of statistics counter, reads stats reg
 * from HW and adds them to fields in @stats
 */
int qep_cmac_stats_get(struct xcmac *cmac, struct qep_cmac_stats *stats);
/* Return length of buffer needed for stats */
int qep_cmac_stats_get_buf_len(void);
/* Populates the name value pair into msg buffer, value is taken from @stats */
int qep_cmac_stats_snprintf(struct qep_cmac_stats *stats, char *msg,
	uint32_t len);
/* Populates the value from @stats into uint64_t buffer*/
int qep_cmac_stats_get_data(struct qep_cmac_stats *stats, uint64_t *data,
	int len);
/* Return length of buffer needed for status */
int qep_cmac_status_get_buf_len(void);
/* Populates the CMAC status into buf in human readable format */
int qep_cmac_status_snprintf(struct xcmac *mac, char *buf, int len);
/* Utility API to add stats struct*/
int qep_cmac_stats_add(struct qep_cmac_stats *to, struct qep_cmac_stats *new);
/* Reads the stats from HW, User should give clear latch statistics */
int qep_cmac_stats_read(struct xcmac *cmac, struct qep_cmac_stats *stats);

int qep_lbus_get_buf_len(void);
/* @addr is base address of lbus IP*/
int qep_lbus_snprintf(void *addr, char *buf, int len);

#endif
