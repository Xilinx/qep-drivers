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
#include "qdma_access.h"
#include "qdma_mbox_protocol.h"
#include "qdma_mbox.h"
#include "stmn.h"
#include "qdma_xcmac.h"
#include "qdma_reg_dump.h"
#include "qdma_platform.h"

struct rte_qep_xstats_name {
	char name[RTE_ETH_XSTATS_NAME_SIZE];
	unsigned int offset;
};

struct rte_qep_stmn_stats32 {
	struct stmn_fifo_fill_level fifo_fill;
	struct stmn_dsc_sts_min_max dsc_sts;
};

static const struct rte_qep_xstats_name rte_qep_stmn_stats_str32[] = {
	{"c2h_cmpt", offsetof(struct rte_qep_stmn_stats32,
					fifo_fill.c2h_cmpt)},
	{"qdma_c2h_sts", offsetof(struct rte_qep_stmn_stats32,
					fifo_fill.qdma_c2h_sts)},
	{"h2c_meta", offsetof(struct rte_qep_stmn_stats32,
					fifo_fill.h2c_meta)},
	{"c2h_dsc_sts", offsetof(struct rte_qep_stmn_stats32,
					fifo_fill.c2h_dsc_sts)},
	{"h2c_dsc_sts", offsetof(struct rte_qep_stmn_stats32,
					fifo_fill.h2c_dsc_sts)},
	{"c2h_crdt", offsetof(struct rte_qep_stmn_stats32,
					fifo_fill.c2h_crdt)},
	{"h2c_crdt", offsetof(struct rte_qep_stmn_stats32,
					fifo_fill.h2c_crdt)},
	{"c2h_byp_out", offsetof(struct rte_qep_stmn_stats32,
					fifo_fill.c2h_byp_out)},
	{"h2c_byp_out", offsetof(struct rte_qep_stmn_stats32,
					fifo_fill.h2c_byp_out)},
	{"dsc_sts_c2h_min_avl", offsetof(struct rte_qep_stmn_stats32,
					dsc_sts.c2h_min_avl)},
	{"dsc_sts_c2h_max_avl", offsetof(struct rte_qep_stmn_stats32,
					dsc_sts.c2h_max_avl)},
	{"dsc_sts_h2c_min_avl", offsetof(struct rte_qep_stmn_stats32,
					dsc_sts.h2c_min_avl)},
	{"dsc_sts_h2c_min_avl", offsetof(struct rte_qep_stmn_stats32,
					dsc_sts.h2c_max_avl)},
};

#define QEP_NB_STMN_XSTATS32 (sizeof(rte_qep_stmn_stats_str32) / \
		sizeof(rte_qep_stmn_stats_str32[0]))

static const struct rte_qep_xstats_name rte_qep_stmn_stats_str64[] = {
	{"cycle_cnt", offsetof(struct stmn_stats, cycle_cnt)},
	{"s_axis_pkt_in_cnt", offsetof(struct stmn_stats,
					s_axis.s_axis_pkt_in_cnt)},
	{"s_axis_pkt_accept_cnt", offsetof(struct stmn_stats,
					s_axis.s_axis_pkt_accept_cnt)},
	{"s_axis_byte_accept_cnt", offsetof(struct stmn_stats,
					s_axis.s_axis_byte_accept_cnt)},
	{"s_axis_pkt_drop_cnt", offsetof(struct stmn_stats,
					s_axis.s_axis_pkt_drop_cnt)},
	{"s_axis_byte_drop_cnt", offsetof(struct stmn_stats,
					s_axis.s_axis_byte_drop_cnt)},
	{"qdma_pkt_accept_cnt", offsetof(struct stmn_stats,
					s_axis.qdma_pkt_accept_cnt)},
	{"qdma_byte_accept_cnt", offsetof(struct stmn_stats,
					s_axis.qdma_byte_accept_cnt)},
	{"qdma_pkt_drop_cnt", offsetof(struct stmn_stats,
					s_axis.qdma_pkt_drop_cnt)},
	{"qdma_byte_drop_cnt", offsetof(struct stmn_stats,
					s_axis.qdma_byte_drop_cnt)},
	{"s_axis_active", offsetof(struct stmn_stats, s_axis.s_axis_active)},
	{"s_axis_idle", offsetof(struct stmn_stats, s_axis.s_axis_idle)},
	{"s_axis_pause", offsetof(struct stmn_stats, s_axis.s_axis_pause)},
	{"qdma_axis_c2h_active", offsetof(struct stmn_stats,
					s_axis.qdma_axis_c2h_active)},
	{"qdma_axis_c2h_idle", offsetof(struct stmn_stats,
					s_axis.qdma_axis_c2h_idle)},
	{"qdma_axis_c2h_pause", offsetof(struct stmn_stats,
					s_axis.qdma_axis_c2h_pause)},
	{"qdma_axis_c2h_cmpt_active", offsetof(struct stmn_stats,
					s_axis.qdma_axis_c2h_cmpt_active)},
	{"qdma_axis_c2h_cmpt_idle", offsetof(struct stmn_stats,
					s_axis.qdma_axis_c2h_cmpt_idle)},
	{"qdma_axis_c2h_cmpt_pause", offsetof(struct stmn_stats,
					s_axis.qdma_axis_c2h_cmpt_pause)},
	{"qdma_axis_c2h_dmawr_cmp_cnt", offsetof(struct stmn_stats,
					s_axis.qdma_axis_c2h_dmawr_cmp_cnt)},
	{"c2h_cmpt_fifo_avg_cnt", offsetof(struct stmn_stats,
					s_axis.c2h_cmpt_fifo_avg_cnt)},
	{"qdma_c2h_sts_fifo_avg_cnt", offsetof(struct stmn_stats,
					s_axis.qdma_c2h_sts_fifo_avg_cnt)},
	{"m_axis_pkt_cnt", offsetof(struct stmn_stats, m_axis.m_axis_pkt_cnt)},
	{"m_axis_byte_cnt", offsetof(struct stmn_stats,
					m_axis.m_axis_byte_cnt)},
	{"m_axis_active", offsetof(struct stmn_stats, m_axis.m_axis_active)},
	{"m_axis_idle", offsetof(struct stmn_stats, m_axis.m_axis_idle)},
	{"m_axis_pause", offsetof(struct stmn_stats, m_axis.m_axis_pause)},
	{"h2c_meta_fifo_avg_cnt", offsetof(struct stmn_stats,
					m_axis.h2c_meta_fifo_avg_cnt)},
	{"dsc_sts_c2h_cnt", offsetof(struct stmn_stats, desc.dsc_sts_c2h_cnt)},
	{"dsc_sts_h2c_cnt", offsetof(struct stmn_stats, desc.dsc_sts_h2c_cnt)},
	{"dsc_sts_c2h_avl_cnt", offsetof(struct stmn_stats,
					desc.dsc_sts_c2h_avl_cnt)},
	{"dsc_sts_h2c_avl_cnt", offsetof(struct stmn_stats,
					desc.dsc_sts_h2c_avl_cnt)},
	{"dsc_sts_c2h_fifo_avg_cnt", offsetof(struct stmn_stats,
					desc.dsc_sts_c2h_fifo_avg_cnt)},
	{"dsc_sts_h2c_fifo_avg_cnt", offsetof(struct stmn_stats,
					desc.dsc_sts_h2c_fifo_avg_cnt)},
	{"crdt_in_c2h_vld_cnt", offsetof(struct stmn_stats,
					crdt_bypout.crdt_in_c2h_vld_cnt)},
	{"crdt_in_h2c_vld_cnt", offsetof(struct stmn_stats,
					crdt_bypout.crdt_in_h2c_vld_cnt)},
	{"crdt_in_c2h_cnt", offsetof(struct stmn_stats,
					crdt_bypout.crdt_in_c2h_cnt)},
	{"crdt_in_h2c_cnt", offsetof(struct stmn_stats,
					crdt_bypout.crdt_in_h2c_cnt)},
	{"byp_out_c2h_cnt", offsetof(struct stmn_stats,
					crdt_bypout.byp_out_c2h_cnt)},
	{"byp_out_h2c_cnt", offsetof(struct stmn_stats,
					crdt_bypout.byp_out_h2c_cnt)},
	{"c2h_crdt_fifo_avg_cnt", offsetof(struct stmn_stats,
					crdt_bypout.c2h_crdt_fifo_avg_cnt)},
	{"h2c_crdt_fifo_avg_cnt", offsetof(struct stmn_stats,
					crdt_bypout.h2c_crdt_fifo_avg_cnt)},
	{"c2h_byp_out_fifo_avg_cnt", offsetof(struct stmn_stats,
					crdt_bypout.c2h_byp_out_fifo_avg_cnt)},
};

#define QEP_NB_STMN_XSTATS64 (sizeof(rte_qep_stmn_stats_str64) / \
		sizeof(rte_qep_stmn_stats_str64[0]))

/* CMAC Statistics */
struct rte_qep_cmac_stats {
	struct xcmac_packet_count_statistics rx_pkt_count;
	struct xcmac_packet_type_statistics rx_pkt_types;
	struct xcmac_malformed_statistics rx_malformed;
	uint64_t rx_bad_code_pkts;
	uint64_t rx_range_error_pkts;
	uint64_t rx_truncated_pkts;
	struct xcmac_packet_count_statistics tx_pkt_count;
	struct xcmac_packet_type_statistics tx_pkt_types;
	struct xcmac_malformed_statistics tx_malformed;
	uint64_t tx_frame_error_pkts;
};

static const struct rte_qep_xstats_name rte_qep_cmac_stats_str64[] = {
	{ "rx_mac_packets", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.total_packets)},
	{ "rx_good_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.total_good_packets)},
	{ "rx_mac_bytes", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.total_bytes)},
	{ "rx_good_bytes", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.total_good_bytes)},
	{ "rx_hist_less_64_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_small)},
	{ "rx_hist_64_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_0_to_64_bytes)},
	{ "rx_hist_65_127_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_65_to_127_bytes)},
	{ "rx_hist_128_255_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_128_to_255_bytes)},
	{ "rx_hist_256_511_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_256_to_511_bytes)},
	{ "rx_hist_512_1023_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_512_to_1023_bytes)},
	{ "rx_hist_1024_1518_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_1024_to_1518_bytes)},
	{ "rx_hist_1519_1522_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_1519_to_1522_bytes)},
	{ "rx_hist_1523_1548_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_1523_to_1548_bytes)},
	{ "rx_hist_1549_2047_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_1549_to_2047_bytes)},
	{ "rx_hist_2048_4095_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_2048_to_4095_bytes)},
	{ "rx_hist_4096_8191_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_4096_to_8191_bytes)},
	{ "rx_hist_8192_9215_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_8192_to_9215_bytes)},
	{ "rx_hist_greater_eq_9216_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_count.packets_large)},

	{ "rx_unicast_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_types.packets_unicast)},
	{ "rx_multicast_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_types.packets_multicast)},
	{ "rx_broadcast_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_types.packets_broadcast)},
	{ "rx_pause_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_types.packets_pause)},
	{ "rx_vlan_tagged_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_types.packets_vlan)},
	{ "rx_priority_pause_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_pkt_types.packets_priority_pause)},

	{ "rx_undersized_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_malformed.under_size_count)},
	{ "rx_frag_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_malformed.fragment_count)},
	{ "rx_oversized_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_malformed.oversize_count)},
	{ "rx_too_long_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_malformed.toolong_count)},
	{ "rx_jabber_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_malformed.jabber_count)},
	{ "rx_bad_fcs_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_malformed.bad_fcs_count)},
	{ "rx_bad_fcs_pkts_64b_to_max", offsetof(struct rte_qep_cmac_stats,
				rx_malformed.packet_bad_fcs_count)},
	{ "rx_stomped_fcs_pkts", offsetof(struct rte_qep_cmac_stats,
				rx_malformed.stomped_fcs_count)},

	{ "rx_bad_code_pkts", 0},
	{ "rx_range_error_pkts", 0},
	{ "rx_truncated_pkts", 0},

	{ "tx_mac_bytes", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.total_bytes)},
	{ "tx_good_bytes", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.total_good_bytes)},
	{ "tx_mac_packets", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.total_packets)},
	{ "tx_good_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.total_good_packets)},
	{ "tx_hist_less_64_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_small)},
	{ "tx_hist_64_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_0_to_64_bytes)},
	{ "tx_hist_65_127_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_65_to_127_bytes)},
	{ "tx_hist_128_255_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_128_to_255_bytes)},
	{ "tx_hist_256_511_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_256_to_511_bytes)},
	{ "tx_hist_512_1023_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_512_to_1023_bytes)},
	{ "tx_hist_1024_1518_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_1024_to_1518_bytes)},
	{ "tx_hist_1519_1522_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_1519_to_1522_bytes)},
	{ "tx_hist_1523_1548_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_1523_to_1548_bytes)},
	{ "tx_hist_1549_2047_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_1549_to_2047_bytes)},
	{ "tx_hist_2048_4095_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_2048_to_4095_bytes)},
	{ "tx_hist_4096_8191_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_4096_to_8191_bytes)},
	{ "tx_hist_8192_9215_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_8192_to_9215_bytes)},
	{ "tx_hist_greater_eq_9216_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_count.packets_large)},

	{ "tx_unicast_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_types.packets_unicast)},
	{ "tx_multicast_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_types.packets_multicast)},
	{ "tx_broadcast_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_types.packets_broadcast)},
	{ "tx_pause_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_types.packets_pause)},
	{ "tx_vlan_tagged_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_types.packets_vlan)},
	{ "tx_priority_pause_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_pkt_types.packets_priority_pause)},

	{ "tx_bad_fcs_pkts", offsetof(struct rte_qep_cmac_stats,
				tx_malformed.bad_fcs_count)},
	{ "tx_frame_error_pkts", 0},
};

#define QEP_NB_CMAC_XSTATS64 (sizeof(rte_qep_cmac_stats_str64) / \
		sizeof(rte_qep_cmac_stats_str64[0]))

#define QEP_NB_XSTATS (QEP_NB_CMAC_XSTATS64 + \
			QEP_NB_STMN_XSTATS32 +	QEP_NB_STMN_XSTATS64)

void qep_init_reta(struct rte_eth_dev *dev, uint32_t offset)
{
	struct qdma_pci_dev *dma_priv;
	uint32_t qval;
	uint32_t i, addr;
	uint32_t nrxq;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;
	nrxq = dev->data->nb_rx_queues;
	for (i = 0; i < QEP_RSS_TBL_SIZE; i++) {
		qval = offset + (i % nrxq);
		addr = (QEP_BASE_OFFSET + QEP_USER_RSS_OFFSET + (i * 4));
		qdma_write_reg((uint64_t)
			       (dma_priv->bar_addr[dma_priv->user_bar_idx]) +
			       addr, qval);
	}

	qep_set_rx_mode(dev, QEP_RX_MODE_TCAM);
}

void  qep_set_reta(struct rte_eth_dev *dev, uint32_t *rss_tbl)
{
	struct qdma_pci_dev *dma_priv;
	uint32_t qval;
	uint32_t i, addr;
	uint32_t offset = 0;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	for (i = 0; i < QEP_RSS_TBL_SIZE; i++) {
		qval = rss_tbl[i] + offset;
		addr = (QEP_BASE_OFFSET + QEP_USER_RSS_OFFSET + (i * 4));
		qdma_write_reg((uint64_t)
			       (dma_priv->bar_addr[dma_priv->user_bar_idx]) +
			       addr, qval);
	}
	qep_set_rx_mode(dev, QEP_RX_MODE_TCAM);

}

int qdma_vf_csr_read(struct rte_eth_dev *dev)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_mbox_msg *m = qdma_mbox_msg_alloc();
	int rv, i;
	struct qdma_csr_info csr_info;

	if (!m)
		return -ENOMEM;

	qdma_mbox_compose_csr_read(qdma_dev->func_id, m->raw_data);
	rv = qdma_mbox_msg_send(dev, m, MBOX_OP_RSP_TIMEOUT);
	if (rv < 0)
		goto free_msg;

	rv = qdma_mbox_vf_csr_get(m->raw_data, &csr_info);
	if (rv < 0)
		goto free_msg;
	for (i = 0; i < QDMA_NUM_RING_SIZES; i++) {
		qdma_dev->g_ring_sz[i] = (uint32_t)csr_info.ringsz[i];
		qdma_dev->g_c2h_buf_sz[i] = (uint32_t)csr_info.bufsz[i];
		qdma_dev->g_c2h_timer_cnt[i] = (uint32_t)csr_info.timer_cnt[i];
		qdma_dev->g_c2h_cnt_th[i] = (uint32_t)csr_info.cnt_thres[i];
	}


free_msg:
	qdma_mbox_msg_free(m);
	return rv;
}

int qdma_pf_csr_read(struct rte_eth_dev *dev)
{
	int ret = 0;
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_hw_access *hw_access = qdma_dev->hw_access;

	ret = hw_access->qdma_global_csr_conf(dev, 0,
				QDMA_NUM_RING_SIZES, qdma_dev->g_ring_sz,
		QDMA_CSR_RING_SZ, QDMA_HW_ACCESS_READ);
	if (ret != QDMA_SUCCESS)
		PMD_DRV_LOG(ERR, "qdma_global_csr_conf for ring size "
				  "returned %d", ret);
	if (qdma_dev->dev_cap.st_en || qdma_dev->dev_cap.mm_cmpt_en) {
		ret = hw_access->qdma_global_csr_conf(dev, 0,
				QDMA_NUM_C2H_TIMERS, qdma_dev->g_c2h_timer_cnt,
		QDMA_CSR_TIMER_CNT, QDMA_HW_ACCESS_READ);
		if (ret != QDMA_SUCCESS)
			PMD_DRV_LOG(ERR, "qdma_global_csr_conf for timer count "
					  "returned %d", ret);

		ret = hw_access->qdma_global_csr_conf(dev, 0,
				QDMA_NUM_C2H_COUNTERS, qdma_dev->g_c2h_cnt_th,
		QDMA_CSR_CNT_TH, QDMA_HW_ACCESS_READ);
		if (ret != QDMA_SUCCESS)
			PMD_DRV_LOG(ERR, "qdma_global_csr_conf for counter threshold "
					  "returned %d", ret);

	}

	if (qdma_dev->dev_cap.st_en) {
		ret = hw_access->qdma_global_csr_conf(dev, 0,
				QDMA_NUM_C2H_BUFFER_SIZES,
				qdma_dev->g_c2h_buf_sz,
				QDMA_CSR_BUF_SZ,
				QDMA_HW_ACCESS_READ);
		if (ret != QDMA_SUCCESS)
			PMD_DRV_LOG(ERR, "qdma_global_csr_conf for buffer sizes "
					  "returned %d", ret);
	}

	return qdma_dev->hw_access->qdma_get_error_code(ret);
}

static int qdma_pf_fmap_prog(struct rte_eth_dev *dev)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_fmap_cfg fmap_cfg;
	int ret = 0;

	memset(&fmap_cfg, 0, sizeof(struct qdma_fmap_cfg));

	/** FMAP configuration **/
	fmap_cfg.qbase = qdma_dev->queue_base;
	fmap_cfg.qmax = qdma_dev->qsets_en;
	ret = qdma_dev->hw_access->qdma_fmap_conf(dev,
			qdma_dev->func_id, &fmap_cfg, QDMA_HW_ACCESS_WRITE);
	if (ret != QDMA_SUCCESS)
		return qdma_dev->hw_access->qdma_get_error_code(ret);

	return ret;
}

int qdma_dev_notify_qadd(struct rte_eth_dev *dev, uint32_t qidx_hw,
						enum qdma_dev_q_type q_type)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_mbox_msg *m;
	int rv = 0;

	m = qdma_mbox_msg_alloc();
	if (!m)
		return -ENOMEM;

	qdma_mbox_compose_vf_notify_qadd(qdma_dev->func_id, qidx_hw,
					q_type, m->raw_data);
	rv = qdma_mbox_msg_send(dev, m, MBOX_OP_RSP_TIMEOUT);

	qdma_mbox_msg_free(m);
	return rv;
}

int qdma_dev_notify_qdel(struct rte_eth_dev *dev, uint32_t qidx_hw,
						enum qdma_dev_q_type q_type)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_mbox_msg *m;
	int rv = 0;

	m = qdma_mbox_msg_alloc();
	if (!m)
		return -ENOMEM;

	qdma_mbox_compose_vf_notify_qdel(qdma_dev->func_id, qidx_hw,
					q_type, m->raw_data);
	rv = qdma_mbox_msg_send(dev, m, MBOX_OP_RSP_TIMEOUT);

	qdma_mbox_msg_free(m);
	return rv;
}

uint8_t qmda_get_desc_sz_idx(enum rte_pmd_qdma_bypass_desc_len size)
{
	uint8_t ret;
	switch (size) {
	case RTE_PMD_QDMA_BYPASS_DESC_LEN_8B:
		ret = 0;
		break;
	case RTE_PMD_QDMA_BYPASS_DESC_LEN_16B:
		ret = 1;
		break;
	case RTE_PMD_QDMA_BYPASS_DESC_LEN_32B:
		ret = 2;
		break;
	case RTE_PMD_QDMA_BYPASS_DESC_LEN_64B:
		ret = 3;
		break;
	default:
		/* Suppress compiler warnings*/
		ret = 0;
	}
	return ret;
}

/**
 * DPDK callback to configure a RX queue.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param rx_queue_id
 *   RX queue index.
 * @param nb_rx_desc
 *   Number of descriptors to configure in queue.
 * @param socket_id
 *   NUMA socket on which memory must be allocated.
 * @param[in] rx_conf
 *   Thresholds parameters.
 * @param mp_pool
 *   Memory pool for buffer allocations.
 *
 * @return
 *   0 on success,
 *   -ENOMEM when memory allocation fails
 *   -ENOTSUP when HW doesn't support the required configuration
 *   -EINVAL on other failure.
 */
int qdma_dev_rx_queue_setup(struct rte_eth_dev *dev, uint16_t rx_queue_id,
				uint16_t nb_rx_desc, unsigned int socket_id,
				const struct rte_eth_rxconf *rx_conf,
				struct rte_mempool *mb_pool)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_rx_queue *rxq = NULL;
	struct qdma_ul_mm_desc *rx_ring_mm;
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	uint32_t sz;
	uint8_t  *rx_ring_bypass;
	int err = 0;

	PMD_DRV_LOG(INFO, "Configuring Rx queue id:%d\n", rx_queue_id);

	if (nb_rx_desc == 0) {
		PMD_DRV_LOG(ERR, "Invalid descriptor ring size %d\n",
				nb_rx_desc);
		return -EINVAL;
	}

	if (!qdma_dev->dev_configured) {
		PMD_DRV_LOG(ERR,
			"Device for Rx queue id %d is not configured yet\n",
			rx_queue_id);
		return -EINVAL;
	}

	if (!qdma_dev->is_vf) {
		err = qdma_dev_increment_active_queue(pci_dev->addr.bus,
						qdma_dev->func_id,
						QDMA_DEV_Q_TYPE_C2H);
		if (err != QDMA_SUCCESS)
			return -EINVAL;

		if (qdma_dev->q_info[rx_queue_id].queue_mode ==
				RTE_PMD_QDMA_STREAMING_MODE) {
			err = qdma_dev_increment_active_queue(
							pci_dev->addr.bus,
							qdma_dev->func_id,
							QDMA_DEV_Q_TYPE_CMPT);
			if (err != QDMA_SUCCESS) {
				qdma_dev_decrement_active_queue(
							pci_dev->addr.bus,
							qdma_dev->func_id,
							QDMA_DEV_Q_TYPE_C2H);
				return -EINVAL;
			}
		}
	} else {
		err = qdma_dev_notify_qadd(dev, rx_queue_id +
						qdma_dev->queue_base,
						QDMA_DEV_Q_TYPE_C2H);
		if (err < 0)
			return -EINVAL;

		if (qdma_dev->q_info[rx_queue_id].queue_mode ==
				RTE_PMD_QDMA_STREAMING_MODE) {
			err = qdma_dev_notify_qadd(dev, rx_queue_id +
							qdma_dev->queue_base,
							QDMA_DEV_Q_TYPE_CMPT);
			if (err < 0) {
				qdma_dev_notify_qdel(dev, rx_queue_id +
							qdma_dev->queue_base,
							QDMA_DEV_Q_TYPE_C2H);
				return -EINVAL;
			}
		}
	}
	if (!qdma_dev->init_q_range) {
		if (qdma_dev->is_vf) {
			err = qdma_vf_csr_read(dev);
			if (err < 0)
				goto rx_setup_err;
		} else {
			err = qdma_pf_csr_read(dev);
			if (err < 0)
				goto rx_setup_err;
		}
		qdma_dev->init_q_range = 1;
	}

	/* allocate rx queue data structure */
	rxq = rte_zmalloc("QDMA_RxQ", sizeof(struct qdma_rx_queue),
						RTE_CACHE_LINE_SIZE);
	if (!rxq) {
		PMD_DRV_LOG(ERR, "Unable to allocate structure rxq of "
				"size %d\n",
				(int)(sizeof(struct qdma_rx_queue)));
		err = -ENOMEM;
		goto rx_setup_err;
	}

	rxq->queue_id = rx_queue_id;
	rxq->port_id = dev->data->port_id;
	rxq->func_id = qdma_dev->func_id;
	rxq->mb_pool = mb_pool;
	rxq->dev = dev;
	rxq->st_mode = qdma_dev->q_info[rx_queue_id].queue_mode;
	rxq->nb_rx_desc = (nb_rx_desc + 1);
	/* <= 2018.2 IP
	 * double the cmpl ring size to avoid run out of cmpl entry while
	 * desc. ring still have free entries
	 */
	rxq->nb_rx_cmpt_desc = ((nb_rx_desc * 2) + 1);
	rxq->en_prefetch = qdma_dev->q_info[rx_queue_id].en_prefetch;
	rxq->cmpt_desc_len = qdma_dev->q_info[rx_queue_id].cmpt_desc_sz;
	if ((rxq->cmpt_desc_len == RTE_PMD_QDMA_CMPT_DESC_LEN_64B) &&
		!qdma_dev->dev_cap.cmpt_desc_64b) {
		PMD_DRV_LOG(ERR, "PF-%d(DEVFN) 64B completion entry size is "
			"not supported in this design\n", qdma_dev->func_id);
		return -ENOTSUP;
	}
	rxq->triggermode = qdma_dev->q_info[rx_queue_id].trigger_mode;
	rxq->rx_deferred_start = rx_conf->rx_deferred_start;
	rxq->dump_immediate_data =
			qdma_dev->q_info[rx_queue_id].immediate_data_state;
	rxq->dis_overflow_check =
			qdma_dev->q_info[rx_queue_id].dis_cmpt_ovf_chk;
	rxq->offloads = rx_conf->offloads | dev->data->dev_conf.rxmode.offloads;

	if (qdma_dev->q_info[rx_queue_id].rx_bypass_mode ==
				RTE_PMD_QDMA_RX_BYPASS_CACHE ||
			qdma_dev->q_info[rx_queue_id].rx_bypass_mode ==
			 RTE_PMD_QDMA_RX_BYPASS_SIMPLE)
		rxq->en_bypass = 1;
	if (qdma_dev->q_info[rx_queue_id].rx_bypass_mode ==
			RTE_PMD_QDMA_RX_BYPASS_SIMPLE)
		rxq->en_bypass_prefetch = 1;

	if (rxq->en_bypass) {
		rxq->bypass_desc_sz =
				qdma_dev->q_info[rx_queue_id].rx_bypass_desc_sz;
		if ((rxq->bypass_desc_sz == RTE_PMD_QDMA_BYPASS_DESC_LEN_64B)
			&&	!qdma_dev->dev_cap.sw_desc_64b) {
			PMD_DRV_LOG(ERR, "PF-%d(DEVFN) C2H bypass descriptor "
				"size of 64B is not supported in this design:\n",
				qdma_dev->func_id);
			return -ENOTSUP;
		}
	}
	/* Calculate the ring index, completion queue ring size,
	 * buffer index and threshold index.
	 * If index is not found , by default use the index as 0
	 */

	/* Find C2H queue ring size index */
	rxq->ringszidx = index_of_array(qdma_dev->g_ring_sz,
					QDMA_NUM_RING_SIZES, rxq->nb_rx_desc);
	if (rxq->ringszidx < 0) {
		PMD_DRV_LOG(ERR, "Expected Ring size %d not found\n",
				rxq->nb_rx_desc);
		err = -EINVAL;
		goto rx_setup_err;
	}

	/* Find completion ring size index */
	rxq->cmpt_ringszidx = index_of_array(qdma_dev->g_ring_sz,
						QDMA_NUM_RING_SIZES,
						rxq->nb_rx_cmpt_desc);
	if (rxq->cmpt_ringszidx < 0) {
		PMD_DRV_LOG(ERR, "Expected completion ring size %d not found\n",
				rxq->nb_rx_cmpt_desc);
		err = -EINVAL;
		goto rx_setup_err;
	}

	/* Find Threshold index */
	rxq->threshidx = index_of_array(qdma_dev->g_c2h_cnt_th,
					QDMA_NUM_C2H_COUNTERS,
					rx_conf->rx_thresh.wthresh);
	if (rxq->threshidx < 0) {
		PMD_DRV_LOG(WARNING, "Expected Threshold %d not found,"
				" using the value %d at index 0\n",
				rx_conf->rx_thresh.wthresh,
				qdma_dev->g_c2h_cnt_th[0]);
		rxq->threshidx = 0;
	}

	/* Find Timer index */
	rxq->timeridx = index_of_array(qdma_dev->g_c2h_timer_cnt,
				QDMA_NUM_C2H_TIMERS,
				qdma_dev->q_info[rx_queue_id].timer_count);
	if (rxq->timeridx < 0) {
		PMD_DRV_LOG(WARNING, "Expected timer %d not found, "
				"using the value %d at index 1\n",
				qdma_dev->q_info[rx_queue_id].timer_count,
				qdma_dev->g_c2h_timer_cnt[1]);
		rxq->timeridx = 1;
	}

	rxq->rx_buff_size = (uint16_t)
				(rte_pktmbuf_data_room_size(rxq->mb_pool) -
				RTE_PKTMBUF_HEADROOM);
	/* Allocate memory for Rx descriptor ring */
	if (rxq->st_mode) {
		if (!qdma_dev->dev_cap.st_en) {
			PMD_DRV_LOG(ERR, "Streaming mode not enabled "
					"in the hardware\n");
			err = -EINVAL;
			goto rx_setup_err;
		}
		/* Find Buffer size index */
		rxq->buffszidx = index_of_array(qdma_dev->g_c2h_buf_sz,
						QDMA_NUM_C2H_BUFFER_SIZES,
						rxq->rx_buff_size);
		if (rxq->buffszidx < 0) {
			PMD_DRV_LOG(ERR, "Expected buffer size %d not found\n",
					rxq->rx_buff_size);
			err = -EINVAL;
			goto rx_setup_err;
		}

		if (qdma_dev->stmn_c2h_buf_size == -1) {
			err = stmn_write_c2h_buf_size(qdma_dev,
						      rxq->rx_buff_size);
			if (err) {
				err = -EINVAL;
				goto rx_setup_err;
			}
			qdma_dev->stmn_c2h_buf_size = rxq->rx_buff_size;
		} else if (qdma_dev->stmn_c2h_buf_size != rxq->rx_buff_size) {
			PMD_DRV_LOG(ERR, "stmn does not support multiple rx "
				    "buffer size %d buffer already set to %d",
				    rxq->rx_buff_size,
				    qdma_dev->stmn_c2h_buf_size);
		}

		if (rxq->en_bypass &&
		     (rxq->bypass_desc_sz != 0))
			sz = (rxq->nb_rx_desc) * (rxq->bypass_desc_sz);
		else
			sz = (rxq->nb_rx_desc) *
					sizeof(struct qdma_ul_st_c2h_desc);

		rxq->rx_mz = qdma_zone_reserve(dev, "RxHwRn", rx_queue_id,
						sz, socket_id);
		if (!rxq->rx_mz) {
			PMD_DRV_LOG(ERR, "Unable to allocate rxq->rx_mz "
					"of size %d\n", sz);
			err = -ENOMEM;
			goto rx_setup_err;
		}
		rxq->rx_ring = rxq->rx_mz->addr;
		memset(rxq->rx_ring, 0, sz);

		/* Allocate memory for Rx completion(CMPT) descriptor ring */
		sz = (rxq->nb_rx_cmpt_desc) * rxq->cmpt_desc_len;
		rxq->rx_cmpt_mz = qdma_zone_reserve(dev, "RxHwCmptRn",
						    rx_queue_id, sz, socket_id);
		if (!rxq->rx_cmpt_mz) {
			PMD_DRV_LOG(ERR, "Unable to allocate rxq->rx_cmpt_mz "
					"of size %d\n", sz);
			err = -ENOMEM;
			goto rx_setup_err;
		}
		rxq->cmpt_ring =
			(struct qdma_ul_st_cmpt_ring *)rxq->rx_cmpt_mz->addr;

		/* Write-back status structure */
		rxq->wb_status = (struct wb_status *)((uint64_t)rxq->cmpt_ring +
				 (((uint64_t)rxq->nb_rx_cmpt_desc - 1) *
				  rxq->cmpt_desc_len));
		memset(rxq->cmpt_ring, 0, sz);
	} else {
		if (!qdma_dev->dev_cap.mm_en) {
			PMD_DRV_LOG(ERR, "Memory mapped mode not enabled "
					"in the hardware\n");
			err = -EINVAL;
			goto rx_setup_err;
		}

		if (rxq->en_bypass &&
			(rxq->bypass_desc_sz != 0))
			sz = (rxq->nb_rx_desc) * (rxq->bypass_desc_sz);
		else
			sz = (rxq->nb_rx_desc) * sizeof(struct qdma_ul_mm_desc);
		rxq->rx_mz = qdma_zone_reserve(dev, "RxHwRn",
						rx_queue_id, sz, socket_id);
		if (!rxq->rx_mz) {
			PMD_DRV_LOG(ERR, "Unable to allocate rxq->rx_mz "
					"of size %d\n", sz);
			err = -ENOMEM;
			goto rx_setup_err;
		}
		rxq->rx_ring = rxq->rx_mz->addr;
		rx_ring_mm = (struct qdma_ul_mm_desc *)rxq->rx_mz->addr;
		memset(rxq->rx_ring, 0, sz);

		rx_ring_bypass = (uint8_t *)rxq->rx_mz->addr;
		if (rxq->en_bypass &&
			(rxq->bypass_desc_sz != 0))
			rxq->wb_status = (struct wb_status *)&
					(rx_ring_bypass[(rxq->nb_rx_desc - 1) *
							(rxq->bypass_desc_sz)]);
		else
			rxq->wb_status = (struct wb_status *)&
					 (rx_ring_mm[rxq->nb_rx_desc - 1]);
	}

	/* allocate memory for RX software ring */
	sz = (rxq->nb_rx_desc) * sizeof(struct rte_mbuf *);
	rxq->sw_ring = rte_zmalloc("RxSwRn", sz, RTE_CACHE_LINE_SIZE);
	if (!rxq->sw_ring) {
		PMD_DRV_LOG(ERR, "Unable to allocate rxq->sw_ring of size %d\n",
									sz);
		err = -ENOMEM;
		goto rx_setup_err;
	}

	/* store rx_pkt_burst function pointer */
	dev->rx_pkt_burst = qdma_recv_pkts;
	dev->data->rx_queues[rx_queue_id] = rxq;

	return 0;

rx_setup_err:
	if (!qdma_dev->is_vf) {
		qdma_dev_decrement_active_queue(pci_dev->addr.bus,
						qdma_dev->func_id,
						QDMA_DEV_Q_TYPE_C2H);

		if (qdma_dev->q_info[rx_queue_id].queue_mode ==
				RTE_PMD_QDMA_STREAMING_MODE)
			qdma_dev_decrement_active_queue(pci_dev->addr.bus,
						qdma_dev->func_id,
						QDMA_DEV_Q_TYPE_CMPT);
	} else {
		qdma_dev_notify_qdel(dev, rx_queue_id +
				qdma_dev->queue_base, QDMA_DEV_Q_TYPE_C2H);

		if (qdma_dev->q_info[rx_queue_id].queue_mode ==
				RTE_PMD_QDMA_STREAMING_MODE)
			qdma_dev_notify_qdel(dev, rx_queue_id +
				qdma_dev->queue_base, QDMA_DEV_Q_TYPE_CMPT);
	}

	if (rxq) {
		if (rxq->rx_mz)
			rte_memzone_free(rxq->rx_mz);
		if (rxq->sw_ring)
			rte_free(rxq->sw_ring);
		rte_free(rxq);
	}
	return err;
}

/**
 * DPDK callback to configure a TX queue.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param tx_queue_id
 *   TX queue index.
 * @param nb_tx_desc
 *   Number of descriptors to configure in queue.
 * @param socket_id
 *   NUMA socket on which memory must be allocated.
 * @param[in] tx_conf
 *   Thresholds parameters.
 *
 * @return
 *   0 on success
 *   -ENOMEM when memory allocation fails
 *   -EINVAL on other failure.
 */
int qdma_dev_tx_queue_setup(struct rte_eth_dev *dev, uint16_t tx_queue_id,
			    uint16_t nb_tx_desc, unsigned int socket_id,
			    const struct rte_eth_txconf *tx_conf)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_tx_queue *txq = NULL;
	struct qdma_ul_mm_desc *tx_ring_mm;
	struct qdma_ul_st_h2c_desc *tx_ring_st;
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	uint32_t sz;
	uint8_t  *tx_ring_bypass;
	int err = 0;

	PMD_DRV_LOG(INFO, "Configuring Tx queue id:%d with %d desc\n",
		    tx_queue_id, nb_tx_desc);

	if (!qdma_dev->dev_configured) {
		PMD_DRV_LOG(ERR,
			"Device for Tx queue id %d is not configured yet\n",
			tx_queue_id);
		return -EINVAL;
	}

	if (!qdma_dev->is_vf) {
		err = qdma_dev_increment_active_queue(pci_dev->addr.bus,
						qdma_dev->func_id,
						QDMA_DEV_Q_TYPE_H2C);
		if (err != QDMA_SUCCESS)
			return -EINVAL;
	} else {
		err = qdma_dev_notify_qadd(dev, tx_queue_id +
						qdma_dev->queue_base,
						QDMA_DEV_Q_TYPE_H2C);
		if (err < 0)
			return -EINVAL;
	}
	if (!qdma_dev->init_q_range) {
		if (qdma_dev->is_vf) {
			err = qdma_vf_csr_read(dev);
			if (err < 0) {
				PMD_DRV_LOG(ERR, "CSR read failed\n");
				goto tx_setup_err;
			}
		} else {
			err = qdma_pf_csr_read(dev);
			if (err < 0) {
				PMD_DRV_LOG(ERR, "CSR read failed\n");
				goto tx_setup_err;
			}
		}
		qdma_dev->init_q_range = 1;
	}
	/* allocate rx queue data structure */
	txq = rte_zmalloc("QDMA_TxQ", sizeof(struct qdma_tx_queue),
						RTE_CACHE_LINE_SIZE);
	if (txq == NULL) {
		PMD_DRV_LOG(ERR, "Memory allocation failed for "
				"Tx queue SW structure\n");
		err = -ENOMEM;
		goto tx_setup_err;
	}

	txq->st_mode = qdma_dev->q_info[tx_queue_id].queue_mode;
	txq->en_bypass = (qdma_dev->q_info[tx_queue_id].tx_bypass_mode) ? 1 : 0;
	txq->bypass_desc_sz = qdma_dev->q_info[tx_queue_id].tx_bypass_desc_sz;

	txq->nb_tx_desc = (nb_tx_desc + 1);
	txq->queue_id = tx_queue_id;
	txq->dev = dev;
	txq->port_id = dev->data->port_id;
	txq->func_id = qdma_dev->func_id;
	txq->num_queues = dev->data->nb_tx_queues;
	txq->tx_deferred_start = tx_conf->tx_deferred_start;
	txq->offloads = tx_conf->offloads | dev->data->dev_conf.txmode.offloads;

	txq->ringszidx = index_of_array(qdma_dev->g_ring_sz,
					QDMA_NUM_RING_SIZES, txq->nb_tx_desc);
	if (txq->ringszidx < 0) {
		PMD_DRV_LOG(ERR, "Expected Ring size %d not found\n",
				txq->nb_tx_desc);
		err = -EINVAL;
		goto tx_setup_err;
	}

	/* Allocate memory for TX descriptor ring */
	if (txq->st_mode) {
		if (!qdma_dev->dev_cap.st_en) {
			PMD_DRV_LOG(ERR, "Streaming mode not enabled "
					"in the hardware\n");
			err = -EINVAL;
			goto tx_setup_err;
		}

		if (txq->en_bypass &&
			(txq->bypass_desc_sz != 0))
			sz = (txq->nb_tx_desc) * (txq->bypass_desc_sz);
		else
			sz = (txq->nb_tx_desc) *
					sizeof(struct qdma_ul_st_h2c_desc);
		txq->tx_mz = qdma_zone_reserve(dev, "TxHwRn", tx_queue_id, sz,
						socket_id);
		if (!txq->tx_mz) {
			PMD_DRV_LOG(ERR, "Couldn't reserve memory for "
					"ST H2C ring of size %d\n", sz);
			err = -ENOMEM;
			goto tx_setup_err;
		}

		txq->tx_ring = txq->tx_mz->addr;
		tx_ring_st = (struct qdma_ul_st_h2c_desc *)txq->tx_ring;

		tx_ring_bypass = (uint8_t *)txq->tx_ring;
		/* Write-back status structure */
		if (txq->en_bypass &&
			(txq->bypass_desc_sz != 0))
			txq->wb_status = (struct wb_status *)&
					tx_ring_bypass[(txq->nb_tx_desc - 1) *
					(txq->bypass_desc_sz)];
		else
			txq->wb_status = (struct wb_status *)&
					tx_ring_st[txq->nb_tx_desc - 1];
	} else {
		if (!qdma_dev->dev_cap.mm_en) {
			PMD_DRV_LOG(ERR, "Memory mapped mode not "
					"enabled in the hardware\n");
			err = -EINVAL;
			goto tx_setup_err;
		}

		if (txq->en_bypass &&
			(txq->bypass_desc_sz != 0))
			sz = (txq->nb_tx_desc) * (txq->bypass_desc_sz);
		else
			sz = (txq->nb_tx_desc) * sizeof(struct qdma_ul_mm_desc);
		txq->tx_mz = qdma_zone_reserve(dev, "TxHwRn", tx_queue_id,
						sz, socket_id);
		if (!txq->tx_mz) {
			PMD_DRV_LOG(ERR, "Couldn't reserve memory for "
					"MM H2C ring of size %d\n", sz);
			err = -ENOMEM;
			goto tx_setup_err;
		}

		txq->tx_ring = txq->tx_mz->addr;
		tx_ring_mm = (struct qdma_ul_mm_desc *)txq->tx_ring;

		/* Write-back status structure */

		tx_ring_bypass = (uint8_t *)txq->tx_ring;
		if (txq->en_bypass &&
			(txq->bypass_desc_sz != 0))
			txq->wb_status = (struct wb_status *)&
				tx_ring_bypass[(txq->nb_tx_desc - 1) *
				(txq->bypass_desc_sz)];
		else
			txq->wb_status = (struct wb_status *)&
				tx_ring_mm[txq->nb_tx_desc - 1];
	}

	PMD_DRV_LOG(INFO, "Tx ring phys addr: 0x%lX, Tx Ring virt addr: 0x%lX",
	    (uint64_t)txq->tx_mz->phys_addr, (uint64_t)txq->tx_ring);

	/* Allocate memory for TX software ring */
	sz = txq->nb_tx_desc * sizeof(struct rte_mbuf *);
	txq->sw_ring = rte_zmalloc("TxSwRn", sz, RTE_CACHE_LINE_SIZE);
	if (txq->sw_ring == NULL) {
		PMD_DRV_LOG(ERR, "Memory allocation failed for "
				 "Tx queue SW ring\n");
		err = -ENOMEM;
		goto tx_setup_err;
	}

	rte_spinlock_init(&txq->pidx_update_lock);
	/* store tx_pkt_burst function pointer */
	dev->tx_pkt_burst = qdma_xmit_pkts;
	dev->data->tx_queues[tx_queue_id] = txq;

	return 0;

tx_setup_err:
	PMD_DRV_LOG(ERR, " Tx queue setup failed");
	if (!qdma_dev->is_vf)
		qdma_dev_decrement_active_queue(pci_dev->addr.bus,
						qdma_dev->func_id,
						QDMA_DEV_Q_TYPE_H2C);
	else
		qdma_dev_notify_qdel(dev, tx_queue_id +
				qdma_dev->queue_base, QDMA_DEV_Q_TYPE_H2C);
	if (txq) {
		if (txq->tx_mz)
			rte_memzone_free(txq->tx_mz);
		if (txq->sw_ring)
			rte_free(txq->sw_ring);
		rte_free(txq);
	}
	return err;
}

#if (MIN_TX_PIDX_UPDATE_THRESHOLD > 1)
void qdma_txq_pidx_update(void *arg)
{
	struct rte_eth_dev *dev = (struct rte_eth_dev *)arg;
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_tx_queue *txq;
	uint32_t qid;

	for (qid = 0; qid < dev->data->nb_tx_queues; qid++) {
		txq = (struct qdma_tx_queue *)dev->data->tx_queues[qid];
		if (txq->tx_desc_pend) {
			rte_spinlock_lock(&txq->pidx_update_lock);
			if (txq->tx_desc_pend) {
				qdma_dev->hw_access->qdma_queue_pidx_update(dev,
					qdma_dev->is_vf,
					qid, 0, &txq->q_pidx_info);

				txq->tx_desc_pend = 0;
			}
			rte_spinlock_unlock(&txq->pidx_update_lock);
		}
	}
	rte_eal_alarm_set(QDMA_TXQ_PIDX_UPDATE_INTERVAL,
			qdma_txq_pidx_update, (void *)arg);
}
#endif


void qdma_dev_tx_queue_release(void *tqueue)
{
	struct qdma_tx_queue *txq = (struct qdma_tx_queue *)tqueue;
	struct qdma_pci_dev *qdma_dev;
	struct rte_pci_device *pci_dev;

	if (txq != NULL) {
		PMD_DRV_LOG(INFO, "Remove H2C queue: %d", txq->queue_id);
		qdma_dev = txq->dev->data->dev_private;
		pci_dev = RTE_ETH_DEV_TO_PCI(txq->dev);

		if (!qdma_dev->is_vf)
			qdma_dev_decrement_active_queue(pci_dev->addr.bus,
							qdma_dev->func_id,
							QDMA_DEV_Q_TYPE_H2C);
		else
			qdma_dev_notify_qdel(txq->dev, txq->queue_id +
						qdma_dev->queue_base,
						QDMA_DEV_Q_TYPE_H2C);
		if (txq->sw_ring)
			rte_free(txq->sw_ring);
		if (txq->tx_mz)
			rte_memzone_free(txq->tx_mz);
		rte_free(txq);
		PMD_DRV_LOG(INFO, "H2C queue %d removed", txq->queue_id);
	}
}

void qdma_dev_rx_queue_release(void *rqueue)
{
	struct qdma_rx_queue *rxq = (struct qdma_rx_queue *)rqueue;
	struct qdma_pci_dev *qdma_dev = NULL;
	struct rte_pci_device *pci_dev = NULL;

	if (rxq != NULL) {
		PMD_DRV_LOG(INFO, "Remove C2H queue: %d", rxq->queue_id);
		qdma_dev = rxq->dev->data->dev_private;
		pci_dev = RTE_ETH_DEV_TO_PCI(rxq->dev);

		if (!qdma_dev->is_vf) {
			qdma_dev_decrement_active_queue(pci_dev->addr.bus,
					qdma_dev->func_id, QDMA_DEV_Q_TYPE_C2H);

			if (rxq->st_mode)
				qdma_dev_decrement_active_queue(
					pci_dev->addr.bus,
					qdma_dev->func_id,
					QDMA_DEV_Q_TYPE_CMPT);
		} else {
			qdma_dev_notify_qdel(rxq->dev, rxq->queue_id +
					qdma_dev->queue_base,
					QDMA_DEV_Q_TYPE_C2H);

			if (rxq->st_mode)
				qdma_dev_notify_qdel(rxq->dev, rxq->queue_id +
						qdma_dev->queue_base,
						QDMA_DEV_Q_TYPE_CMPT);
		}

		if (rxq->sw_ring)
			rte_free(rxq->sw_ring);
		if (rxq->st_mode) { /** if ST-mode **/
			if (rxq->rx_cmpt_mz)
				rte_memzone_free(rxq->rx_cmpt_mz);
		}
		if (rxq->rx_mz)
			rte_memzone_free(rxq->rx_mz);
		rte_free(rxq);
		PMD_DRV_LOG(INFO, "C2H queue %d removed", rxq->queue_id);
	}
}

static void qdma_user_h2c_c2h_reset(struct rte_eth_dev *dev)
{

	struct qdma_pci_dev *dma_priv;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
				(QEP_BASE_OFFSET + QEP_USER_H2C_CONV_RST), 1);
	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
				(QEP_BASE_OFFSET + QEP_USER_H2C_CONV_RST), 0);
	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
				(QEP_BASE_OFFSET + QEP_USER_C2H_CONV_RST), 1);
	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
				(QEP_BASE_OFFSET + QEP_USER_C2H_CONV_RST), 0);

}

/**
 * DPDK callback to start the device.
 *
 * Start the device by configuring the Rx/Tx descriptor and device registers.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */
static int qdma_dev_start(struct rte_eth_dev *dev)
{
	struct qdma_tx_queue *txq;
	struct qdma_rx_queue *rxq;
	struct rte_eth_conf *eth_conf = &dev->data->dev_conf;
	uint64_t rx_offloads = eth_conf->rxmode.offloads;
	uint64_t rsstype;
	uint32_t qid;
	int err;

	PMD_DRV_LOG(INFO, "qdma-dev-start: Starting\n");

	if (rx_offloads & DEV_RX_OFFLOAD_JUMBO_FRAME) {
		if (eth_conf->rxmode.max_rx_pkt_len <= QDMA_MAX_RX_PKTLEN &&
		    eth_conf->rxmode.max_rx_pkt_len > ETHER_MAX_LEN) {
			xcmac_set_mtu(dev, eth_conf->rxmode.max_rx_pkt_len);
			dev->data->mtu = eth_conf->rxmode.max_rx_pkt_len -
					(ETHER_HDR_LEN + ETHER_CRC_LEN);
		} else {
			PMD_DRV_LOG(ERR, "max_rx_pkt_len %u is invalid",
				    eth_conf->rxmode.max_rx_pkt_len);
			return -EINVAL;
		}
	}

	qdma_user_h2c_c2h_reset(dev);
	rsstype = eth_conf->rx_adv_conf.rss_conf.rss_hf;
	if (!rsstype) {
		qep_set_rx_mode(dev, QEP_RX_MODE_RR);
	} else {
		if ((rsstype &  QEP_RSS_FLOW_TYPE) == rsstype) {
			qep_init_reta(dev, 0);
		} else {
			PMD_DRV_LOG(INFO, "%s expected rss type should be subset of %llx, "
			"received %lx\n", __func__, QEP_RSS_FLOW_TYPE, rsstype);
			return -EINVAL;
		}
	}

	/* prepare descriptor rings for operation */
	for (qid = 0; qid < dev->data->nb_tx_queues; qid++) {
		txq = (struct qdma_tx_queue *)dev->data->tx_queues[qid];

		/*Deferred Queues should not start with dev_start*/
		if (!txq->tx_deferred_start) {
			err = qdma_dev_tx_queue_start(dev, qid);
			if (err != 0)
				return err;
		}
	}

	for (qid = 0; qid < dev->data->nb_rx_queues; qid++) {
		rxq = (struct qdma_rx_queue *)dev->data->rx_queues[qid];

		/*Deferred Queues should not start with dev_start*/
		if (!rxq->rx_deferred_start) {
			err = qdma_dev_rx_queue_start(dev, qid);
			if (err != 0)
				return err;
		}
	}

	err = xcmac_start(dev);
	if (err != 0) {
		PMD_DRV_LOG(ERR, "%s: xcmac_start failed with status %d\n",
			    __func__, err);
		return err;
	}

	/* Start thread to monitor link status */

#if (MIN_TX_PIDX_UPDATE_THRESHOLD > 1)
	rte_eal_alarm_set(QDMA_TXQ_PIDX_UPDATE_INTERVAL,
			qdma_txq_pidx_update, (void *)dev);
#endif
	return 0;
}

/**
 * DPDK callback to retrieve the physical link information.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param wait_to_complete
 *   If 0, get the current link status
 *   If 1, wait till the Ethernet link is up or timeout is reached.
 */
static int qdma_dev_link_update(struct rte_eth_dev *dev,
				int wait_to_complete)
{
	int ret = 0, retry;
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct xcmac_rx_lane_am_status lane_status;
	struct rte_eth_link link;

	retry = (wait_to_complete) ? QEP_LINK_STATUS_TIMEOUT : 0;
	memset(&link, 0, sizeof(link));

	do {
		ret = xcmac_get_rx_lane_status(&qdma_dev->cmac_instance,
					       &lane_status);
		if (ret != 0) {
			RTE_LOG(INFO, PMD,
				"%s: Error in retrieving Rx lane status\n",
				__func__);
			link.link_status = ETH_LINK_DOWN;
		} else {
			if (lane_status.aligned == true) {
				link.link_status = ETH_LINK_UP;
				break;
			}
		}
		rte_delay_ms(QEP_LINK_STATUS_MS);

	} while (retry-- != 0);

	if (lane_status.aligned == true)
		link.link_status = ETH_LINK_UP;
	else
		link.link_status = ETH_LINK_DOWN;

	link.link_duplex = ETH_LINK_FULL_DUPLEX;
	link.link_speed = ETH_SPEED_NUM_100G;

	PMD_DRV_LOG(INFO, "Link update done\n");

	return rte_eth_linkstatus_set(dev, &link);
}

/**
 * DPDK callback to get information about the device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param[out] dev_info
 *   Device information structure output buffer.
 */
static void qdma_dev_infos_get(__rte_unused struct rte_eth_dev *dev,
				struct rte_eth_dev_info *dev_info)
{
	dev_info->max_rx_queues = QEP_MAX_STMN_QUEUES;
	dev_info->max_tx_queues = QEP_MAX_STMN_QUEUES;
	dev_info->max_rx_pktlen = QDMA_MAX_RX_PKTLEN;
	dev_info->min_rx_bufsize = ETHER_MIN_MTU;
	dev_info->rx_offload_capa = QEP_RX_CAPA;
	dev_info->tx_offload_capa = QEP_TX_CAPA;
	dev_info->max_mac_addrs = 1;
	dev_info->hash_key_size = QEP_RSS_HKEY_SIZE;
	dev_info->reta_size = QEP_RSS_TBL_SIZE;
	dev_info->flow_type_rss_offloads = QEP_RSS_FLOW_TYPE;
}

/**
 * DPDK callback to stop the device.
 *
 * Stop the device by clearing all configured Rx/Tx queue
 * descriptors and registers.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 */
static void qdma_dev_stop(struct rte_eth_dev *dev)
{
#ifdef RTE_LIBRTE_QDMA_DEBUG_DRIVER
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
#endif
	uint32_t qid;
	int ret = 0;

	/* Stop CMAC */
	ret = xcmac_stop(dev);
	if (ret != 0) {
		PMD_DRV_LOG(ERR, "%s: xcmac_stop failed with status %d\n",
			    __func__, ret);
	}

	/* reset driver's internal queue structures to default values */
	PMD_DRV_LOG(INFO, "PF-%d(DEVFN) Stop H2C & C2H queues",
			qdma_dev->func_id);
	for (qid = 0; qid < dev->data->nb_tx_queues; qid++)
		qdma_dev_tx_queue_stop(dev, qid);
	for (qid = 0; qid < dev->data->nb_rx_queues; qid++)
		qdma_dev_rx_queue_stop(dev, qid);

#if (MIN_TX_PIDX_UPDATE_THRESHOLD > 1)
	/* Cancel pending PIDX updates */
	rte_eal_alarm_cancel(qdma_txq_pidx_update, (void *)dev);
#endif
}

/**
 * DPDK callback to close the device.
 *
 * Destroy all queues and objects, free memory.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 */
void qdma_dev_close(struct rte_eth_dev *dev)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct qdma_tx_queue *txq;
	struct qdma_rx_queue *rxq;
	struct qdma_cmpt_queue *cmptq;
	uint32_t qid;
	struct qdma_fmap_cfg fmap_cfg;

	PMD_DRV_LOG(INFO, "PF-%d(DEVFN) DEV Close\n", qdma_dev->func_id);

	if (dev->data->dev_started)
		qdma_dev_stop(dev);

	memset(&fmap_cfg, 0, sizeof(struct qdma_fmap_cfg));
	qdma_dev->hw_access->qdma_fmap_conf(dev,
		qdma_dev->func_id, &fmap_cfg, QDMA_HW_ACCESS_CLEAR);

	/* iterate over rx queues */
	for (qid = 0; qid < dev->data->nb_rx_queues; ++qid) {
		rxq = dev->data->rx_queues[qid];
		if (rxq) {
			PMD_DRV_LOG(INFO, "Remove C2H queue: %d", qid);

			if (rxq->sw_ring)
				rte_free(rxq->sw_ring);
			if (rxq->st_mode) { /** if ST-mode **/
				if (rxq->rx_cmpt_mz)
					rte_memzone_free(rxq->rx_cmpt_mz);
			}

			qdma_dev_decrement_active_queue(pci_dev->addr.bus,
					qdma_dev->func_id, QDMA_DEV_Q_TYPE_C2H);

			if (rxq->st_mode)
				qdma_dev_decrement_active_queue(
					pci_dev->addr.bus,
					qdma_dev->func_id,
					QDMA_DEV_Q_TYPE_CMPT);

			if (rxq->rx_mz)
				rte_memzone_free(rxq->rx_mz);
			rte_free(rxq);
			PMD_DRV_LOG(INFO, "C2H queue %d removed", qid);
		}
	}

	/* iterate over tx queues */
	for (qid = 0; qid < dev->data->nb_tx_queues; ++qid) {
		txq = dev->data->tx_queues[qid];
		if (txq) {
			PMD_DRV_LOG(INFO, "Remove H2C queue: %d", qid);

			if (txq->sw_ring)
				rte_free(txq->sw_ring);
			if (txq->tx_mz)
				rte_memzone_free(txq->tx_mz);
			rte_free(txq);
			PMD_DRV_LOG(INFO, "H2C queue %d removed", qid);

			qdma_dev_decrement_active_queue(pci_dev->addr.bus,
					qdma_dev->func_id, QDMA_DEV_Q_TYPE_H2C);
		}
	}
	if (qdma_dev->dev_cap.mm_cmpt_en) {
		/* iterate over cmpt queues */
		for (qid = 0; qid < qdma_dev->qsets_en; ++qid) {
			cmptq = qdma_dev->cmpt_queues[qid];
			if (cmptq != NULL) {
				PMD_DRV_LOG(INFO, "PF-%d(DEVFN) Remove CMPT queue: %d",
						qdma_dev->func_id, qid);
				if (cmptq->cmpt_mz)
					rte_memzone_free(cmptq->cmpt_mz);
				rte_free(cmptq);
				PMD_DRV_LOG(INFO, "PF-%d(DEVFN) CMPT queue %d removed",
						qdma_dev->func_id, qid);
				qdma_dev_decrement_active_queue(
					pci_dev->addr.bus,
					qdma_dev->func_id,
					QDMA_DEV_Q_TYPE_CMPT);
			}
		}

		if (qdma_dev->cmpt_queues != NULL) {
			rte_free(qdma_dev->cmpt_queues);
			qdma_dev->cmpt_queues = NULL;
		}
	}
	qdma_dev->qsets_en = 0;
	qdma_dev_update(pci_dev->addr.bus, qdma_dev->func_id,
			qdma_dev->qsets_en, (int *)&qdma_dev->queue_base);
	qdma_dev->init_q_range = 0;
	rte_free(qdma_dev->q_info);
	qdma_dev->q_info = NULL;
	qdma_dev->dev_configured = 0;
}

/**
 * DPDK callback to set MTU.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param mtu
 *   MTU to set.
 */
int qdma_dev_mtu_set(struct rte_eth_dev *dev, uint16_t mtu)
{
	struct rte_eth_dev_info dev_info;
	uint16_t new_mtu = mtu + ETHER_HDR_LEN + ETHER_CRC_LEN;

	qdma_dev_infos_get(dev, &dev_info);

	/* Must accommodate at least ETHER_MIN_MTU */
	if ((new_mtu < ETHER_MIN_MTU) || (new_mtu > dev_info.max_rx_pktlen))
		return -EINVAL;

	if (dev->data->dev_started) {
		PMD_DRV_LOG(ERR, "Stop port first to set MTU");
		return -EINVAL;
	}
	/* set to jumbo mode if needed */
	if (new_mtu > ETHER_MAX_LEN)
		dev->data->dev_conf.rxmode.offloads |=
			DEV_RX_OFFLOAD_JUMBO_FRAME;
	else
		dev->data->dev_conf.rxmode.offloads &=
			~DEV_RX_OFFLOAD_JUMBO_FRAME;

	xcmac_set_mtu(dev, new_mtu);
	dev->data->dev_conf.rxmode.max_rx_pkt_len = new_mtu;

	return 0;
}

static int validate_offload_support(struct rte_eth_dev *dev)
{
	if (dev->data->dev_conf.rxmode.offloads & ~QEP_RX_CAPA) {
		PMD_DRV_LOG(ERR, "Requested Rx offloads (0x%lx) not supported",
				dev->data->dev_conf.rxmode.offloads);
		return -EINVAL;
	}

	if (dev->data->dev_conf.txmode.offloads & ~QEP_TX_CAPA) {
		PMD_DRV_LOG(ERR, "Requested Tx offloads (0x%lx) not supported",
				dev->data->dev_conf.txmode.offloads);
		return -EINVAL;
	}

	return 0;
}

/**
 * DPDK callback to reset the device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */
static int qdma_dev_reset(struct rte_eth_dev *dev)
{
#ifdef RTE_LIBRTE_QDMA_DEBUG_DRIVER
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
#endif
	int ret = 0;

	/* Uninitialze PCI device */
	ret = qdma_eth_dev_uninit(dev);
	if (ret != QDMA_SUCCESS) {
		PMD_DRV_LOG(ERR, "PF-%d(DEVFN) uninitialization failed: %d\n",
			qdma_dev->func_id, ret);
		return -1;
	}

	/* Initialize PCI device */
	ret = qdma_eth_dev_init(dev);
	if (ret != QDMA_SUCCESS) {
		PMD_DRV_LOG(ERR, "PF-%d(DEVFN) initialization failed: %d\n",
			qdma_dev->func_id, ret);
		return -1;
	}

	return ret;
}

/**
 * DPDK callback for Ethernet device configuration.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */
static int qdma_dev_configure(struct rte_eth_dev *dev)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
	struct rte_eth_conf *eth_conf = &dev->data->dev_conf;
	uint64_t rsstype;
	uint16_t qid = 0;
	int ret = 0, queue_base = -1;
	uint8_t stat_id;

	rsstype = eth_conf->rx_adv_conf.rss_conf.rss_hf;
	if (qdma_dev->dev_configured == 0) {
		PMD_DRV_LOG(INFO, "Configure the qdma engines\n");

		qdma_dev->qsets_en = RTE_MAX(dev->data->nb_rx_queues,
						dev->data->nb_tx_queues);
		if (qdma_dev->qsets_en > qdma_dev->dev_cap.num_qs) {
			PMD_DRV_LOG(ERR, "PF-%d(DEVFN) Error: Number of Queues to be"
					" configured are greater than the queues"
					" supported by the hardware\n",
					qdma_dev->func_id);
			qdma_dev->qsets_en = 0;
			return -1;
		}
		qdma_dev->nb_rx_queues = dev->data->nb_rx_queues;
		qdma_dev->nb_tx_queues = dev->data->nb_tx_queues;
		if ((rsstype & QEP_RSS_FLOW_TYPE) != rsstype) {
			PMD_DRV_LOG(INFO, "%s expected rss type should be "
			"subset of %llx, received %lx\n",
			__func__, QEP_RSS_FLOW_TYPE, rsstype);
			return -EINVAL;
		}
		ret = validate_offload_support(dev);
		if (ret != 0)
			return ret;

		/* Request queue base from the resource manager */
		ret = qdma_dev_update(pci_dev->addr.bus, qdma_dev->func_id,
				qdma_dev->qsets_en, &queue_base);
		if (ret != QDMA_SUCCESS) {
			PMD_DRV_LOG(ERR, "PF-%d(DEVFN) queue allocation failed: %d\n",
				qdma_dev->func_id, ret);
			return -1;
		}

		ret = qdma_dev_qinfo_get(pci_dev->addr.bus, qdma_dev->func_id,
					(int *)&qdma_dev->queue_base,
					&qdma_dev->qsets_en);
		if (ret != QDMA_SUCCESS) {
			PMD_DRV_LOG(ERR, "%s: Error %d querying qbase\n",
					__func__, ret);
			return -1;
		}
		PMD_DRV_LOG(INFO, "Bus: 0x%x, PF-%d(DEVFN) queue_base: %d\n",
			pci_dev->addr.bus, qdma_dev->func_id,
			qdma_dev->queue_base);

		qdma_dev->q_info = rte_zmalloc("qinfo",
					sizeof(struct queue_info) *
					(qdma_dev->qsets_en),
					0);
		if (!qdma_dev->q_info) {
			PMD_DRV_LOG(ERR, "PF-%d(DEVFN) Cannot allocate "
				"memory for queue info\n", qdma_dev->func_id);
			return (-ENOMEM);
		}

		/* Reserve memory for cmptq ring pointers
		 * Max completion queues can be maximum of rx and tx queues.
		 */
		qdma_dev->cmpt_queues = rte_zmalloc("cmpt_queues",
					sizeof(qdma_dev->cmpt_queues[0]) *
					qdma_dev->qsets_en,
					RTE_CACHE_LINE_SIZE);
		if (qdma_dev->cmpt_queues == NULL) {
			PMD_DRV_LOG(ERR, "PF-%d(DEVFN) cmpt ring pointers memory "
				"allocation failed:\n", qdma_dev->func_id);
			rte_free(qdma_dev->q_info);
			qdma_dev->q_info = NULL;
			return -(ENOMEM);
		}

		for (qid = 0; qid < qdma_dev->qsets_en; qid++) {
			/* Initialize queue_modes Streaming */
			qdma_dev->q_info[qid].queue_mode =
					RTE_PMD_QDMA_STREAMING_MODE;

			/* Disable the cmpt over flow check by default */
			qdma_dev->q_info[qid].dis_cmpt_ovf_chk = 0;

			qdma_dev->q_info[qid].trigger_mode =
						qdma_dev->trigger_mode;
			qdma_dev->q_info[qid].timer_count =
						qdma_dev->timer_count;
		}

		for (qid = 0; qid < dev->data->nb_rx_queues; qid++) {
			qdma_dev->q_info[qid].cmpt_desc_sz =
						qdma_dev->cmpt_desc_len;
			qdma_dev->q_info[qid].rx_bypass_mode =
						qdma_dev->c2h_bypass_mode;
			qdma_dev->q_info[qid].en_prefetch =
						qdma_dev->en_desc_prefetch;
			qdma_dev->q_info[qid].immediate_data_state = 0;
		}

		for (qid = 0; qid < dev->data->nb_tx_queues; qid++)
			qdma_dev->q_info[qid].tx_bypass_mode =
						qdma_dev->h2c_bypass_mode;

		for (stat_id = 0, qid = 0;
			stat_id < RTE_ETHDEV_QUEUE_STAT_CNTRS;
			stat_id++, qid++) {
			/* Initialize map with qid same as stat_id */
			qdma_dev->tx_qid_statid_map[stat_id] =
				(qid < dev->data->nb_tx_queues) ? qid : -1;
			qdma_dev->rx_qid_statid_map[stat_id] =
				(qid < dev->data->nb_rx_queues) ? qid : -1;
		}

		ret = qdma_pf_fmap_prog(dev);
		if (ret < 0) {
			PMD_DRV_LOG(ERR, "FMAP programming failed\n");
			rte_free(qdma_dev->q_info);
			qdma_dev->q_info = NULL;
			rte_free(qdma_dev->cmpt_queues);
			qdma_dev->cmpt_queues = NULL;
			return ret;
		}

		qep_set_rx_mode(dev, QEP_RX_MODE_RR);

		qdma_dev->dev_configured = 1;
	} else if ((qdma_dev->dev_configured == 1) &&
			   (qdma_dev->nb_rx_queues ==
					dev->data->nb_rx_queues) &&
			   (qdma_dev->nb_tx_queues ==
					dev->data->nb_tx_queues)) {
		PMD_DRV_LOG(INFO, "Device reconfigured with same number of queues\n");
		ret = validate_offload_support(dev);
		if (ret != 0)
			return ret;
	} else {
		PMD_DRV_LOG(INFO, "Currently driver doesn't support "
					" device reconfiguration with number of"
					" queues changed\n");
		return -EINVAL;
	}

	return 0;
}

int qdma_dev_tx_queue_start(struct rte_eth_dev *dev, uint16_t qid)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_tx_queue *txq;
	uint32_t queue_base =  qdma_dev->queue_base;
	int err, bypass_desc_sz_idx;
	struct qdma_descq_sw_ctxt q_sw_ctxt;
	struct qdma_hw_access *hw_access = qdma_dev->hw_access;

	txq = (struct qdma_tx_queue *)dev->data->tx_queues[qid];

	memset(&q_sw_ctxt, 0, sizeof(struct qdma_descq_sw_ctxt));

	bypass_desc_sz_idx = qmda_get_desc_sz_idx(txq->bypass_desc_sz);

	qdma_reset_tx_queue(txq);
	qdma_clr_tx_queue_ctxts(dev, (qid + queue_base), txq->st_mode);

	if (txq->st_mode) {
		q_sw_ctxt.desc_sz = SW_DESC_CNTXT_H2C_STREAM_DMA;
		q_sw_ctxt.frcd_en = txq->en_bypass;
	} else {
		q_sw_ctxt.desc_sz = SW_DESC_CNTXT_MEMORY_MAP_DMA;
		q_sw_ctxt.is_mm = 1;
	}
	q_sw_ctxt.wbi_chk = 1;
	q_sw_ctxt.wbi_intvl_en = 1;
	q_sw_ctxt.fnc_id = txq->func_id;
	q_sw_ctxt.qen = 1;
	q_sw_ctxt.rngsz_idx = txq->ringszidx;
	q_sw_ctxt.bypass = txq->en_bypass;
	q_sw_ctxt.wbk_en = 1;
	q_sw_ctxt.ring_bs_addr = (uint64_t)txq->tx_mz->phys_addr;

	if (txq->en_bypass &&
		(txq->bypass_desc_sz != 0))
		q_sw_ctxt.desc_sz = bypass_desc_sz_idx;

	/* Set SW Context */
	err = hw_access->qdma_sw_ctx_conf(dev, 0,
			(qid + queue_base), &q_sw_ctxt,
			QDMA_HW_ACCESS_WRITE);
	if (err != QDMA_SUCCESS)
		return qdma_dev->hw_access->qdma_get_error_code(err);

	txq->q_pidx_info.pidx = 0;
	hw_access->qdma_queue_pidx_update(dev, qdma_dev->is_vf,
		qid, 0, &txq->q_pidx_info);

	dev->data->tx_queue_state[qid] = RTE_ETH_QUEUE_STATE_STARTED;
	txq->status = RTE_ETH_QUEUE_STATE_STARTED;
	return 0;
}


int qdma_dev_rx_queue_start(struct rte_eth_dev *dev, uint16_t qid)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_rx_queue *rxq;
	uint32_t queue_base =  qdma_dev->queue_base;
	uint8_t cmpt_desc_fmt;
	int err, bypass_desc_sz_idx;
	struct qdma_descq_sw_ctxt q_sw_ctxt;
	struct qdma_descq_cmpt_ctxt q_cmpt_ctxt;
	struct qdma_descq_prefetch_ctxt q_prefetch_ctxt;
	struct qdma_hw_access *hw_access = qdma_dev->hw_access;

	rxq = (struct qdma_rx_queue *)dev->data->rx_queues[qid];

	memset(&q_sw_ctxt, 0, sizeof(struct qdma_descq_sw_ctxt));

	qdma_reset_rx_queue(rxq);
	qdma_clr_rx_queue_ctxts(dev, (qid + queue_base), rxq->st_mode);

	bypass_desc_sz_idx = qmda_get_desc_sz_idx(rxq->bypass_desc_sz);

	switch (rxq->cmpt_desc_len) {
	case RTE_PMD_QDMA_CMPT_DESC_LEN_8B:
		cmpt_desc_fmt = CMPT_CNTXT_DESC_SIZE_8B;
		break;
	case RTE_PMD_QDMA_CMPT_DESC_LEN_16B:
		cmpt_desc_fmt = CMPT_CNTXT_DESC_SIZE_16B;
		break;
	case RTE_PMD_QDMA_CMPT_DESC_LEN_32B:
		cmpt_desc_fmt = CMPT_CNTXT_DESC_SIZE_32B;
		break;
	case RTE_PMD_QDMA_CMPT_DESC_LEN_64B:
		cmpt_desc_fmt = CMPT_CNTXT_DESC_SIZE_64B;
		break;
	default:
		cmpt_desc_fmt = CMPT_CNTXT_DESC_SIZE_8B;
		break;
	}

	err = qdma_init_rx_queue(rxq);
	if (err != 0)
		return err;

	if (rxq->st_mode) {
		memset(&q_cmpt_ctxt, 0, sizeof(struct qdma_descq_cmpt_ctxt));
		memset(&q_prefetch_ctxt, 0,
				sizeof(struct qdma_descq_prefetch_ctxt));

		q_prefetch_ctxt.bypass = (rxq->en_bypass_prefetch) ? 1 : 0;
		q_prefetch_ctxt.bufsz_idx = rxq->buffszidx;
		q_prefetch_ctxt.pfch_en = (rxq->en_prefetch) ? 1 : 0;
		q_prefetch_ctxt.valid = 1;

		q_cmpt_ctxt.en_stat_desc = 1;
		q_cmpt_ctxt.trig_mode = rxq->triggermode;
		q_cmpt_ctxt.fnc_id = rxq->func_id;
		q_cmpt_ctxt.counter_idx = rxq->threshidx;
		q_cmpt_ctxt.timer_idx = rxq->timeridx;
		q_cmpt_ctxt.color = CMPT_DEFAULT_COLOR_BIT;
		q_cmpt_ctxt.ringsz_idx = rxq->cmpt_ringszidx;
		q_cmpt_ctxt.bs_addr = (uint64_t)rxq->rx_cmpt_mz->phys_addr;
		q_cmpt_ctxt.desc_sz = cmpt_desc_fmt;
		q_cmpt_ctxt.valid = 1;
		if (qdma_dev->dev_cap.cmpt_ovf_chk_dis)
			q_cmpt_ctxt.ovf_chk_dis = rxq->dis_overflow_check;


		q_sw_ctxt.desc_sz = SW_DESC_CNTXT_C2H_STREAM_DMA;
		q_sw_ctxt.frcd_en = 1;
	} else {
		q_sw_ctxt.desc_sz = SW_DESC_CNTXT_MEMORY_MAP_DMA;
		q_sw_ctxt.is_mm = 1;
		q_sw_ctxt.wbi_chk = 1;
		q_sw_ctxt.wbi_intvl_en = 1;
	}

	q_sw_ctxt.fnc_id = rxq->func_id;
	q_sw_ctxt.qen = 1;
	q_sw_ctxt.rngsz_idx = rxq->ringszidx;
	q_sw_ctxt.bypass = rxq->en_bypass;
	q_sw_ctxt.wbk_en = 1;
	q_sw_ctxt.ring_bs_addr = (uint64_t)rxq->rx_mz->phys_addr;

	if (rxq->en_bypass &&
		(rxq->bypass_desc_sz != 0))
		q_sw_ctxt.desc_sz = bypass_desc_sz_idx;

	/* Set SW Context */
	err = hw_access->qdma_sw_ctx_conf(dev, 1, (qid + queue_base),
			&q_sw_ctxt, QDMA_HW_ACCESS_WRITE);
	if (err != QDMA_SUCCESS)
		return qdma_dev->hw_access->qdma_get_error_code(err);

	if (rxq->st_mode) {
		/* Set Prefetch Context */
		err = hw_access->qdma_pfetch_ctx_conf(dev, (qid + queue_base),
				&q_prefetch_ctxt, QDMA_HW_ACCESS_WRITE);
		if (err != QDMA_SUCCESS)
			return qdma_dev->hw_access->qdma_get_error_code(err);

		/* Set Completion Context */
		err = hw_access->qdma_cmpt_ctx_conf(dev, (qid + queue_base),
				&q_cmpt_ctxt, QDMA_HW_ACCESS_WRITE);
		if (err != QDMA_SUCCESS)
			return qdma_dev->hw_access->qdma_get_error_code(err);

		rte_wmb();
		/* enable status desc , loading the triggermode,
		 * thresidx and timeridx passed from the user
		 */

		rxq->cmpt_cidx_info.counter_idx = rxq->threshidx;
		rxq->cmpt_cidx_info.timer_idx = rxq->timeridx;
		rxq->cmpt_cidx_info.trig_mode = rxq->triggermode;
		rxq->cmpt_cidx_info.wrb_en = 1;
		rxq->cmpt_cidx_info.wrb_cidx = 0;
		hw_access->qdma_queue_cmpt_cidx_update(dev, qdma_dev->is_vf,
			qid, &rxq->cmpt_cidx_info);

		rxq->q_pidx_info.pidx = (rxq->nb_rx_desc - 2);
		hw_access->qdma_queue_pidx_update(dev, qdma_dev->is_vf, qid,
				1, &rxq->q_pidx_info);
	}

	dev->data->rx_queue_state[qid] = RTE_ETH_QUEUE_STATE_STARTED;
	rxq->status = RTE_ETH_QUEUE_STATE_STARTED;
	return 0;
}

int qdma_dev_rx_queue_stop(struct rte_eth_dev *dev, uint16_t qid)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	struct qdma_rx_queue *rxq;
	uint32_t queue_base =  qdma_dev->queue_base;
	int i = 0;

	rxq = (struct qdma_rx_queue *)dev->data->rx_queues[qid];

	rxq->status = RTE_ETH_QUEUE_STATE_STOPPED;

	qdma_inv_rx_queue_ctxts(dev, (qid + queue_base), rxq->st_mode);

	if (rxq->st_mode) {  /** ST-mode **/
#ifdef DUMP_MEMPOOL_USAGE_STATS
		PMD_DRV_LOG(INFO, "%s(): %d: queue id %d,"
		"mbuf_avail_count = %d, mbuf_in_use_count = %d",
		__func__, __LINE__, rxq->queue_id,
		rte_mempool_avail_count(rxq->mb_pool),
		rte_mempool_in_use_count(rxq->mb_pool));
#endif //DUMP_MEMPOOL_USAGE_STATS
		for (i = 0; i < rxq->nb_rx_desc - 1; i++) {
			rte_pktmbuf_free(rxq->sw_ring[i]);
			rxq->sw_ring[i] = NULL;
		}
#ifdef DUMP_MEMPOOL_USAGE_STATS
		PMD_DRV_LOG(INFO, "%s(): %d: queue id %d,"
		"mbuf_avail_count = %d, mbuf_in_use_count = %d",
			__func__, __LINE__, rxq->queue_id,
			rte_mempool_avail_count(rxq->mb_pool),
			rte_mempool_in_use_count(rxq->mb_pool));
#endif //DUMP_MEMPOOL_USAGE_STATS
	}

	qdma_reset_rx_queue(rxq);

	dev->data->rx_queue_state[qid] = RTE_ETH_QUEUE_STATE_STOPPED;

	return 0;
}

int qdma_dev_tx_queue_stop(struct rte_eth_dev *dev, uint16_t qid)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	uint32_t queue_base =  qdma_dev->queue_base;
	struct qdma_tx_queue *txq;
	uint16_t count;

	txq = (struct qdma_tx_queue *)dev->data->tx_queues[qid];

	txq->status = RTE_ETH_QUEUE_STATE_STOPPED;
	qdma_inv_tx_queue_ctxts(dev, (qid + queue_base), txq->st_mode);

	/* Relinquish pending mbufs */
	for (count = 0; count < txq->nb_tx_desc - 1; count++) {
		rte_pktmbuf_free(txq->sw_ring[count]);
		txq->sw_ring[count] = NULL;
	}
	qdma_reset_tx_queue(txq);

	dev->data->tx_queue_state[qid] = RTE_ETH_QUEUE_STATE_STOPPED;

	return 0;
}

/**
 * DPDK callback to retrieve device registers and
 * register attributes (number of registers and register size)
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param regs
 *   Pointer to rte_dev_reg_info structure to fill in. If regs->data is
 *   NULL the function fills in the width and length fields. If non-NULL
 *   the registers are put into the buffer pointed at by the data field.
 *
 * @return
 *   0 on success, -ENOTSUP on failure.
 */
static int
qdma_dev_get_regs(struct rte_eth_dev *dev,
	      struct rte_dev_reg_info *regs)
{
	uint32_t *data = regs->data;
	uint32_t count = 0;
	uint32_t reg_length = (sizeof(qdma_config_regs) /
				sizeof(qdma_config_regs[0])) - 1;

	if (data == NULL) {
		regs->length = reg_length;
		regs->width = sizeof(uint32_t);
		return 0;
	}

	/* Support only full register dump */
	if ((regs->length == 0) ||
	    (regs->length == reg_length)) {
		regs->version = 1;
		for (count = 0; count < reg_length; count++) {
			data[count] = qdma_reg_read(dev,
					qdma_config_regs[count].addr);
		}
		return 0;
	}

	PMD_DRV_LOG(ERR, "%s: Unsupported length (0x%x) requested\n",
				__func__, regs->length);
	return -ENOTSUP;
}

/**
 * DPDK callback to set a queue statistics mapping for
 * a tx/rx queue of an Ethernet device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param queue_id
 *   Index of the queue for which a queue stats mapping is required.
 * @param stat_idx
 *   The per-queue packet statistics functionality number that
 *   the queue_id is to be assigned.
 * @param is_rx
 *   Whether queue is a Rx or a Tx queue.
 *
 * @return
 *   0 on success, -EINVAL on failure.
 */
static int qdma_dev_queue_stats_mapping(struct rte_eth_dev *dev,
					     uint16_t queue_id,
					     uint8_t stat_idx,
					     uint8_t is_rx)
{
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;

	if (is_rx && (queue_id >= dev->data->nb_rx_queues)) {
		PMD_DRV_LOG(ERR, "%s: Invalid Rx qid %d\n",
			__func__, queue_id);
		return -EINVAL;
	}

	if (!is_rx && (queue_id >= dev->data->nb_tx_queues)) {
		PMD_DRV_LOG(ERR, "%s: Invalid Tx qid %d\n",
			__func__, queue_id);
		return -EINVAL;
	}

	if (stat_idx >= RTE_ETHDEV_QUEUE_STAT_CNTRS) {
		PMD_DRV_LOG(ERR, "%s: Invalid stats index %d\n",
			__func__, stat_idx);
		return -EINVAL;
	}

	if (is_rx)
		qdma_dev->rx_qid_statid_map[stat_idx] = queue_id;
	else
		qdma_dev->tx_qid_statid_map[stat_idx] = queue_id;

	return 0;
}

/**
 * DPDK callback for retrieving Port statistics.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param eth_stats
 *   Pointer to structure containing statistics.
 *
 * @return
 *   Returns 0 i.e. success
 */
int qdma_dev_stats_get(struct rte_eth_dev *dev,
				struct rte_eth_stats *eth_stats)
{
	uint32_t i;
	int qid;
	struct qdma_rx_queue *rxq;
	struct qdma_tx_queue *txq;
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;

	memset(eth_stats, 0, sizeof(struct rte_eth_stats));
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		rxq = (struct qdma_rx_queue *)dev->data->rx_queues[i];
		eth_stats->ipackets += rxq->stats.pkts;
		eth_stats->ibytes += rxq->stats.bytes;
	}

	for (i = 0; i < RTE_ETHDEV_QUEUE_STAT_CNTRS; i++) {
		qid = qdma_dev->rx_qid_statid_map[i];
		if (qid >= 0) {
			rxq = (struct qdma_rx_queue *)dev->data->rx_queues[qid];
			eth_stats->q_ipackets[i] = rxq->stats.pkts;
			eth_stats->q_ibytes[i] = rxq->stats.bytes;
#ifdef QEP_DEBUG_DESC_USAGE
			RTE_LOG(INFO, PMD, ": Rx Q %d cmpt nb_pkts_avail 0x%x\n",
			qid, rxq->stats.max_desc_used);
#endif
		}
	}

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		txq = (struct qdma_tx_queue *)dev->data->tx_queues[i];
		eth_stats->opackets += txq->stats.pkts;
		eth_stats->obytes   += txq->stats.bytes;
	}

	for (i = 0; i < RTE_ETHDEV_QUEUE_STAT_CNTRS; i++) {
		qid = qdma_dev->tx_qid_statid_map[i];
		if (qid >= 0) {
			txq = (struct qdma_tx_queue *)dev->data->tx_queues[qid];
			eth_stats->q_opackets[i] = txq->stats.pkts;
			eth_stats->q_obytes[i] = txq->stats.bytes;
#ifdef QEP_DEBUG_DESC_USAGE
			RTE_LOG(INFO, PMD, ": Tx Q %d in_use 0x%x\n",
			qid, txq->stats.max_desc_used);
#endif
		}
	}
	return 0;
}

/**
 * DPDK callback to reset Port statistics.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 *
 */
static void qdma_dev_stats_reset(struct rte_eth_dev *dev)
{
	uint32_t i;

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		struct qdma_rx_queue *rxq =
			(struct qdma_rx_queue *)dev->data->rx_queues[i];
		rxq->stats.pkts = 0;
		rxq->stats.bytes = 0;
#ifdef QEP_DEBUG_DESC_USAGE
		rxq->stats.max_desc_used = 0;
#endif
	}

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		struct qdma_tx_queue *txq =
			(struct qdma_tx_queue *)dev->data->tx_queues[i];
		txq->stats.pkts = 0;
		txq->stats.bytes = 0;
#ifdef QEP_DEBUG_DESC_USAGE
		txq->stats.max_desc_used = 0;
#endif
	}
}

/**
 * DPDK callback to get names of extended stats of the device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param xstats_names
 *   Array of name values to be filled in
 * @param size
 *   Number of elements in the xstats_names array
 *
 * @return
 *   When size >= the number of stats,
 *   return the number of stat values filled into the array.
 *
 *   When size < the number of available stats,
 *   return the number of stats values,
 *   and do not fill in any data into xstats_names.
 */
static int
qep_dev_xstats_get_names(__rte_unused struct rte_eth_dev *dev,
		      struct rte_eth_xstat_name *xstats_names,
		      unsigned int size)
{
	uint32_t i, count = 0;
	uint32_t exp_size = QEP_NB_XSTATS;

	if ((xstats_names == NULL) || (size < exp_size))
		return exp_size;

	for (i = 0; i < QEP_NB_CMAC_XSTATS64; i++, count++) {
		snprintf(xstats_names[count].name, sizeof(xstats_names[i].name),
			 "%s", rte_qep_cmac_stats_str64[i].name);
	}
	for (i = 0; i < QEP_NB_STMN_XSTATS32; i++, count++) {
		snprintf(xstats_names[count].name, sizeof(xstats_names[i].name),
			 "%s", rte_qep_stmn_stats_str32[i].name);
	}
	for (i = 0; i < QEP_NB_STMN_XSTATS64; i++, count++) {
		snprintf(xstats_names[count].name, sizeof(xstats_names[i].name),
			 "%s", rte_qep_stmn_stats_str64[i].name);
	}

	return count;
}

/**
 * DPDK callback to get names of extended stats of the device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param xstats_names
 *   An rte_eth_xstat_name array of at least size elements to be filled.
 *   If set to NULL, the function returns the required number of elements.
 * @param ids
 *   IDs array given by app to retrieve specific statistics
 * @param size
 *   Number of elements in the xstats_names array
 *
 * @return
 *   Success: A positive value lower or equal to size.
 *   The return value is the number of entries filled in the stats table.
 *
 *   Error: A positive value higher than size,
 *   the given statistics table is too small.
 *   The return value corresponds to the size that should be given to succeed.
 *   The entries in the table are not valid and shall not be used by the caller.
 *
 *   A negative value on other error
 */
static int
qep_dev_xstats_get_names_by_id(struct rte_eth_dev *dev,
	struct rte_eth_xstat_name *xstats_names, const uint64_t *ids,
	unsigned int size)
{
	uint32_t i = 0;
	uint32_t exp_size = QEP_NB_XSTATS;
	struct rte_eth_xstat_name xstats_names_cp[QEP_NB_XSTATS];

	if (!ids) {
		if (!xstats_names)
			return exp_size;
		else if (xstats_names && size < exp_size)
			return exp_size;
	}

	if (ids && !xstats_names) {
		PMD_DRV_LOG(ERR, "%s: Invalid xstats_names pointer\n",
			__func__);
		return -EINVAL;
	}

	qep_dev_xstats_get_names(dev, xstats_names_cp, exp_size);

	if (!ids) {
		for (i = 0; i < exp_size; i++) {
			strcpy(xstats_names[i].name,
				xstats_names_cp[i].name);
		}
	} else {
		for (i = 0; i < size; i++) {
			if (ids[i] >= exp_size) {
				PMD_DRV_LOG(ERR, "%s: id value isn't valid",
					__func__);
				return -EINVAL;
			}
			strcpy(xstats_names[i].name,
				xstats_names_cp[ids[i]].name);
		}
	}

	return i;
}

/**
 * DPDK callback to retrieve extended statistics of an Ethernet device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param xstats
 *   Pointer to the array of structure to be filled with
 *   device statistics ids and values
 * @param n
 *   Number of elements in the xstats array
 *
 * @return
 *   When n >= the number of stats,
 *   return the number of stat values filled into the array.
 *
 *   When n < the number of available stats,
 *   return the number of supported extended stats
 *   and do not fill in any data into xstats array.
 */
static int
qep_dev_xstats_get(__rte_unused struct rte_eth_dev *dev,
		struct rte_eth_xstat *xstats,
		unsigned int n)
{
	struct rte_qep_stmn_stats32 stats;
	struct qdma_pci_dev *dma_priv;
	struct stmn_stats stats64;
	struct rte_qep_cmac_stats cmac_stats;
	int count = 0, ret = 0;
	uint32_t i;
	uint32_t exp_size = QEP_NB_XSTATS;

	if ((xstats == NULL) || (n < exp_size))
		return exp_size;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;
	memset(&cmac_stats, 0, sizeof(cmac_stats));

	/* Set the Tick register so that statistics are
	 * captured in readable registers
	 */
	ret = xcmac_latch_statistics(&dma_priv->cmac_instance);
	if (ret != 0)
		PMD_DRV_LOG(ERR, "%s: Error in setting tick register\n",
				__func__);

	/* Get Rx Packet count statistics */
	ret = xcmac_get_rx_packet_statistics(&dma_priv->cmac_instance,
					&cmac_stats.rx_pkt_count);
	if (ret != 0) {
		PMD_DRV_LOG(ERR,
			"%s: Error in retrieving Rx packet count statistics\n",
			__func__);
	}

	/* Get Rx Packet types statistics */
	ret = xcmac_get_rx_packet_type_statistics(&dma_priv->cmac_instance,
					&cmac_stats.rx_pkt_types);
	if (ret != 0) {
		PMD_DRV_LOG(ERR,
			"%s: Error in retrieving Rx packet count statistics\n",
			__func__);
	}

	/* Get Rx malformed packet statistics */
	ret = xcmac_get_rx_malformed_statistics(&dma_priv->cmac_instance,
					&cmac_stats.rx_malformed);
	if (ret != 0) {
		PMD_DRV_LOG(ERR,
			"%s: Error in retrieving Rx Malformed statistics\n",
			__func__);
	}

	cmac_stats.rx_bad_code_pkts =
		xcmac_get_rx_bad_64b66b_code_count(&dma_priv->cmac_instance);
	cmac_stats.rx_range_error_pkts =
		xcmac_get_rx_range_error_count(&dma_priv->cmac_instance);
	cmac_stats.rx_truncated_pkts =
		xcmac_get_rx_truncated_count(&dma_priv->cmac_instance);

	/* Get Tx Packet count statistics */
	ret = xcmac_get_tx_packet_statistics(&dma_priv->cmac_instance,
					&cmac_stats.tx_pkt_count);
	if (ret != 0) {
		PMD_DRV_LOG(ERR,
			"%s: Error in retrieving Tx packet count statistics\n",
			__func__);
	}

	/* Get Tx Packet types statistics */
	ret = xcmac_get_tx_packet_type_statistics(&dma_priv->cmac_instance,
					&cmac_stats.tx_pkt_types);
	if (ret != 0) {
		PMD_DRV_LOG(ERR,
			"%s: Error in retrieving Tx packet count statistics\n",
			__func__);
	}

	/* Get Tx malformed packet statistics */
	ret = xcmac_get_tx_malformed_statistics(&dma_priv->cmac_instance,
					&cmac_stats.tx_malformed);
	if (ret != 0) {
		PMD_DRV_LOG(ERR,
			"%s: Error in retrieving Tx Malformed statistics\n",
			__func__);
	}

	cmac_stats.tx_frame_error_pkts +=
		xcmac_get_tx_frame_error_count(&dma_priv->cmac_instance);

	/* Get STM-N statistics */
	stmn_get_fifo_fill_level(dma_priv, &stats.fifo_fill);
	stmn_get_dsc_sts_min_max(dma_priv, &stats.dsc_sts);
	stmn_get_stats(dma_priv, &stats64);

	count = 0;
	for (i = 0; i < QEP_NB_CMAC_XSTATS64; i++) {
		xstats[count].value = *(uint64_t *)(((uint8_t *)&cmac_stats) +
				rte_qep_cmac_stats_str64[i].offset);
		xstats[count].id = count;
		count++;
	}
	for (i = 0; i < QEP_NB_STMN_XSTATS32; i++) {
		xstats[count].value = *(uint32_t *)(((uint8_t *)&stats) +
				rte_qep_stmn_stats_str32[i].offset);
		xstats[count].id = count;
		count++;
	}
	for (i = 0; i < QEP_NB_STMN_XSTATS64; i++) {
		xstats[count].value = *(uint64_t *)(((uint8_t *)&stats64) +
				rte_qep_stmn_stats_str64[i].offset);
		xstats[count].id = count;
		count++;
	}

	return count;
}

/**
 * DPDK callback to retrieve extended statistics of an Ethernet device by ID.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param ids
 *   A pointer to an ids array passed by application.
 *   This tells which statistics values function should retrieve.
 * @param values
 *   A pointer to a table to be filled with device statistics values
 * @param n
 *   The size of the ids array (number of elements)
 *
 * @return
 *   A positive value lower or equal to size: success.
 *     The return value is the number of entries filled in the stats table.
 *   A positive value higher than size: error,
 *     the given statistics table is too small.
 *     The return value corresponds to the size that should be given to succeed.
 *     The entries in the table are not valid
 *     and shall not be used by the caller.
 *   A negative value on error.
 */
static int
qep_dev_xstats_get_by_id(struct rte_eth_dev *dev,
				      const uint64_t *ids,
				      uint64_t *values,
				      unsigned int n)
{
	uint32_t i = 0;
	uint32_t exp_size = QEP_NB_XSTATS;
	struct rte_eth_xstat xstats[QEP_NB_XSTATS];

	if (!ids && !values) {
		PMD_DRV_LOG(ERR, "%s: Invalid ids and values pointer\n",
			__func__);
		return -EINVAL;
	}

	if (ids && !values) {
		PMD_DRV_LOG(ERR, "%s: Invalid values pointer\n",
			__func__);
		return -EINVAL;
	}

	if (!ids && n < exp_size)
		return exp_size;

	qep_dev_xstats_get(dev, xstats, exp_size);

	if (!ids) {
		for (i = 0; i < exp_size; i++)
			values[i] = xstats[i].value;
	} else {
		for (i = 0; i < n; i++) {
			if (ids[i] >= exp_size) {
				PMD_DRV_LOG(ERR, "%s: id value isn't valid",
					__func__);
				return -EINVAL;
			}
			values[i] = xstats[ids[i]].value;
		}
	}

	return i;
}

/**
 * DPDK callback to reset extended statistics of an Ethernet device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 *
 */
static void qep_dev_xstats_reset(struct rte_eth_dev *dev)
{
	struct qdma_pci_dev *dma_priv;
	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	xcmac_latch_statistics(&dma_priv->cmac_instance);
	stmn_snap_stats(dma_priv);
}

/**
 * DPDK callback to get Rx Queue info of an Ethernet device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param rx_queue_id
 *   The RX queue on the Ethernet device for which information will be
 *   retrieved
 * @param qinfo
 *   A pointer to a structure of type rte_eth_rxq_info_info to be filled with
 *   the information of given Rx queue.
 */
static void
qep_dev_rxq_info_get(struct rte_eth_dev *dev, uint16_t rx_queue_id,
		     struct rte_eth_rxq_info *qinfo)
{
	struct qdma_pci_dev *dma_priv;
	struct qdma_rx_queue *rxq = NULL;

	if (!qinfo)
		return;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	rxq = dev->data->rx_queues[rx_queue_id];
	memset(qinfo, 0, sizeof(struct rte_eth_rxq_info));
	qinfo->mp = rxq->mb_pool;
	qinfo->conf.offloads = rxq->offloads;
	qinfo->conf.rx_deferred_start = rxq->rx_deferred_start;
	qinfo->conf.rx_drop_en = 1;
	qinfo->conf.rx_thresh.wthresh = dma_priv->g_c2h_cnt_th[rxq->threshidx];
	qinfo->scattered_rx = 1;
	qinfo->nb_desc = rxq->nb_rx_desc - 1;
}

/**
 * DPDK callback to get Tx Queue info of an Ethernet device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param tx_queue_id
 *   The TX queue on the Ethernet device for which information will be
 *   retrieved
 * @param qinfo
 *   A pointer to a structure of type rte_eth_txq_info_info to be filled with
 *   the information of given Tx queue.
 */
static void
qep_dev_txq_info_get(struct rte_eth_dev *dev, uint16_t tx_queue_id,
		      struct rte_eth_txq_info *qinfo)
{
	struct qdma_tx_queue *txq = NULL;

	if (!qinfo)
		return;

	txq = dev->data->tx_queues[tx_queue_id];
	qinfo->conf.offloads = txq->offloads;
	qinfo->conf.tx_deferred_start = txq->tx_deferred_start;
	qinfo->conf.tx_rs_thresh = 0;
	qinfo->nb_desc = txq->nb_tx_desc - 1;

}

/**
 * DPDK callback to get packet types supported and identified by device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 *
 * @return
 *   Pointer to an array of type uint32_t indicating supported packet types.
 *   Returns NULL if called from secondary application.
 */
static const uint32_t *
qep_dev_supported_ptypes_get(struct rte_eth_dev *dev)
{
	static const uint32_t ptypes[] = {
		RTE_PTYPE_L2_ETHER_VLAN,
		RTE_PTYPE_L3_IPV4,
		RTE_PTYPE_L3_IPV4_EXT,
		RTE_PTYPE_L3_IPV6,
		RTE_PTYPE_L4_TCP,
		RTE_PTYPE_L4_UDP,
		RTE_PTYPE_L4_FRAG,
		RTE_PTYPE_UNKNOWN
	};

	if (dev->rx_pkt_burst == qdma_recv_pkts)
		return ptypes;

	PMD_DRV_LOG(ERR, "%s: ptypes unsupported\n",
		__func__);
	return NULL;
}

/**
 * DPDK callback to Link up an Ethernet device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 *
 * @return
 *    0 on success, negative errno value on failure.
 */


static int qep_dev_set_link_up(struct rte_eth_dev *dev)
{
	int ret;

	ret = xcmac_start(dev);
	if (ret != 0) {
		PMD_DRV_LOG(ERR, "%s: xcmac_start failed with status %d\n",
			    __func__, ret);
		return ret;
	}
	qdma_dev_link_update(dev, 1);

	return 0;

}

/**
 * DPDK callback to Link down an Ethernet device.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 *
 * @return
 *       0 on success, negative errno value on failure.
 */


static int qep_dev_set_link_down(struct rte_eth_dev *dev)
{
	int ret;
	struct qdma_pci_dev *dma_priv;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;
	ret = xcmac_stop(dev);

	if (ret != 0) {
		PMD_DRV_LOG(ERR, "%s: xcmac_stop failed with status %d\n",
			    __func__, ret);
		return ret;
	}
	/* TODO: Do we really need gt and core reset here. With IXIA, cmac_stop
	 * is not clearing lane status bit.
	 */
	xcmac_reset_gt(&dma_priv->cmac_instance);
	xcmac_reset_core(&dma_priv->cmac_instance, XCMAC_CORE_BOTH);
	qdma_dev_link_update(dev, 0);

	return 0;
}

/**
 * DPDK callback to retrieve RSS Indirection table.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param reta_conf
 *   An Array of structure used to retrieve entries of Redirection Table of
 *   the Receive Side Scaling (RSS).Each element of array can have 64
 *   entries.
 * @param reta_size
 *   Size of the RETA table
 *
 * @return
 *   0 on success, -EINVAL on failure.
 */

static int
qep_dev_rss_reta_query(struct rte_eth_dev *dev,
		       struct rte_eth_rss_reta_entry64 *reta_conf,
		       uint16_t reta_size)
{
	struct qdma_pci_dev *dma_priv;
	uint32_t idx;
	uint32_t i;
	uint32_t val, addr;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;
	if (!reta_size || reta_size > QEP_RSS_TBL_SIZE) {
		PMD_DRV_LOG(ERR, "%s: Invalid RETA table size %d\n",
			__func__, reta_size);
		rte_errno = EINVAL;
		return -rte_errno;
	}
	/* Fill each entry of the table even if its bit is not set. */
	for (idx = 0, i = 0; (i != reta_size); ++i) {
		idx = i / RTE_RETA_GROUP_SIZE;
		addr = (QEP_BASE_OFFSET + QEP_USER_RSS_OFFSET + (i * 4));
		val = qdma_read_reg((uint64_t)
				(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
				addr);
		reta_conf[idx].reta[i % RTE_RETA_GROUP_SIZE] = val;
	}
	return 0;
}

/**
 * DPDK callback to update the RSS indirection table.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param reta_conf
 *   An Array of structure used to configure entries of Redirection Table of
 *   the Receive Side Scaling (RSS).Each element of array can have 64
 *   entries. For each requested reta entry, corresponding bit in mask
 *   must be set.
 * @param reta_size
 *   Size of the RETA table.
 *
 * @return
 *   0 on success, -EINVAL on failure.
 */

int
qep_dev_rss_reta_update(struct rte_eth_dev *dev,
			 struct rte_eth_rss_reta_entry64 *reta_conf,
			 uint16_t reta_size)
{
	struct qdma_pci_dev *dma_priv;
	int idx, i, pos;
	uint32_t qval, nrxq;
	uint32_t addr;

	nrxq = dev->data->nb_rx_queues;
	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	/* Validate queue id received from user*/
	for (idx = 0, i = 0; (i != reta_size); ++i) {
		idx = i / RTE_RETA_GROUP_SIZE;
		pos = i % RTE_RETA_GROUP_SIZE;
		if (((reta_conf[idx].mask >> pos) & 0x1) == 0)
			continue;
		if (reta_conf[idx].reta[pos] >= nrxq) {
			PMD_DRV_LOG(ERR, "%s: Invalid qid %d "
				"specified in reta_conf\n",
				__func__, reta_conf[idx].reta[pos]);
			return -EINVAL;
		}
	}

	/* Program queue id received into Redirection table */
	for (idx = 0, i = 0; (i != reta_size); ++i) {
		idx = i / RTE_RETA_GROUP_SIZE;
		pos = i % RTE_RETA_GROUP_SIZE;
		if (((reta_conf[idx].mask >> pos) & 0x1) == 0)
			continue;
		addr = (QEP_BASE_OFFSET + QEP_USER_RSS_OFFSET + (i * 4));
		qval = reta_conf[idx].reta[pos];
		qdma_write_reg((uint64_t)
		       (dma_priv->bar_addr[dma_priv->user_bar_idx]) + addr,
		       qval);
	}
	return 0;
}

/**
 * DPDK callback to get the RSS hash configuration.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param rss_conf
 *   RSS configuration data to be stored.
 *
 * @return
 *   0 on success, -EINVAL on failure and rte_errno is set.
 */

int
qep_dev_rss_conf_get(struct rte_eth_dev *dev,
		     struct rte_eth_rss_conf *rss_conf)
{
	struct qdma_pci_dev *dma_priv;
	uint8_t rss_key[40] = {
		0x6D, 0x5A, 0x56, 0xDA, 0x25, 0x5B, 0x0E, 0xC2,
		0x41, 0x67, 0x25, 0x3D, 0x43, 0xA3, 0x8F, 0xB0,
		0xD0, 0xCA, 0x2B, 0xCB, 0xAE, 0x7B, 0x30, 0xB4,
		0x77, 0xCB, 0x2D, 0xA3, 0x80, 0x30, 0xF2, 0x0C,
		0x6A, 0x42, 0xB7, 0x3B, 0xBE, 0xAC, 0x01, 0xFA,
	};

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	(void)dma_priv;
	if (!rss_conf) {
		PMD_DRV_LOG(ERR, "%s: Invalid rss_conf pointer\n",
			__func__);
		rte_errno = EINVAL;
		return -rte_errno;
	}
	if (rss_conf->rss_key && (rss_conf->rss_key_len >= QEP_RSS_HKEY_SIZE))
		memcpy(rss_conf->rss_key, rss_key, QEP_RSS_HKEY_SIZE);
	rss_conf->rss_key_len = QEP_RSS_HKEY_SIZE;
	rss_conf->rss_hf = QEP_RSS_FLOW_TYPE;
	return 0;
}

/**
 * DPDK callback to get rte_flow callbacks and manage filter operations.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param filter_type
 *   Filter type.
 * @param filter_op
 *   Operation to perform.
 * @param arg
 *   Pointer to operation-specific structure.
 *
 * @return
 *   0 on success, -EINVAL on failure.
 */

static int
qep_filter_ctrl(struct rte_eth_dev *dev __rte_unused,
		enum rte_filter_type filter_type,
		enum rte_filter_op filter_op, void *arg)
{
	int ret = 0;

	switch (filter_type) {
	case RTE_ETH_FILTER_GENERIC:
		if (filter_op != RTE_ETH_FILTER_GET)
			return -EINVAL;
		*(const void **)arg = &qep_flow_ops;
		break;
	default:
		RTE_LOG(ERR, PMD,
			"Filter type (%d) not supported", filter_type);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static struct eth_dev_ops qdma_eth_dev_ops = {
	.dev_configure            = qdma_dev_configure,
	.dev_infos_get            = qdma_dev_infos_get,
	.dev_start                = qdma_dev_start,
	.dev_stop                 = qdma_dev_stop,
	.dev_close                = qdma_dev_close,
	.dev_reset                = qdma_dev_reset,
	.link_update              = qdma_dev_link_update,
	.dev_set_link_up          = qep_dev_set_link_up,
	.dev_set_link_down        = qep_dev_set_link_down,
	.rx_queue_setup           = qdma_dev_rx_queue_setup,
	.tx_queue_setup           = qdma_dev_tx_queue_setup,
	.rx_queue_release         = qdma_dev_rx_queue_release,
	.tx_queue_release         = qdma_dev_tx_queue_release,
	.rx_queue_start           = qdma_dev_rx_queue_start,
	.rx_queue_stop            = qdma_dev_rx_queue_stop,
	.tx_queue_start           = qdma_dev_tx_queue_start,
	.tx_queue_stop            = qdma_dev_tx_queue_stop,
	.rx_queue_count           = qdma_dev_rx_queue_count,
	.rx_descriptor_status     = qdma_dev_rx_descriptor_status,
	.tx_descriptor_status     = qdma_dev_tx_descriptor_status,
	.tx_done_cleanup          = qdma_dev_tx_done_cleanup,
	.queue_stats_mapping_set  = qdma_dev_queue_stats_mapping,
	.get_reg                  = qdma_dev_get_regs,
	.stats_get                = qdma_dev_stats_get,
	.stats_reset              = qdma_dev_stats_reset,
	.xstats_get_names         = qep_dev_xstats_get_names,
	.xstats_get_names_by_id   = qep_dev_xstats_get_names_by_id,
	.xstats_get               = qep_dev_xstats_get,
	.xstats_get_by_id         = qep_dev_xstats_get_by_id,
	.xstats_reset             = qep_dev_xstats_reset,
	.mtu_set                  = qdma_dev_mtu_set,
	.rxq_info_get             = qep_dev_rxq_info_get,
	.txq_info_get             = qep_dev_txq_info_get,
	.dev_supported_ptypes_get = qep_dev_supported_ptypes_get,
	.reta_query	              = qep_dev_rss_reta_query,
	.reta_update	          = qep_dev_rss_reta_update,
	.rss_hash_conf_get        = qep_dev_rss_conf_get,
	.filter_ctrl              = qep_filter_ctrl,
};

void qdma_dev_ops_init(struct rte_eth_dev *dev)
{
	dev->dev_ops = &qdma_eth_dev_ops;
	if (rte_eal_process_type() == RTE_PROC_PRIMARY) {
		dev->rx_pkt_burst = &qdma_recv_pkts;
		dev->tx_pkt_burst = &qdma_xmit_pkts;
	}
}
