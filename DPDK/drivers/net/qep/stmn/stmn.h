/*
 * Copyright(c) 2019 Xilinx, Inc. All rights reserved.
 *
 * BSD LICENSE
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

#ifndef __STMN_H__
#define __STMN_H__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/**************************************************************************/
#define STMN_SUCCESS 0
#define STMN_FAILURE 1
#define STMN_INVALID_ARGS 2
#define STMN_INVALID_IP 3
#define STMN_INVALID_PARAM 4

/* Design Specific Configurable*/
#define STMN_REG_NAME_LEN_MAX 100
#define STMN_MSG_BUF_LEN_MAX 8192

#define STMN_POLL_COUNT_MAX 1
#define STMN_POLL_INTERVAL 1

#define STMN_CRL_RAM_QID_BASE 64
#define STMN_CRL_RAM_NUM_QID 5

#define STMN_BAR_NUM 2
#define STMN_BAR_OFFSET 0x000D0000

#define STMN_EN_MEM_DUMP 0

/****************************************************************************/
/* Need Platform dependent implementation for Linux and DPDK */
void stmn_reg_write(void *dev_hndl, uint32_t reg_offst, uint32_t val);
uint32_t stmn_reg_read(void *dev_hndl, uint32_t reg_offst);

/****************************************************************************/
struct stmn_dev {
	unsigned short bar_num;
	void *stm_regs;
	void *xdev;
};
int stmn_initialize(void *dev_hndl, struct stmn_dev *stmn,
		    unsigned short dev_id);
int stmn_deinitialize(void *dev_hndl, struct stmn_dev *stmn);
int stmn_config(void *dev_hndl, int flags);
int stmn_print_msg(void *dev_hndl, char *msg, uint32_t len);
int stmn_print_stats(void *dev_hndl, char *buf, int len);
int stmn_print_debug(void *dev_hndl, char *msg, uint32_t len);

/****************************************************************************/
/* Version for validating IP */
struct stmn_ip_version {
	uint8_t minor;
	uint8_t major;
	uint16_t block_id;
};
int stmn_present_check(void *dev_hndl, unsigned short pcidev_id);
int stmn_get_version(void *dev_hndl, struct stmn_ip_version *ver);
int stmn_validate_version(struct stmn_ip_version *ver);
int stmn_get_c2h_buf_size(void *dev_hndl, uint32_t *buf_size_idx);
int stmn_set_c2h_buf_size(void *dev_hndl, uint32_t buf_size_idx);
int stmn_write_c2h_buf_size(void *dev_hndl, uint16_t buf_size);
/*************************** Stats ******************************************/

struct stmn_fifo_fill_level {
	uint32_t c2h_cmpt;
	uint32_t qdma_c2h_sts;
	uint32_t h2c_meta;
	uint32_t c2h_dsc_sts;
	uint32_t h2c_dsc_sts;
	uint32_t c2h_crdt;
	uint32_t h2c_crdt;
	uint32_t c2h_byp_out;
	uint32_t h2c_byp_out;
};

struct stmn_dsc_sts_min_max {
	uint32_t c2h_min_avl;
	uint32_t c2h_max_avl;
	uint32_t h2c_min_avl;
	uint32_t h2c_max_avl;
};

struct stmn_axis_slave_stats {
	uint64_t s_axis_pkt_in_cnt;
	uint64_t s_axis_pkt_accept_cnt;
	uint64_t s_axis_byte_accept_cnt;
	uint64_t s_axis_pkt_drop_cnt;
	uint64_t s_axis_byte_drop_cnt;
	uint64_t qdma_pkt_accept_cnt;
	uint64_t qdma_byte_accept_cnt;
	uint64_t qdma_pkt_drop_cnt;
	uint64_t qdma_byte_drop_cnt;
	uint64_t s_axis_active;
	uint64_t s_axis_idle;
	uint64_t s_axis_pause;
	uint64_t qdma_axis_c2h_active;
	uint64_t qdma_axis_c2h_idle;
	uint64_t qdma_axis_c2h_pause;
	uint64_t qdma_axis_c2h_cmpt_active;
	uint64_t qdma_axis_c2h_cmpt_idle;
	uint64_t qdma_axis_c2h_cmpt_pause;
	uint64_t qdma_axis_c2h_dmawr_cmp_cnt;
	uint64_t c2h_cmpt_fifo_avg_cnt;
	uint64_t qdma_c2h_sts_fifo_avg_cnt;
};

struct stmn_axis_master_stats {
	uint64_t m_axis_pkt_cnt;
	uint64_t m_axis_byte_cnt;
	uint64_t m_axis_active;
	uint64_t m_axis_idle;
	uint64_t m_axis_pause;
	uint64_t h2c_meta_fifo_avg_cnt;
};

struct stmn_dsc_stats {
	uint64_t dsc_sts_c2h_cnt;
	uint64_t dsc_sts_h2c_cnt;
	uint64_t dsc_sts_c2h_avl_cnt;
	uint64_t dsc_sts_h2c_avl_cnt;
	uint64_t dsc_sts_c2h_fifo_avg_cnt;
	uint64_t dsc_sts_h2c_fifo_avg_cnt;
};

struct stmn_crdt_bypout_stats {
	uint64_t crdt_in_c2h_vld_cnt;
	uint64_t crdt_in_h2c_vld_cnt;
	uint64_t crdt_in_c2h_cnt;
	uint64_t crdt_in_h2c_cnt;
	uint64_t byp_out_c2h_cnt;
	uint64_t byp_out_h2c_cnt;
	uint64_t c2h_crdt_fifo_avg_cnt;
	uint64_t h2c_crdt_fifo_avg_cnt;
	uint64_t c2h_byp_out_fifo_avg_cnt;
	uint64_t h2c_byp_out_fifo_avg_cnt;
};

struct stmn_stats {
	uint64_t cycle_cnt;
	struct stmn_axis_slave_stats s_axis;
	struct stmn_axis_master_stats m_axis;
	struct stmn_dsc_stats desc;
	struct stmn_crdt_bypout_stats crdt_bypout;
};

int stmn_get_fifo_fill_level(void *dev_hndl,
			     struct stmn_fifo_fill_level *stats);
int stmn_get_dsc_sts_min_max(void *dev_hndl,
			     struct stmn_dsc_sts_min_max *stats);
int stmn_get_stats(void *dev_hndl, struct stmn_stats *stats);
int stmn_print_stats_msg(void *dev_hndl, char *buf, int len);
int stmn_print_fifo_level_msg(void *dev_hndl, char *buf, int len);
int stmn_print_dsc_minmax_msg(void *dev_hndl, char *buf, int len);

/*****************************Error****************************************/
struct stmn_fatal_err_status {
	uint32_t s_axis_err;
	uint32_t m_axis_err;
	uint32_t dsc_sts_err;
	uint32_t crdt_bypout_err;
	uint32_t c2h_dsc_sts_err;
	uint32_t c2h_dsc_str_wr_err;
	uint32_t c2h_dsc_str_rd_err;
	uint32_t h2c_dsc_sts_err;
	uint32_t h2c_dsc_str_wr_err;
	uint32_t h2c_dsc_str_rd_err;
};

struct stmn_dsc_err_info {
	uint32_t err : 1;
	uint32_t qinv : 1;
	uint32_t irq_arm : 1;
	uint32_t mm : 1;
	uint32_t byp : 1;
	uint32_t qen : 1;
	uint32_t reserved1 : 2;
	uint32_t port_id : 3;
	uint32_t reserved2 : 5;
	uint32_t avl : 16;
	uint32_t qid;
};

struct stmn_bypass_err_info {
	uint32_t error : 1;
	uint32_t st_mm : 1;
	uint32_t mrkr_rsp : 1;
	uint32_t dsc_sz : 2;
	uint32_t port_id : 3;
	uint32_t func : 8;
	uint32_t cidx : 16;
	uint32_t qid;
};

struct stmn_err_capture {
	struct stmn_dsc_err_info c2h_dsc;
	struct stmn_dsc_err_info h2c_dsc;
	struct stmn_bypass_err_info c2h_byp_out;
	struct stmn_bypass_err_info h2c_byp_out;
};

int stmn_get_fatal_err_info(void *dev_hndl,
			    struct stmn_fatal_err_status *err_info);
int stmn_get_err_capture_info(void *dev_hndl,
			      struct stmn_err_capture *err_info);
int stmn_print_error_msg(void *dev_hndl, char *buf, int len);

/***************************Status abd debug**********************************/
struct stmn_idle_status {
	uint32_t s_axis;
	uint32_t m_axis;
	uint32_t dsc_sts;
	uint32_t crdt_bypout;
	uint32_t c2h_dsc_sts;
	uint32_t c2h_dsc_str_wr;
	uint32_t c2h_dsc_str_rd;
	uint32_t h2c_dsc_sts;
	uint32_t h2c_dsc_str_wr;
	uint32_t h2c_dsc_str_rd;
};

struct stmn_axis_resp {
	uint32_t drop_code : 4;
	uint32_t num_desc : 4;
	uint32_t num_pkt : 4;
	uint32_t accept : 1;
	uint32_t sim_byp_mode : 1;
	uint32_t resvd : 2;
	uint32_t qid : 11;
	uint32_t resvd2 : 4;
	uint32_t reg_val : 1;
};

struct stmn_axis_resp_flags {
	uint32_t accept_sim_byp : 1;
	uint32_t accept_not_byp : 1;
	uint32_t drop_cache_byp : 1;
	uint32_t drop_sim_byp : 1;
	uint32_t drop_not_byp : 1;
	uint32_t drop_sim_byp_len : 1;
	uint32_t drop_sim_byp_avl : 1;
	uint32_t drop_sim_byp_init : 1;
	uint32_t drop_qid_err : 1;
	uint32_t drop_qid_not_st : 1;
	uint32_t drop_qid_not_en : 1;
	uint32_t resvd : 21;
};

struct stmn_qdma_c2h_sts_flags {
	uint32_t last : 1;
	uint32_t cmp : 1;
	uint32_t not_last : 1;
	uint32_t drop : 1;
	uint32_t rsvd : 12;
	uint32_t max_qid : 11;
	uint32_t rsvd2 : 5;
};

struct stmn_axis_debug {
	struct stmn_axis_resp last_rsp;
	struct stmn_axis_resp first_drop;
	struct stmn_axis_resp_flags axis_flgs;
	struct stmn_qdma_c2h_sts_flags c2h_flags;
};

struct stmn_sts_ram_val {
	uint32_t qen : 1;
	uint32_t byp : 1;
	uint32_t init : 1;
	uint32_t irq_arm : 1;
	uint32_t rst : 1;
	uint32_t err : 1;
	uint32_t mm : 1;
	uint32_t rsvd : 1;
	uint32_t sts_avl : 8;
	uint32_t dsc_avl : 16;
};

struct stmn_store_ram_val {
	uint32_t en : 1;
	uint32_t err : 1;
	uint32_t rst : 1;
	uint32_t rsvd : 5;
	uint32_t in_arb : 1;
	uint32_t rsvd1 : 7;
	uint32_t rd_ptr : 8;
	uint32_t wr_ptr : 8;
};

struct stmn_ctrl_ram_status {
	struct stmn_sts_ram_val c2h_dsc;
	struct stmn_sts_ram_val h2c_dsc;
	struct stmn_store_ram_val c2h_wr;
	struct stmn_store_ram_val c2h_rd;
	struct stmn_store_ram_val h2c_wr;
	struct stmn_store_ram_val h2c_rd;
};

int stmn_get_idle_status(void *dev_hndl, struct stmn_idle_status *idle);
int stmn_get_axis_debug_status(void *dev_hndl, struct stmn_axis_debug *debug);
int stmn_get_ctrl_ram_status(void *dev_hndl, uint16_t qid,
			     struct stmn_ctrl_ram_status *ram_info);
int stmn_print_ram_status_msg(void *dev_hndl, char *buf, int len);
int stmn_print_status_msg(void *dev_hndl, char *buf, int len);

enum stmn_reg_group {
	stmn_rg_idle_status = 0,
	stmn_rg_s_axis_dbg,
	stmn_rg_ctrl_ram_status,
	stmn_rg_fatal_err,
	stmn_rg_err_capture,
	stmn_rg_fifo_fill_level,
	stmn_rg_dsc_sts_minmax,
	stmn_rg_stats,
	stmn_rg_max
};

struct stmn_reg_offset {
	uint32_t idle_status;
	uint32_t s_axis_dbg;
	uint32_t ctrl_ram_status;
	uint32_t fatal_err;
	uint32_t err_capture;
	uint32_t fifo_fill_level;
	uint32_t dsc_sts_minmax;
	uint32_t stats;
};

extern const char stmn_reg_name_fatal_err[][STMN_REG_NAME_LEN_MAX];
extern const char stmn_reg_name_idle_status[][STMN_REG_NAME_LEN_MAX];
extern const char stmn_reg_name_dscsts_minmax[][STMN_REG_NAME_LEN_MAX];
extern const char stmn_reg_name_fifo_fill[][STMN_REG_NAME_LEN_MAX];
extern const char *stmn_reg_name_stats[STMN_REG_NAME_LEN_MAX];
extern const struct stmn_reg_offset stmn_regs;
#endif
