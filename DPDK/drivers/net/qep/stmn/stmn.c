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

#include "stmn.h"

#include <stddef.h>

#ifdef __KERNEL__
#include <linux/delay.h>
#include <linux/string.h>
static unsigned int sleep(unsigned int msec)
{
	mdelay(msec);
	return 0;
}
#else
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#define dummy_p(...)
#define pr_err printf
#define pr_debug dummy_p
#endif

/*
 * TODO  Add more documentation as per kernel guidelines
 * TODO  Reuse bit manipulation macro from reg.h of common lib
 *
 */

/**************************************************************************/
/** Boiler Plate Macros for null and failure check, code reuse and readability*/
#define stmn_null_check(arg, msg)                                              \
	do {                                                                   \
		if ((arg) == NULL) {                                           \
			pr_err("%s : %s null pointer! \r\n", __func__, (msg)); \
			return -STMN_INVALID_ARGS;                             \
		}                                                              \
	} while (0)

#define stmn_fail_check(arg, msg)                                              \
	do {                                                                   \
		if ((arg) < STMN_SUCCESS) {                                    \
			pr_err("%s : %s Failed! Error Code: %d \r\n",          \
			       __func__, (msg), arg);                          \
			return arg;                                            \
		}                                                              \
	} while (0)

/************************************************************************/
#define STMN_REGS_VERSION 0x00000000
#define STMN_BLOCK_ID 0x0
#define STMN_MAJOR_VERSION 0x0
#define STMN_MINOR_VERSION 0x1

#define STMN_BLOCK_ID_SHIFT 16
#define STMN_MAJOR_VERSION_SHIFT 8
#define STMN_MINOR_VERSION_SHIFT 0

#define STMN_REGS_C2H_CFG_BUF_SIZE 0x00000080
#define STMN_REGS_C2H_CFG_BUF_SIZE_MASK 0x11

#define STMN_RAM_READ_QID_MASK 0x7fff
#define STMN_RAM_READ_VLD_BIT (1 << 16)
#define STMN_RAM_READ_DONE_BIT (1 << 17)

#define STMN_REG_SNAP 0x00000300
#define STMN_REG_SNAP_MASK 0x00000100
#define STMN_C2H_BUF_2048 2048
#define STMN_C2H_BUF_4096 4096
#define STMN_C2H_BUF_8192 8192
#define STMN_C2H_BUF_16384 16384

static const unsigned short stmn_enabled_device_id[] = { 0x903f, 0x6aa0,
							 0x7002 };

const struct stmn_reg_offset stmn_regs = { 0x00000100, 0x00000140, 0x00000180,
					   0x00000200, 0x00000280, 0x00000310,
					   0x00000360, 0x00000400 };

const char *stmn_reg_name_stats[STMN_REG_NAME_LEN_MAX] = {
	"cycle_count",
	"s_axis_pkt_in_cnt",
	"s_axis_pkt_accept_cnt",
	"s_axis_byte_accept_cnt",
	"s_axis_pkt_drop_cnt",
	"s_axis_byte_drop_cnt",
	"qdma_pkt_accept_cnt",
	"qdma_byte_accept_cnt",
	"qdma_pkt_drop_cnt",
	"qdma_byte_drop_cnt",
	"s_axis_active",
	"s_axis_idle",
	"s_axis_pause",
	"qdma_axis_c2h_active",
	"qdma_axis_c2h_idle",
	"qdma_axis_c2h_pause",
	"qdma_axis_c2h_cmpt_active",
	"qdma_axis_c2h_cmpt_idle",
	"qdma_axis_c2h_cmpt_pause",
	"qdma_axis_c2h_dmawr_cmp_cnt",
	"c2h_cmpt_fifo_avg_cnt",
	"qdma_c2h_sts_fifo_avg_cnt",
	"m_axis_pkt_cnt",
	"m_axis_byte_cnt",
	"m_axis_active",
	"m_axis_idle",
	"m_axis_pause",
	"h2c_meta_fifo_avg_cnt",
	"dsc_sts_c2h_cnt",
	"dsc_sts_h2c_cnt",
	"dsc_sts_c2h_avl_cnt",
	"dsc_sts_h2c_avl_cnt",
	"dsc_sts_c2h_fifo_avg_cnt",
	"dsc_sts_h2c_fifo_avg_cnt",
	"crdt_in_c2h_vld_cnt",
	"crdt_in_h2c_vld_cnt",
	"crdt_in_c2h_cnt",
	"crdt_in_h2c_cnt",
	"byp_out_c2h_cnt",
	"byp_out_h2c_cnt",
	"c2h_crdt_fifo_avg_cnt",
	"h2c_crdt_fifo_avg_cnt",
	"c2h_byp_out_fifo_avg_cnt",
	"h2c_byp_out_fifo_avg_cnt",
};

const char stmn_reg_name_fifo_fill[][STMN_REG_NAME_LEN_MAX] = {
	"c2h_cmpt", "qdma_c2h_sts", "h2c_meta",    "c2h_dsc_sts", "h2c_dsc_sts",
	"c2h_crdt", "h2c_crdt",     "c2h_byp_out", "h2c_byp_out",
};

const char stmn_reg_name_dscsts_minmax[][STMN_REG_NAME_LEN_MAX] = {
	"c2h_min_avl",
	"c2h_max_avl",
	"h2c_min_avl",
	"h2c_max_avl",
};

const char stmn_reg_name_idle_status[][STMN_REG_NAME_LEN_MAX] = {
	"s_axis",	 "m_axis",	 "dsc_sts",	"crdt_bypout",
	"c2h_dsc_sts",    "c2h_dsc_str_wr", "c2h_dsc_str_rd", "h2c_dsc_sts",
	"h2c_dsc_str_wr", "h2c_dsc_str_rd "
};

const char stmn_reg_name_fatal_err[][STMN_REG_NAME_LEN_MAX] = {
	"s_axis_err",	 "m_axis_err",	 "dsc_sts_err",
	"crdt_bypout_err",    "c2h_dsc_sts_err",    "c2h_dsc_str_wr_err",
	"c2h_dsc_str_rd_err", "h2c_dsc_sts_err   ", "h2c_dsc_str_wr_err",
	"h2c_dsc_str_rd_err",
};

/******************************************************************************/
int stmn_get_version(void *dev_hndl, struct stmn_ip_version *ver)
{
	uint32_t reg_val;

	stmn_null_check(dev_hndl, "dev_hndl");
	stmn_null_check(ver, "stmn_ip_version");

	reg_val = stmn_reg_read(dev_hndl, STMN_REGS_VERSION);
	ver->minor = (uint8_t)(reg_val >> STMN_MINOR_VERSION_SHIFT);
	ver->major = (uint8_t)(reg_val >> STMN_MAJOR_VERSION_SHIFT);
	ver->block_id = (uint16_t)(reg_val >> STMN_BLOCK_ID_SHIFT);

	pr_debug("%s Reg:%x Minor:%d Major:%d BlcokID:%x", __func__, reg_val,
		 ver->minor, ver->major, ver->block_id);
	return STMN_SUCCESS;
}

int stmn_validate_version(struct stmn_ip_version *ver)
{
	stmn_null_check(ver, "stmn_ip_version");

	if (ver->minor != STMN_MINOR_VERSION) {
		pr_err("%s : minor version mismatch", __func__);
		return -STMN_INVALID_IP;
	}

	if (ver->major != STMN_MAJOR_VERSION) {
		pr_err("%s : major version mismatch", __func__);
		return -STMN_INVALID_IP;
	}

	if (ver->block_id != STMN_BLOCK_ID) {
		pr_err("%s : block ID version mismatch", __func__);
		return -STMN_INVALID_IP;
	}

	return STMN_SUCCESS;
}

int stmn_present_check(void *dev_hndl, unsigned short pcidev_id)
{
	int i;
	int len = sizeof(stmn_enabled_device_id) /
		  sizeof(stmn_enabled_device_id[0]);

	stmn_null_check(dev_hndl, "dev_hndl");

	for (i = 0; i < len; i++)
		if (pcidev_id == stmn_enabled_device_id[i])
			break;

	if (i == len) {
		pr_err("PCIe Device does Not support STMN\n");
		return -STMN_FAILURE;
	}

	return 0;
}

int stmn_get_c2h_buf_size(void *dev_hndl, uint32_t *buf_size_idx)
{
	uint32_t reg_val;

	stmn_null_check(dev_hndl, "dev_hndl");
	stmn_null_check(buf_size_idx, "buf_size");

	reg_val = stmn_reg_read(dev_hndl, STMN_REGS_C2H_CFG_BUF_SIZE);
	reg_val = reg_val & STMN_REGS_C2H_CFG_BUF_SIZE_MASK;

	*buf_size_idx = reg_val;

	return STMN_SUCCESS;
}

int stmn_set_c2h_buf_size(void *dev_hndl, uint32_t buf_size_idx)
{
	uint32_t reg_val;

	stmn_null_check(dev_hndl, "dev_hndl");

	reg_val = stmn_reg_read(dev_hndl, STMN_REGS_C2H_CFG_BUF_SIZE);
	reg_val &= ~STMN_REGS_C2H_CFG_BUF_SIZE_MASK;
	reg_val |= (buf_size_idx & STMN_REGS_C2H_CFG_BUF_SIZE_MASK);
	stmn_reg_write(dev_hndl, STMN_REGS_C2H_CFG_BUF_SIZE, reg_val);

	pr_debug(" Stmn: C2H buf size %d reg_setting %x ", buf_size_idx,
		 reg_val);

	return STMN_SUCCESS;
}

int stmn_write_c2h_buf_size(void *dev_hndl, uint16_t buf_size)
{
	int buf_size_idx = -1;
	int ret = STMN_FAILURE;

	switch (buf_size) {
	case STMN_C2H_BUF_2048:
		buf_size_idx = 0;
		break;
	case STMN_C2H_BUF_4096:
		buf_size_idx = 1;
		break;
	case STMN_C2H_BUF_8192:
		buf_size_idx = 2;
		break;
	case STMN_C2H_BUF_16384:
		buf_size_idx = 3;
		break;
	default:
		buf_size_idx = -1;
		break;
	}
	if (buf_size_idx != -1)
		ret = stmn_set_c2h_buf_size(dev_hndl, buf_size_idx);

	return ret;
}

int stmn_config(void *dev_hndl, int flags)
{
	stmn_null_check(dev_hndl, "dev_hndl");

	flags = 0;
	stmn_set_c2h_buf_size(dev_hndl, flags);

	return STMN_SUCCESS;
}

int stmn_initialize(void *dev_hndl, struct stmn_dev *stmn,
		    unsigned short dev_id)
{
	int ret;
	struct stmn_ip_version ver;

	stmn_null_check(dev_hndl, "dev_hndl");
	stmn_null_check(stmn, "stmn");
	stmn_null_check(stmn->stm_regs, "stmn->stm_regs");

	ret = stmn_present_check(stmn, dev_id);
	if (ret < 0) {
		stmn->stm_regs = NULL;
		return -STMN_FAILURE;
	}
	ret = stmn_get_version(dev_hndl, &ver);
	stmn_fail_check(ret, "stmn_get_version");

	ret = stmn_validate_version(&ver);
	if (ret == -STMN_INVALID_IP)
		pr_err("IP HW version and Software version mismatch\n");

	stmn->xdev = dev_hndl;

	return ret;
}

int stmn_deinitialize(void *dev_hndl, struct stmn_dev *stmn)
{
	stmn_null_check(dev_hndl, "dev_hndl");
	stmn_null_check(stmn, "stmn");
	stmn_null_check(stmn->stm_regs, "stmn->regs");

	stmn->stm_regs = NULL;
	stmn->xdev = NULL;

	return 0;
}
/******************************************************************************/
static int stmn_snap_stats(void *dev_hndl)
{
	uint32_t reg, counter = 0;
#ifdef STMN_TEST_US
	return STMN_SUCCESS;
#endif
	reg = stmn_reg_read(dev_hndl, STMN_REG_SNAP);
	reg |= 0x1;
	stmn_reg_write(dev_hndl, STMN_REG_SNAP, reg);

	do {
		reg = stmn_reg_read(dev_hndl, STMN_REG_SNAP);
		if ((reg & STMN_REG_SNAP_MASK) || counter > STMN_POLL_COUNT_MAX)
			break;
		pr_debug("%s : Reg:0x%x Counter:%d\n", __func__, reg, counter);
		counter++;
		sleep(STMN_POLL_INTERVAL);
	} while (1);

	if (counter > STMN_POLL_COUNT_MAX)
		return -STMN_FAILURE;

	return STMN_SUCCESS;
}

int stmn_get_stats(void *dev_hndl, struct stmn_stats *stats)
{
	int ret;
	uint32_t i;
	uint32_t *val = (uint32_t *)stats;
	uint32_t num_reg = sizeof(struct stmn_stats) / sizeof(uint32_t);

	stmn_null_check(dev_hndl, "Invalid dev_hndl");
	stmn_null_check(stats, "Invalid stmn_stats param");

	ret = stmn_snap_stats(dev_hndl);
	if (ret < 0) {
		pr_err("Polling failed for stats snap done\n");
		return -STMN_INVALID_ARGS;
	}
	for (i = 0; i < num_reg; i++)
		*(val + i) = stmn_reg_read(dev_hndl, stmn_regs.stats + (4 * i));

	return STMN_SUCCESS;
}

int stmn_get_fifo_fill_level(void *dev_hndl, struct stmn_fifo_fill_level *stats)
{
	uint32_t *val = (uint32_t *)stats;
	uint32_t i;
	uint32_t num_reg =
		sizeof(struct stmn_fifo_fill_level) / sizeof(uint32_t);

	stmn_null_check(dev_hndl, "Invalid dev_hndl");
	stmn_null_check(stats, "Invalid stmn_fifo_fill_level param");

	for (i = 0; i < num_reg; i++)
		*(val + i) = stmn_reg_read(dev_hndl,
					   stmn_regs.fifo_fill_level + (4 * i));

	return STMN_SUCCESS;
}
int stmn_get_dsc_sts_min_max(void *dev_hndl, struct stmn_dsc_sts_min_max *stats)
{
	int i;
	int num_reg = sizeof(struct stmn_dsc_sts_min_max) / sizeof(uint32_t);
	uint32_t *val = (uint32_t *)stats;

	stmn_null_check(dev_hndl, "Invalid dev_hndl");
	stmn_null_check(stats, "Invalid stmn_dsc_sts_min_max param");

	for (i = 0; i < num_reg; i++) {
		*(val + i) = stmn_reg_read(dev_hndl,
					   stmn_regs.dsc_sts_minmax + (4 * i));
	}
	return STMN_SUCCESS;
}
enum stmn_stats_group {
	STMN_STATS_AXIS_SLAVE = 0,
	STMN_STATS_AXIS_MASTER,
	STMN_STATS_DESC_STATS,
	STMN_STATS_CREDT_BYPOUT
} stmn_stats_group;

static void stmn_print_stats_group(struct stmn_stats *stats,
				   enum stmn_stats_group subgroup, char *buf,
				   int len)
{
	uint32_t i, num_entry, offset;
	uint64_t *stats_ptr = (uint64_t *)stats;
	char rg[STMN_REG_NAME_LEN_MAX];

	switch (subgroup) {
	case STMN_STATS_AXIS_SLAVE:
		num_entry =
			sizeof(struct stmn_axis_slave_stats) / sizeof(uint64_t);
		offset = offsetof(struct stmn_stats, s_axis) / sizeof(uint64_t);
		strcpy(rg, "STMN_STATS_AXIS_SLAVE\n");
		break;
	case STMN_STATS_AXIS_MASTER:
		num_entry = sizeof(struct stmn_axis_master_stats) /
			    sizeof(uint64_t);
		offset = offsetof(struct stmn_stats, m_axis) / sizeof(uint64_t);
		strcpy(rg, "STMN_STATS_AXIS_MASTER\n");
		break;
	case STMN_STATS_DESC_STATS:
		num_entry = sizeof(struct stmn_dsc_stats) / sizeof(uint64_t);
		offset = offsetof(struct stmn_stats, desc) / sizeof(uint64_t);
		strcpy(rg, "STMN_STATS_DESC_STATS\n");
		break;
	case STMN_STATS_CREDT_BYPOUT:
		num_entry = sizeof(struct stmn_crdt_bypout_stats) /
			    sizeof(uint64_t);
		offset = offsetof(struct stmn_stats, crdt_bypout) /
			 sizeof(uint64_t);
		strcpy(rg, "STMN_STATS_CREDT_BYPOUT\n");
		break;
	default:
		num_entry = 0;
		offset = 0;
		strcpy(rg, "invalid read\n");
		break;
	}

	snprintf(buf + strlen(buf), len - strlen(buf), "%s", rg);
	for (i = 0; i < num_entry; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			 "  %-30s 0x%lx\n", stmn_reg_name_stats[offset + i],
			 *(stats_ptr + offset + i));
}

int stmn_print_stats_msg(void *dev_hndl, char *buf, int len)
{
	int ret;
	struct stmn_stats stats;

	stmn_null_check(dev_hndl, "dev_hndl");
	stmn_null_check(buf, "buf");

	ret = stmn_get_stats(dev_hndl, &stats);
	stmn_fail_check(ret, "stmn_get_stats");
	stmn_print_stats_group(&stats, STMN_STATS_AXIS_SLAVE, buf, len);
	stmn_print_stats_group(&stats, STMN_STATS_AXIS_MASTER, buf, len);
	stmn_print_stats_group(&stats, STMN_STATS_DESC_STATS, buf, len);
	stmn_print_stats_group(&stats, STMN_STATS_CREDT_BYPOUT, buf, len);

	return STMN_SUCCESS;
}

int stmn_print_fifo_level_msg(void *dev_hndl, char *buf, int len)
{
	int i, ret, num_entry;
	uint32_t *reg_val;
	struct stmn_fifo_fill_level fifo;

	stmn_null_check(dev_hndl, "dev_hndl");
	stmn_null_check(buf, "buf");

	ret = stmn_get_fifo_fill_level(dev_hndl, &fifo);
	stmn_fail_check(ret, "stmn_get_fifo_fill_level");

	reg_val = (uint32_t *)&fifo;
	num_entry = sizeof(struct stmn_fifo_fill_level) / sizeof(uint32_t);
	snprintf(buf + strlen(buf), len - strlen(buf), "%s",
		 "STMN_FIFO_FILL_LEVEL\n");
	for (i = 0; i < num_entry; i++)
		snprintf(buf + strlen(buf), len - strlen(buf), "  %-30s 0x%x\n",
			 stmn_reg_name_fifo_fill[i], *(reg_val + i));
	return STMN_SUCCESS;
}
int stmn_print_dsc_minmax_msg(void *dev_hndl, char *buf, int len)
{
	int i, ret, num_entry;
	uint32_t *reg_val;
	struct stmn_dsc_sts_min_max desc_minmax;

	stmn_null_check(dev_hndl, "dev_hndl");
	stmn_null_check(buf, "buf");

	ret = stmn_get_dsc_sts_min_max(dev_hndl, &desc_minmax);
	stmn_fail_check(ret, "stmn_get_dsc_sts_min_max");
	reg_val = (uint32_t *)&desc_minmax;
	num_entry = sizeof(struct stmn_dsc_sts_min_max) / sizeof(uint32_t);
	snprintf(buf + strlen(buf), len - strlen(buf), "%s",
		 "STMN_DSC_MIN_MAX\n");
	for (i = 0; i < num_entry; i++)
		snprintf(buf + strlen(buf), len - strlen(buf), "  %-30s 0x%x\n",
			 stmn_reg_name_dscsts_minmax[i], *(reg_val + i));
	return STMN_SUCCESS;
}
/******************************************************************************/
int stmn_get_fatal_err_info(void *dev_hndl,
			    struct stmn_fatal_err_status *err_info)
{
	uint32_t i;
	uint32_t *val = (uint32_t *)err_info;
	uint32_t num_reg =
		sizeof(struct stmn_fatal_err_status) / sizeof(uint32_t);

	stmn_null_check(dev_hndl, "Invalid dev_hndl");
	stmn_null_check(err_info, "Invalid stmn_fatal_err_status param");

	for (i = 0; i < num_reg; i++)
		*(val + i) =
			stmn_reg_read(dev_hndl, stmn_regs.fatal_err + (4 * i));

	return STMN_SUCCESS;
}

int stmn_get_err_capture_info(void *dev_hndl, struct stmn_err_capture *err_info)
{
	uint32_t i;
	uint32_t *val = (uint32_t *)err_info;
	uint32_t num_reg = sizeof(struct stmn_err_capture) / sizeof(uint32_t);

	stmn_null_check(dev_hndl, "Invalid dev_hndl");
	stmn_null_check(err_info, "Invalid stmn_err_capture param");

	for (i = 0; i < num_reg; i++)
		*(val + i) = stmn_reg_read(dev_hndl,
					   stmn_regs.err_capture + (4 * i));

	return STMN_SUCCESS;
}

static void stmn_prn_dsc_err(struct stmn_dsc_err_info *dsc, char *buf, int len,
			     const char *msg)
{
	snprintf(buf + strlen(buf), len - strlen(buf),
		 "%s err:%d  qinv:%d  irq_arm:%d  mm:%d  byp:%d qen:%d  port_id:%d  avl:%d  qid:%d\n",
		 msg, dsc->err, dsc->qinv, dsc->irq_arm, dsc->mm, dsc->byp,
		 dsc->qen, dsc->port_id, dsc->avl, dsc->qid);
}

static void stmn_prn_byp_err(struct stmn_bypass_err_info *byp, char *buf,
			     int len, const char *msg)
{
	snprintf(buf + strlen(buf), len - strlen(buf),
		 "%s error:%d  st_mm:%d mrkr_rsp:%d  dsc_sz:%d port_id:%d  func:%d  cidx%d  qid:%u\n",
		 msg, byp->error, byp->st_mm, byp->mrkr_rsp, byp->dsc_sz,
		 byp->port_id, byp->func, byp->cidx, byp->qid);
}

int stmn_print_error_msg(void *dev_hndl, char *buf, int len)
{
	int ret;
	uint32_t i, num_entry;
	uint32_t *reg_val;
	struct stmn_fatal_err_status err;
	struct stmn_err_capture errc;

	/*Append fatal err reg name and value to msg*/
	ret = stmn_get_fatal_err_info(dev_hndl, &err);
	stmn_fail_check(ret, "stmn_get_fatal_err_info");
	reg_val = (uint32_t *)&err;
	num_entry = sizeof(struct stmn_fatal_err_status) / sizeof(uint32_t);
	for (i = 0; i < num_entry; i++)
		snprintf(buf + strlen(buf), len - strlen(buf), "%-30s 0x%x\n",
			 stmn_reg_name_fatal_err[i], *(reg_val + i));

	/*Append err capture to msg */
	ret = stmn_get_err_capture_info(dev_hndl, &errc);
	stmn_fail_check(ret, "stmn_get_err_capture_info");
	stmn_prn_dsc_err(&errc.c2h_dsc, buf, len, "C2H Desc Err     : ");
	stmn_prn_dsc_err(&errc.h2c_dsc, buf, len, "H2C Desc Err     : ");
	stmn_prn_byp_err(&errc.c2h_byp_out, buf, len, "C2H Desc Byp Err : ");
	stmn_prn_byp_err(&errc.h2c_byp_out, buf, len, "H2C Desc Byp Err : ");

	return STMN_SUCCESS;
}

/******************************************************************************/
static int stmn_trigger_ram_read(void *dev_hndl, uint16_t qid,
				 uint32_t reg_offst)
{
	uint32_t reg, counter = 0;
#ifdef STMN_TEST_US
	return STMN_SUCCESS;
#endif
	/* write Queue id for status*/
	reg = stmn_reg_read(dev_hndl, reg_offst);
	reg = (reg & ~(STMN_RAM_READ_QID_MASK)) | qid;
	stmn_reg_write(dev_hndl, reg_offst, reg);

	/* write to read request for status*/
	reg = reg | STMN_RAM_READ_VLD_BIT;
	stmn_reg_write(dev_hndl, reg_offst, reg);

	do {
		reg = stmn_reg_read(dev_hndl, reg_offst);
		if ((reg & STMN_RAM_READ_DONE_BIT) ||
		    counter > STMN_POLL_COUNT_MAX)
			break;
		counter++;
		pr_debug("Polling for read-done, RegVal:%d Counter:%d\n", reg,
			 counter);
		sleep(STMN_POLL_INTERVAL);
	} while (1);

	if (counter > STMN_POLL_COUNT_MAX) {
		pr_err("Polling failed for qid %d RamOffset:0x%x\n", qid,
		       reg_offst);
		return -STMN_FAILURE;
	}

	return STMN_SUCCESS;
}

int stmn_get_ctrl_ram_status(void *dev_hndl, uint16_t qid,
			     struct stmn_ctrl_ram_status *ram_info)
{
	int ret;
	uint32_t i, offset;
	uint32_t *val = (uint32_t *)ram_info;
	uint32_t num_reg =
		sizeof(struct stmn_ctrl_ram_status) / sizeof(uint32_t);

	stmn_null_check(dev_hndl, "Invalid dev_hndl");
	stmn_null_check(ram_info, "Invalid status_ctrl_ram_info param");

	for (i = 0; i < num_reg; i++) {
		offset = stmn_regs.ctrl_ram_status + (4 * 2 * i);
		ret = stmn_trigger_ram_read(dev_hndl, qid, offset);
		stmn_fail_check(ret, "stmn_trigger_ram_read");
		*(val + i) = stmn_reg_read(dev_hndl, offset + 4);
	}

	return STMN_SUCCESS;
}

static void stmn_prn_sts_ram(struct stmn_sts_ram_val *ram, char *buf, int len,
			     const char *msg, uint16_t qid)
{
	snprintf(buf + strlen(buf), len - strlen(buf),
		 "%-16s: qid:%d qen:%d byp:%d init:%d irq_arm:%d rst:%d err:%d mm:%d dsc_avl:0x%x\n",
		 msg, qid, ram->qen, ram->byp, ram->init, ram->irq_arm,
		 ram->rst, ram->err, ram->mm, ram->dsc_avl);
}

static void stmn_prn_str_ram(struct stmn_store_ram_val *ram, char *buf, int len,
			     const char *msg, uint16_t qid)
{
	snprintf(buf + strlen(buf), len - strlen(buf),
		 "%-16s: qid:%d en:%d err:%d rst:%d in_arb:%d ptr1:%d ptr2:%d\n",
		 msg, qid, ram->en, ram->err, ram->rst, ram->in_arb,
		 ram->rd_ptr, ram->wr_ptr);
}

int stmn_print_ram_status_msg(void *dev_hndl, char *buf, int len)
{
	int ret;
	struct stmn_ctrl_ram_status ram;
	uint16_t i;
	uint16_t qid_base = STMN_CRL_RAM_QID_BASE;
	uint16_t nq = STMN_CRL_RAM_NUM_QID;

	for (i = qid_base; i < qid_base + nq; i++) {
		ret = stmn_get_ctrl_ram_status(dev_hndl, i, &ram);
		//TODO BUG in HW
		stmn_fail_check(ret, "stmn_get_ctrl_ram_status");
		stmn_prn_sts_ram(&ram.c2h_dsc, buf, len, "C2H Desc RAM", i);
		stmn_prn_sts_ram(&ram.h2c_dsc, buf, len, "H2C Desc RAM", i);
		stmn_prn_str_ram(&ram.c2h_wr, buf, len, "C2H Desc Wr RAM", i);
		stmn_prn_str_ram(&ram.c2h_rd, buf, len, "C2H Desc Rd RAM", i);
		stmn_prn_str_ram(&ram.h2c_wr, buf, len, "H2C Desc Wr RAM", i);
		stmn_prn_str_ram(&ram.h2c_rd, buf, len, "H2C Desc Rd RAM", i);
		memset(&ram, 0, sizeof(struct stmn_ctrl_ram_status));
	}

	return STMN_SUCCESS;
}
/*****************************************************************************/
int stmn_get_idle_status(void *dev_hndl, struct stmn_idle_status *idle)
{
	uint32_t i;
	uint32_t *val = (uint32_t *)idle;
	uint32_t num_reg = sizeof(struct stmn_idle_status) / sizeof(uint32_t);

	stmn_null_check(dev_hndl, "Invalid dev_hndl");
	stmn_null_check(idle, "Invalid stmn_idle_status param");

	for (i = 0; i < num_reg; i++)
		*(val + i) = stmn_reg_read(dev_hndl,
					   stmn_regs.idle_status + (4 * i));

	return STMN_SUCCESS;
}

int stmn_get_axis_debug_status(void *dev_hndl, struct stmn_axis_debug *debug)
{
	uint32_t i;
	uint32_t *val = (uint32_t *)debug;
	uint32_t num_reg = sizeof(struct stmn_axis_debug) / sizeof(uint32_t);

	stmn_null_check(dev_hndl, "Invalid dev_hndl");
	stmn_null_check(debug, "Invalid stmn_axis_debug param");

	for (i = 0; i < num_reg; i++)
		*(val + i) =
			stmn_reg_read(dev_hndl, stmn_regs.s_axis_dbg + (4 * i));

	return STMN_SUCCESS;
}
static void stmn_prn_axis_rsp(struct stmn_axis_resp *rsp, char *buf, int len,
			      const char *msg)
{
	snprintf(buf + strlen(buf), len - strlen(buf),
		 "%-24s : drop_code:%d num_desc:%d num_pkt:%d accept:%d sim_byp:%d qid:%d reg_val:%d\n",
		 msg, rsp->drop_code, rsp->num_desc, rsp->num_pkt, rsp->accept,
		 rsp->sim_byp_mode, rsp->qid, rsp->reg_val);
}
int stmn_print_status_msg(void *dev_hndl, char *buf, int len)
{
	int ret;
	uint32_t i, num_entry;
	uint32_t *reg_val;

	struct stmn_idle_status debug;
	struct stmn_axis_debug axis_dbg;
	struct stmn_axis_resp_flags *flags;
	struct stmn_qdma_c2h_sts_flags *c2h_flags;

	ret = stmn_get_idle_status(dev_hndl, &debug);
	stmn_fail_check(ret, "stmn_get_idle_status");
	reg_val = (uint32_t *)&debug;
	num_entry = sizeof(struct stmn_idle_status) / sizeof(uint32_t);
	for (i = 0; i < num_entry; i++)
		snprintf(buf + strlen(buf), len - strlen(buf), "%-30s 0x%x\n",
			 stmn_reg_name_idle_status[i], *(reg_val + i));

	ret = stmn_get_axis_debug_status(dev_hndl, &axis_dbg);
	stmn_fail_check(ret, "stmn_get_axis_debug_status");

	stmn_prn_axis_rsp(&axis_dbg.last_rsp, buf, len,
			  "Last Drop Axis response");
	stmn_prn_axis_rsp(&axis_dbg.first_drop, buf, len,
			  "First Drop Axis response");

	flags = &axis_dbg.axis_flgs;
	snprintf(
		buf + strlen(buf), len - strlen(buf),
		"Dropping Queue Type      : accept_sim_byp:%d  accept_not_byp:%d drop_cache_byp:%d  drop_sim_byp:%d  drop_not_byp:%d\n",
		flags->accept_sim_byp, flags->accept_not_byp,
		flags->drop_cache_byp, flags->drop_sim_byp,
		flags->drop_not_byp);
	snprintf(
		buf + strlen(buf), len - strlen(buf),
		"Dropping Queue cause     : drop_sim_byp_len:%d  drop_sim_byp_avl:%d drop_sim_byp_init:%d  drop_qid_err:%d  drop_qid_not_st:%d  drop_qid_not_en:%d\n",
		flags->drop_sim_byp_len, flags->drop_sim_byp_avl,
		flags->drop_sim_byp_init, flags->drop_qid_err,
		flags->drop_qid_not_st, flags->drop_qid_not_en);
	c2h_flags = &axis_dbg.c2h_flags;
	snprintf(buf + strlen(buf), len - strlen(buf),
		 "C2h Flags: last:%d  cmp:%d not_last:%d  drop:%d  max_qid:%d\n",
		 c2h_flags->last, c2h_flags->cmp, c2h_flags->not_last,
		 c2h_flags->drop, c2h_flags->max_qid);

	return STMN_SUCCESS;
}
/******************************************************************************/

int stmn_print_stats(void *dev_hndl, char *msg, int len)
{
	int ret;

	stmn_null_check(dev_hndl, "dev_hndl");
	stmn_null_check(msg, "msg");

	ret = stmn_print_stats_msg(dev_hndl, msg, len);
	stmn_fail_check(ret, "stmn_print_stats_msg");

	ret = stmn_print_fifo_level_msg(dev_hndl, msg, len);
	stmn_fail_check(ret, "stmn_print_fifo_level_msg");

	ret = stmn_print_dsc_minmax_msg(dev_hndl, msg, len);
	stmn_fail_check(ret, "stmn_print_dsc_minmax_msg");

	pr_debug("%s buf_len: %lu\n", __func__, strlen(msg));

	return STMN_SUCCESS;
}

int stmn_print_debug(void *dev_hndl, char *msg, uint32_t len)
{
	int ret;
	struct stmn_ip_version ver = { 0, 0, 0 };

	stmn_null_check(dev_hndl, "dev_hndl");
	stmn_null_check(msg, "msg");

	ret = stmn_get_version(dev_hndl, &ver);
	stmn_fail_check(ret, "stmn_get_version");
	snprintf(msg, len, "STMN version Minor:%d Major:%d BlcokID:%x\n",
		 ver.minor, ver.major, ver.block_id);

	snprintf(msg + strlen(msg), len - strlen(msg),
		 "***********Status***********\n");
	ret = stmn_print_status_msg(dev_hndl, msg, len);
	stmn_fail_check(ret, "stmn_print_status_msg");

	snprintf(msg + strlen(msg), len - strlen(msg),
		 "***********Error***********\n");
	ret = stmn_print_error_msg(dev_hndl, msg, len);
	stmn_fail_check(ret, "stmn_print_error_msg");
#if STMN_EN_MEM_DUMP
	snprintf(msg + strlen(msg), len - strlen(msg),
		 "***********Memory***********\n");
	stmn_print_ram_status_msg(dev_hndl, msg, len);

	pr_debug("%s buf_len: %lu\n", __func__, strlen(msg));
#endif
	return STMN_SUCCESS;
}

int stmn_print_msg(void *dev_hndl, char *msg, uint32_t len)
{
	stmn_null_check(dev_hndl, "dev_hndl");
	stmn_null_check(msg, "msg");

	stmn_print_debug(dev_hndl, msg, len);
	snprintf(msg + strlen(msg), len - strlen(msg),
		 "***********Statistics***********\n");
	stmn_print_stats(dev_hndl, msg, len);

	return STMN_SUCCESS;
}

/******************************************************************************/
