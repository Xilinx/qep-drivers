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
#include <linux/delay.h>
#include "libqdma_export.h"
#include "qep_stmn.h"
#include "qep.h"

#define H2C_DESC_MAX_DATA_LEN   (PAGE_SIZE)
#define H2C_STMN_GL_SHIFT       (5)
#define H2C_STMN_SDI_SHIFT      (15)
#define H2C_STMN_EOT_SHIFT      (14)

#define stmn_fls      (fls(H2C_DESC_MAX_DATA_LEN) - 1)
#define stmn_get_desc_cnt(x) \
	((x) ? ((x + H2C_DESC_MAX_DATA_LEN - 1) >> stmn_fls) : 1)

/* STMN Completion(CMPT) entry  format */
/* Calculate number of desc required for DMA request  */
static unsigned int calculate_stmn_gl_count(struct qdma_request *req)
{
	struct qdma_sw_sg *sg = req->sgl;
	int i, count = 0;

	for (i = 0; i < req->sgcnt; i++) {
		count += stmn_get_desc_cnt(sg->len);
		sg = sg->next;
	}
	return count;
}

/* Update desc for a single H2C DMA request */
static int stmn_proc_h2c_request_single(void *qhndl,
					struct qdma_request *req)
{
	struct qdma_sw_sg *sg = req->sgl;
	unsigned int sg_max = req->sgcnt;
	unsigned int data_cnt = 0;
	unsigned int desc_cnt = 0;
	unsigned int sg_offset = 0;
	int i;
	int rv;
	unsigned int gl_count = 0;

#if (QEP_DBG_DUMP_EN)
	dump_qdma_sw_sgl(sg_max, req->sgl);
#endif
	i = qdma_sgl_find_offset(req, &sg, &sg_offset);
	if (unlikely(i < 0))
		return -EINVAL;
	if (likely(i == 0))
		gl_count = calculate_stmn_gl_count(req);

	for (; i < sg_max; i++, sg++) {
		struct qdma_q_desc_list *desc_list = NULL;
		struct qdma_stmn_h2c_desc *desc = NULL;
		unsigned int tlen = sg->len;
		dma_addr_t addr = sg->dma_addr;
		unsigned int len;
		unsigned int req_desc_cnt;
		int j;

		if (sg_offset) {
			tlen -= sg_offset;
			addr += sg_offset;
			sg_offset = 0;
		}

		req_desc_cnt = stmn_get_desc_cnt(tlen);
		rv = qdma_q_desc_get(qhndl, req_desc_cnt, &desc_list);
		if (unlikely(rv < 0)) {
			if (desc_cnt)
				goto update_req;
			else
				return 0;
		}
		for (j = 0; j < req_desc_cnt; j++) {
			desc = desc_list->desc;
			desc->flags = 0;
			len = min_t(unsigned int, tlen,
					H2C_DESC_MAX_DATA_LEN);
			desc->src_addr = addr;
			desc->len = len;
			if ((i == 0) && (j == 0))
				desc->flags = (gl_count <<
						H2C_STMN_GL_SHIFT);

			/* Copy Checksum related data */
			memcpy(&desc->metadata, req->udd,
					sizeof(desc->metadata));

			data_cnt += len;
			addr += len;
			tlen -= len;

			desc_cnt++;
			desc_list = desc_list->next;
		}
	}
	sg = NULL;

update_req:
	qdma_update_request(qhndl, req, desc_cnt, data_cnt, 0, sg);

	return desc_cnt;
}

/* Parse CMPT dsc entry */
int qep_stmn_parse_cmpl_entry(void *cmpl_entry, struct qdma_ul_cmpt_info *cmpl)
{
	struct stmn_cmpt_entry *cmpt = (struct stmn_cmpt_entry *)cmpl_entry;

	dma_rmb();

	cmpl->entry = (__be64 *)cmpt;
	cmpl->f.format = cmpt->format;
	cmpl->f.color = cmpt->color;
	cmpl->f.err = (cmpt->err | cmpt->usr_err);
	cmpl->f.desc_used = (cmpt->desc_used);
	if (!cmpl->f.format && cmpl->f.desc_used) {
		cmpl->len = (cmpt->len);
		/* zero length transfer allowed */
	} else
		cmpl->len = 0;

	return cmpl->f.err ? -EINVAL : 0;
}

/* Fill desc entry for DMA request */
int qep_stmn_bypass_desc_fill(void *q_hndl, enum qdma_q_mode q_mode,
			       enum qdma_q_dir q_dir, struct qdma_request *req)
{
	if (likely((q_mode == QDMA_Q_MODE_ST) && (q_dir == QDMA_Q_DIR_H2C)))
		return stmn_proc_h2c_request_single(q_hndl, req);

	return -EINVAL;
}
