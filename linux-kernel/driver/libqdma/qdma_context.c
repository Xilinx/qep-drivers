/*
 * This file is part of the Xilinx DMA IP Core driver for Linux
 *
 * Copyright (c) 2017-2019,  Xilinx, Inc.
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

#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/pci.h>
#include "qdma_device.h"
#include "qdma_descq.h"
#include "qdma_intr.h"
#include "qdma_regs.h"
#include "qdma_context.h"
#include "qdma_access.h"
#include "qdma_mbox_protocol.h"
#include "qdma_reg.h"

/**
 * Make the interrupt context
 */
static int make_intr_context(struct xlnx_dma_dev *xdev,
			     struct qdma_indirect_intr_ctxt *ctxt)
{
	int i;

	if ((xdev->conf.qdma_drv_mode != INDIRECT_INTR_MODE) &&
			(xdev->conf.qdma_drv_mode != AUTO_MODE)) {
		return -EINVAL;
	}

	/** program the coalescing context
	 *  i -> Number of vectors
	 */
	for (i = 0; i <  QDMA_NUM_DATA_VEC_FOR_INTR_CXT; i++) {
		struct intr_coal_conf *entry = (xdev->intr_coal_list + i);

		ctxt[i].valid = 1;
		ctxt[i].vec = entry->vec_id;
		ctxt[i].baddr_4k = entry->intr_ring_bus;
		ctxt[i].color = entry->color;
		ctxt[i].page_size = QDMA_INDIRECT_INTR_RING_SIZE_4KB;
	}

	return 0;
}

#ifndef __QDMA_VF__
static int make_sw_context(struct qdma_descq *descq,
			   struct qdma_descq_sw_ctxt *sw_ctxt)
{
	memset(sw_ctxt, 0, sizeof(struct qdma_descq_sw_ctxt));

	/* sw context */
	if ((descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
			(descq->xdev->conf.qdma_drv_mode == AUTO_MODE)) {
		sw_ctxt->vec = get_intr_ring_index(descq->xdev,
						       descq->intr_id);
		sw_ctxt->intr_aggr = 0x01;
	} else {
		sw_ctxt->vec = descq->intr_id;
	}

	sw_ctxt->ring_bs_addr = descq->desc_bus;
	sw_ctxt->wbi_chk = descq->conf.cmpl_status_pend_chk;
	sw_ctxt->wbi_intvl_en = descq->conf.cmpl_status_acc_en;
	sw_ctxt->rngsz_idx = descq->conf.desc_rng_sz_idx;
	sw_ctxt->bypass = descq->conf.desc_bypass;
	sw_ctxt->wbk_en = descq->conf.wb_status_en;
	sw_ctxt->irq_en = descq->conf.irq_en;
	sw_ctxt->is_mm = ~descq->conf.st;
	sw_ctxt->qen = 1;

	if (descq->conf.desc_bypass &&
			(descq->conf.sw_desc_sz == DESC_SZ_64B)) {
		sw_ctxt->desc_sz = descq->conf.sw_desc_sz;
	} else {
		if (!descq->conf.st) { /* mm h2c/c2h */
			sw_ctxt->desc_sz = DESC_SZ_32B;
			sw_ctxt->mm_chn = descq->channel;
		} else if (descq->conf.c2h) {  /* st c2h */
			sw_ctxt->frcd_en = descq->conf.fetch_credit;
			sw_ctxt->desc_sz = DESC_SZ_8B;
		} else /* st h2c */
			sw_ctxt->desc_sz = DESC_SZ_16B;
	}

	/* pidx = 0; irq_ack = 0 */
	sw_ctxt->fnc_id = descq->xdev->func_id;
	sw_ctxt->irq_arm = descq->conf.irq_en;

	if (descq->conf.st && descq->conf.c2h) {
		sw_ctxt->irq_en = 0;
		sw_ctxt->irq_arm = 0;
		sw_ctxt->wbk_en = 0;
		sw_ctxt->wbi_chk = 0;
	}

#ifdef ERR_DEBUG
	if (descq->induce_err & (1 << param)) {
		sw_ctxt->fnc_id = 0xFFF;
		pr_info("induced error %d", ind_ctxt_cmd_err);
	}
#endif

	return 0;
}

/* ST: prefetch context setup */
static int make_prefetch_context(struct qdma_descq *descq,
				 struct qdma_descq_prefetch_ctxt *pfetch_ctxt)
{
	BUG_ON(!descq);
	BUG_ON(!pfetch_ctxt);

	memset(pfetch_ctxt, 0, sizeof(struct qdma_descq_prefetch_ctxt));

	/* prefetch context */
	pfetch_ctxt->valid = 1;
	pfetch_ctxt->bypass = descq->conf.pfetch_bypass;
	pfetch_ctxt->bufsz_idx = descq->conf.c2h_buf_sz_idx;
	pfetch_ctxt->pfch_en = descq->conf.pfetch_en;

	return 0;
}

/* ST C2H : writeback context setup */
static int make_cmpt_context(struct qdma_descq *descq,
			     struct qdma_descq_cmpt_ctxt *cmpt_ctxt)
{
	int ring_index;

	memset(cmpt_ctxt, 0, sizeof(struct qdma_descq_cmpt_ctxt));

	cmpt_ctxt->en_stat_desc = descq->conf.cmpl_stat_en;
	cmpt_ctxt->en_int = descq->conf.cmpl_en_intr;
	cmpt_ctxt->trig_mode = descq->conf.cmpl_trig_mode;
	cmpt_ctxt->fnc_id = descq->xdev->func_id;
	cmpt_ctxt->timer_idx = descq->conf.cmpl_timer_idx;
	cmpt_ctxt->counter_idx = descq->conf.cmpl_cnt_th_idx;
	cmpt_ctxt->color = 1;
	cmpt_ctxt->ringsz_idx = descq->conf.cmpl_rng_sz_idx;

	cmpt_ctxt->bs_addr = descq->desc_cmpt_bus;
	cmpt_ctxt->desc_sz = descq->conf.cmpl_desc_sz;
	cmpt_ctxt->full_upd = descq->conf.adaptive_rx;

	cmpt_ctxt->valid = 1;

	cmpt_ctxt->ovf_chk_dis = descq->conf.cmpl_ovf_chk_dis;
	if ((descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
			(descq->xdev->conf.qdma_drv_mode == AUTO_MODE)) {
		ring_index = get_intr_ring_index(descq->xdev, descq->intr_id);
		cmpt_ctxt->vec = ring_index;
		cmpt_ctxt->int_aggr = 1;
	} else {
		cmpt_ctxt->vec = descq->intr_id;
	}

	return 0;
}
#endif

#ifdef __QDMA_VF__
int qdma_intr_context_setup(struct xlnx_dma_dev *xdev)
{
	int i = 0;
	int rv;
	struct mbox_msg *m = NULL;
	struct mbox_msg_intr_ctxt ictxt;

	if ((xdev->conf.qdma_drv_mode != INDIRECT_INTR_MODE) &&
			(xdev->conf.qdma_drv_mode != AUTO_MODE))
		return 0;

	m = qdma_mbox_msg_alloc();
	if (!m)
		return -ENOMEM;

	memset(&ictxt, 0, sizeof(struct mbox_msg_intr_ctxt));

	ictxt.num_rings = QDMA_NUM_DATA_VEC_FOR_INTR_CXT;

	for (i = 0; i < QDMA_NUM_DATA_VEC_FOR_INTR_CXT; i++) {
		ictxt.ring_index_list[i] =
			get_intr_ring_index(xdev, xdev->dvec_start_idx + i);
	}

	rv = make_intr_context(xdev, ictxt.ictxt);
	if (rv < 0)
		goto free_msg;

	qdma_mbox_compose_vf_intr_ctxt_write(xdev->func_id, &ictxt, m->raw);
	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		pr_err("%s, mbox failed for interrupt context %d.\n",
				xdev->conf.name, rv);
		goto free_msg;
	}
	rv = qdma_mbox_vf_response_status(m->raw);

free_msg:
	qdma_mbox_msg_free(m);
	return rv;
}

int qdma_intr_context_read(struct xlnx_dma_dev *xdev,
				   int ring_index,
				   struct qdma_indirect_intr_ctxt *ctxt)
{
	struct mbox_msg *m;
	int rv = 0;
	struct mbox_msg_intr_ctxt ictxt;

	m = qdma_mbox_msg_alloc();
	if (!m)
		return -ENOMEM;
	memset(&ictxt, 0, sizeof(struct mbox_msg_intr_ctxt));
	ictxt.num_rings = 1;

	ictxt.ring_index_list[0] = ring_index;
	qdma_mbox_compose_vf_intr_ctxt_read(xdev->func_id,
			&ictxt, m->raw);
	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		pr_err("%s invalidate interrupt context failed %d.\n",
			xdev->conf.name, rv);
	}
	rv = qdma_mbox_vf_intr_context_get(m->raw, &ictxt);
	if (rv == 0)
		memcpy(ctxt, &ictxt.ictxt[0],
		       sizeof(struct qdma_indirect_intr_ctxt));

	qdma_mbox_msg_free(m);

	return rv;
}

int qdma_descq_context_clear(struct xlnx_dma_dev *xdev, unsigned int qid_hw,
				bool st, bool c2h, bool mm_cmpt_en, bool clr)
{
	struct mbox_msg *m = qdma_mbox_msg_alloc();
	int rv;
	enum mbox_cmpt_ctxt_type cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_NONE;

	if (!m)
		return -ENOMEM;

	if (!st) {
		if (mm_cmpt_en)
			cmpt_ctxt_type = QDMA_MBOX_CMPT_WITH_MM;
		else
			cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_NONE;
	} else {
		if (c2h)
			cmpt_ctxt_type = QDMA_MBOX_CMPT_WITH_ST;
	}

	if (clr)
		qdma_mbox_compose_vf_qctxt_clear(xdev->func_id,
				qid_hw, st, c2h, cmpt_ctxt_type, m->raw);
	else
		qdma_mbox_compose_vf_qctxt_invalidate(xdev->func_id,
				qid_hw, st, c2h, cmpt_ctxt_type, m->raw);

	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		if (rv != -ENODEV)
			pr_info("%s, qid_hw 0x%x mbox failed %d.\n",
				xdev->conf.name, qid_hw, rv);
		goto err_out;
	}

	rv = qdma_mbox_vf_response_status(m->raw);

err_out:
	qdma_mbox_msg_free(m);
	return rv;
}

int qdma_descq_context_read(struct xlnx_dma_dev *xdev, unsigned int qid_hw,
				bool st, bool c2h, bool mm_cmpt_en,
				struct qdma_descq_context *context)
{
	struct mbox_msg *m = qdma_mbox_msg_alloc();
	int rv;
	enum mbox_cmpt_ctxt_type cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_NONE;

	if (!m)
		return -ENOMEM;

	if (!st) {
		if (mm_cmpt_en)
			cmpt_ctxt_type = QDMA_MBOX_CMPT_WITH_MM;
		else
			cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_NONE;
	} else {
		if (c2h)
			cmpt_ctxt_type = QDMA_MBOX_CMPT_WITH_ST;
	}

	qdma_mbox_compose_vf_qctxt_read(xdev->func_id,
				qid_hw, st, c2h, cmpt_ctxt_type, m->raw);

	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		if (rv != -ENODEV)
			pr_info("%s, qid_hw 0x%x mbox failed %d.\n",
				xdev->conf.name, qid_hw, rv);
		goto err_out;
	}

	rv = qdma_mbox_vf_context_get(m->raw, context);

err_out:
	qdma_mbox_msg_free(m);
	return rv;
}

int qdma_descq_context_setup(struct qdma_descq *descq)
{
	struct xlnx_dma_dev *xdev = descq->xdev;
	struct mbox_msg *m = qdma_mbox_msg_alloc();
	struct mbox_descq_conf descq_conf;
	int rv;
	enum mbox_cmpt_ctxt_type cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_NONE;

	if (!m)
		return -ENOMEM;
	memset(&descq_conf, 0, sizeof(struct mbox_descq_conf));
	descq_conf.ring_bs_addr = descq->desc_bus;
	descq_conf.cmpt_ring_bs_addr = descq->desc_cmpt_bus;
	descq_conf.en_bypass = descq->conf.desc_bypass;
	descq_conf.irq_arm = descq->conf.irq_en;
	descq_conf.wbi_intvl_en = descq->conf.cmpl_status_acc_en;
	descq_conf.wbi_chk = descq->conf.cmpl_status_pend_chk;
	descq_conf.at = descq->conf.at;
	descq_conf.wbk_en = descq->conf.wb_status_en;
	descq_conf.irq_en = descq->conf.irq_en;
	descq_conf.pfch_en = descq->conf.pfetch_en;
	descq_conf.en_bypass_prefetch = descq->conf.pfetch_bypass;
	descq_conf.dis_overflow_check = descq->conf.cmpl_ovf_chk_dis;
	descq_conf.cmpt_int_en = descq->conf.cmpl_en_intr;
	descq_conf.cmpl_stat_en = descq->conf.cmpl_stat_en;
	if (descq->conf.desc_bypass &&
			(descq->conf.sw_desc_sz == DESC_SZ_64B))
		descq_conf.desc_sz = descq->conf.sw_desc_sz;
	else {
		if (!descq->conf.st) /* mm h2c/c2h */
			descq_conf.desc_sz = DESC_SZ_32B;
		else if (descq->conf.c2h)  {/* st c2h */
			descq_conf.desc_sz = DESC_SZ_8B;
			descq_conf.forced_en = descq->conf.fetch_credit;
		} else /* st h2c */
			descq_conf.desc_sz = DESC_SZ_16B;
	}
	descq_conf.cmpt_desc_sz = descq->conf.cmpl_desc_sz;
	descq_conf.triggermode = descq->conf.cmpl_trig_mode;
	descq_conf.cmpt_at = descq->conf.at;
	descq_conf.cmpt_color = 1;
	descq_conf.cmpt_full_upd = 0;
	descq_conf.func_id = descq->xdev->func_id;
	descq_conf.cnt_thres =
			xdev->csr_info.c2h_cnt_th[descq->conf.cmpl_cnt_th_idx];
	descq_conf.timer_thres =
		xdev->csr_info.c2h_timer_cnt[descq->conf.cmpl_timer_idx];
	descq_conf.ringsz = descq->conf.rngsz;
	descq_conf.bufsz = descq->conf.c2h_bufsz;
	descq_conf.cmpt_ringsz = descq->conf.rngsz_cmpt;
	if ((descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
			(descq->xdev->conf.qdma_drv_mode == AUTO_MODE)) {
		int ring_index = get_intr_ring_index(descq->xdev,
						     descq->intr_id);
		descq_conf.intr_id = ring_index & 0xFFF;
		descq_conf.intr_aggr = 1;
	} else
		descq_conf.intr_id = descq->intr_id;

	if (!descq->conf.st) {
		if (descq->mm_cmpt_ring_crtd)
			cmpt_ctxt_type = QDMA_MBOX_CMPT_WITH_MM;
		else
			cmpt_ctxt_type = QDMA_MBOX_CMPT_CTXT_NONE;
	} else {
		if (descq->conf.c2h)
			cmpt_ctxt_type = QDMA_MBOX_CMPT_WITH_ST;
	}

	qdma_mbox_compose_vf_qctxt_write(xdev->func_id, descq->qidx_hw,
				descq->conf.st, descq->conf.c2h,
				cmpt_ctxt_type, &descq_conf, m->raw);

	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		if (rv != -ENODEV)
			pr_info("%s, qid_hw 0x%x, %s mbox failed %d.\n",
				xdev->conf.name, descq->qidx_hw,
				descq->conf.name, rv);
		goto err_out;
	}

	rv = qdma_mbox_vf_response_status(m->raw);

err_out:
	qdma_mbox_msg_free(m);
	return rv;
}

#else /* PF only */

int qdma_prog_intr_context(struct xlnx_dma_dev *xdev,
		struct mbox_msg_intr_ctxt *ictxt)
{
	int i = 0;
	int rv;
	int ring_index;
	struct qdma_indirect_intr_ctxt *ctxt;

	for (i = 0; i < ictxt->num_rings; i++) {
		ring_index = ictxt->ring_index_list[i];

		ctxt = &ictxt->ictxt[i];
		rv = qdma_indirect_intr_context_write(xdev, ring_index, ctxt);
		if (rv < 0)
			return rv;
	}

	return 0;
}

int qdma_intr_context_setup(struct xlnx_dma_dev *xdev)
{
	struct qdma_indirect_intr_ctxt ctxt[QDMA_NUM_DATA_VEC_FOR_INTR_CXT];
	int i = 0;
	int rv;
	int ring_index;

	if ((xdev->conf.qdma_drv_mode != INDIRECT_INTR_MODE) &&
			(xdev->conf.qdma_drv_mode != AUTO_MODE))
		return 0;

	memset(ctxt, 0, sizeof(struct qdma_indirect_intr_ctxt) *
	       QDMA_NUM_DATA_VEC_FOR_INTR_CXT);
	/** Preparing the interrupt context for all the vectors
	 *  each vector's context width is QDMA_REG_IND_CTXT_WCNT_3(3)
	 */
	rv = make_intr_context(xdev, ctxt);
	if (rv < 0)
		return rv;

	for (i = 0; i <  QDMA_NUM_DATA_VEC_FOR_INTR_CXT; i++) {
		ring_index = get_intr_ring_index(xdev,
				(i + xdev->dvec_start_idx));
		rv = qdma_indirect_intr_context_clear(xdev, ring_index);
		if (rv < 0)
			return rv;
		rv = qdma_indirect_intr_context_write(xdev,
				ring_index, &ctxt[i]);
		if (rv < 0)
			return rv;
	}

	return 0;
}

int qdma_descq_context_clear(struct xlnx_dma_dev *xdev, unsigned int qid_hw,
				bool st, bool c2h, bool mm_cmpt_en, bool clr)
{
	int rv = 0;

	if (clr) {
		rv = qdma_sw_context_clear(xdev, c2h, qid_hw);
		if (rv < 0) {
			pr_err("Fail to clear sw context, rv = %d", rv);
			return rv;
		}

		rv = qdma_hw_context_clear(xdev, c2h, qid_hw);
		if (rv < 0) {
			pr_err("Fail to clear hw context, rv = %d", rv);
			return rv;
		}

		rv = qdma_credit_context_clear(xdev, c2h, qid_hw);
		if (rv < 0) {
			pr_err("Fail to clear credit context, rv = %d", rv);
			return rv;
		}

		/* Only clear prefetch and writeback contexts if this queue is
		 * ST C2H
		 */
		if (st && c2h) {
			rv = qdma_pfetch_context_clear(xdev, qid_hw);
			if (rv < 0) {
				pr_err("Fail to clear pfetch context, rv = %d",
				       rv);
				return rv;
			}
		}

		/* Only clear cmpt context if this queue is ST C2H or MM cmpt*/
		if ((st && c2h) || (!st && mm_cmpt_en)) {
			rv = qdma_cmpt_context_clear(xdev, qid_hw);
			if (rv < 0) {
				pr_err("Fail to clear cmpt context, rv = %d",
				       rv);
				return rv;
			}
		}

	} else {

		rv = qdma_sw_context_invalidate(xdev, c2h, qid_hw);
		if (rv < 0) {
			pr_err("Fail to invalidate sw context, rv = %d", rv);
			return rv;
		}

		rv = qdma_hw_context_invalidate(xdev, c2h, qid_hw);
		if (rv < 0) {
			pr_err("Fail to invalidate hw context, rv = %d", rv);
			return rv;
		}

		rv = qdma_credit_context_invalidate(xdev, c2h, qid_hw);
		if (rv < 0) {
			pr_err("Fail to invalidate credit context, rv = %d",
			       rv);
			return rv;
		}

		/* Only clear prefetch and writeback contexts if this queue is
		 * ST C2H
		 */
		if (st && c2h) {

			rv = qdma_pfetch_context_invalidate(xdev, qid_hw);
			if (rv < 0) {
				pr_err("Fail to invalidate pfetch context, rv = %d",
				       rv);
				return rv;
			}
		}

		/* Only clear cmpt context if this queue is ST C2H MM cmpt*/
		if ((st && c2h) || (!st && mm_cmpt_en)) {
			rv = qdma_cmpt_context_invalidate(xdev, qid_hw);
			if (rv < 0) {
				pr_err("Fail to invalidate cmpt context, rv = %d",
				       rv);
				return rv;
			}
		}
	}

	return 0;
}

int qdma_descq_context_setup(struct qdma_descq *descq)
{
	struct qdma_descq_context context;

	memset(&context, 0, sizeof(context));

	make_sw_context(descq, &context.sw_ctxt);

	if (descq->conf.st && descq->conf.c2h)
		make_prefetch_context(descq, &context.pfetch_ctxt);

	if ((descq->conf.st && descq->conf.c2h) ||
	    (!descq->conf.st && descq->mm_cmpt_ring_crtd)) {
		make_cmpt_context(descq, &context.cmpt_ctxt);
	}


	return qdma_descq_context_program(descq->xdev, descq->qidx_hw,
				descq->conf.st, descq->conf.c2h,
				descq->mm_cmpt_ring_crtd, &context);
}

int qdma_descq_context_read(struct xlnx_dma_dev *xdev, unsigned int qid_hw,
				bool st, bool c2h, bool mm_cmpt_en,
				struct qdma_descq_context *context)
{
	int rv = 0;

	memset(context, 0, sizeof(struct qdma_descq_context));

	rv = qdma_sw_context_read(xdev, c2h, qid_hw, &context->sw_ctxt);
	if (rv < 0) {
		pr_err("Failed to read sw context, rv = %d", rv);
		return rv;
	}

	rv = qdma_hw_context_read(xdev, c2h, qid_hw, &context->hw_ctxt);
	if (rv < 0) {
		pr_err("Failed to read hw context, rv = %d", rv);
		return rv;
	}

	rv = qdma_credit_context_read(xdev, c2h, qid_hw, &context->cr_ctxt);
	if (rv < 0) {
		pr_err("Failed to read hw context, rv = %d", rv);
		return rv;
	}

	if (st && c2h) {
		rv = qdma_pfetch_context_read(xdev, qid_hw,
					      &context->pfetch_ctxt);
		if (rv < 0) {
			pr_err("Failed to read pftch context, rv = %d", rv);
			return rv;
		}
	}

	if ((st && c2h) || (!st && mm_cmpt_en)) {
		rv = qdma_cmpt_context_read(xdev, qid_hw, &context->cmpt_ctxt);
		if (rv < 0) {
			pr_err("Failed to read cmpt context, rv = %d", rv);
			return rv;
		}

	}

	return 0;
}

int qdma_intr_context_read(struct xlnx_dma_dev *xdev,
	int ring_index, struct qdma_indirect_intr_ctxt *ctxt)
{
	int rv = 0;

	memset(ctxt, 0, sizeof(struct qdma_indirect_intr_ctxt));
	rv = qdma_indirect_intr_context_read(xdev, ring_index, ctxt);
	if (rv < 0) {
		pr_err("Failed to read intr context, rv = %d", rv);
		return rv;
	}

	return 0;
}

int qdma_descq_context_program(struct xlnx_dma_dev *xdev, unsigned int qid_hw,
				bool st, bool c2h, bool mm_cmpt_en,
				struct qdma_descq_context *context)

{
	int rv;

	/* always clear first */
	rv = qdma_descq_context_clear(xdev, qid_hw, st, c2h, mm_cmpt_en, 1);
	if (rv < 0) {
		pr_err("failed to clear the context, rv= %d", rv);
		return rv;
	}

	rv = qdma_sw_context_write(xdev, c2h, qid_hw, &context->sw_ctxt);
	if (rv < 0) {
		pr_err("failed to program sw context, rv= %d", rv);
		return rv;
	}

	if (st && c2h) {
		/* prefetch context */
		rv = qdma_pfetch_context_write(xdev, qid_hw,
							&context->pfetch_ctxt);
		if (rv < 0) {
			pr_err("failed to program pfetch context, rv= %d",
			       rv);
			return rv;
		}
	}


	if ((st && c2h) || (!st && mm_cmpt_en)) {
		/* cmpt context */
		rv = qdma_cmpt_context_write(xdev, qid_hw, &context->cmpt_ctxt);
		if (rv < 0) {
			pr_err("failed to program cmpt context, rv= %d", rv);
			return rv;
		}
	}

	return 0;
}
#endif
