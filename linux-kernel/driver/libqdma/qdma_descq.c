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

#include "qdma_descq.h"

#include <linux/kernel.h>
#include <linux/delay.h>

#include "qdma_device.h"
#include "qdma_intr.h"
#include "qdma_regs.h"
#include "qdma_thread.h"
#include "qdma_context.h"
#include "qdma_st_c2h.h"
#include "qdma_access.h"
#include "thread.h"
#include "qdma_ul_ext.h"
#include "version.h"
#ifdef ERR_DEBUG
#include "qdma_nl.h"
#endif

#define H2C_DESC_MAX_DATA_LEN   PAGE_SIZE
/*
 * dma transfer requests
 */
#ifdef DEBUG
static void sgl_dump(struct qdma_sw_sg *sgl, unsigned int sgcnt)
{
	struct qdma_sw_sg *sg = sgl;
	int i;

	pr_info("sgl 0x%p, sgcntt %u.\n", sgl, sgcnt);

	for (i = 0; i < sgcnt; i++, sg++)
		pr_info("%d, 0x%p, pg 0x%p,%u+%u, dma 0x%llx.\n",
			i, sg, sg->pg, sg->offset, sg->len, sg->dma_addr);
}
#endif

int qdma_sgl_find_offset(struct qdma_request *req, struct qdma_sw_sg **sg_p,
			unsigned int *sg_offset)
{
	struct qdma_sgt_req_cb *cb = qdma_req_cb_get(req);
	struct qdma_sw_sg *sg = req->sgl;
	unsigned int sgcnt = req->sgcnt;
	unsigned int offset = cb->offset;
	unsigned int len = 0;
	int i;

	for (i = 0;  i < sgcnt; i++, sg++) {
		len += sg->len;

		if (len == offset) {
			*sg_p = sg + 1;
			*sg_offset = 0;
			++i;
			break;
		} else if (len > offset) {
			*sg_p = sg;
			*sg_offset = sg->len - (len - offset);
			break;
		}
	}

	if (i < sgcnt)
		return i;

	return -EINVAL;
}

void qdma_update_request(void *q_hndl, struct qdma_request *req,
			unsigned int num_desc,
			unsigned int data_cnt,
			unsigned int sg_offset,
			void *sg)
{
	struct qdma_descq *descq = (struct qdma_descq *)q_hndl;
	struct qdma_sgt_req_cb *cb = qdma_req_cb_get(req);

	cb->desc_nr += num_desc;
	cb->offset += data_cnt;
	cb->sg_offset = sg_offset;
	cb->sg = sg;
	if (cb->offset >= req->count) {
		list_del(&cb->list);
		list_add_tail(&cb->list, &descq->pend_list);
	}
}

static int descq_mm_n_h2c_cmpl_status(struct qdma_descq *descq);

static int descq_poll_mm_n_h2c_cmpl_status(struct qdma_descq *descq)
{
	enum qdma_drv_mode drv_mode = descq->xdev->conf.qdma_drv_mode;

	if ((drv_mode == POLL_MODE) || (drv_mode == AUTO_MODE)) {
		descq->proc_req_running = 1;
		return descq_mm_n_h2c_cmpl_status(descq);
	} else
		return 0;
}

static inline unsigned int incr_pidx(unsigned int pidx, unsigned int incr_val,
				     unsigned int ring_sz)
{
	pidx += incr_val;
	if (pidx >= ring_sz)
		pidx -= ring_sz;

	return pidx;
}

static ssize_t descq_mm_proc_request(struct qdma_descq *descq)
{
	int rv = 0;
	unsigned int desc_written = 0;
	unsigned int rngsz = descq->conf.rngsz;
	unsigned int pidx;
	struct qdma_mm_desc *desc;
	struct qdma_queue_conf *qconf = &descq->conf;
	unsigned char is_ul_ext = (qconf->desc_bypass &&
			qconf->fp_bypass_desc_fill) ? 1 : 0;

	lock_descq(descq);
	/* process completion of submitted requests */
	if (descq->q_stop_wait) {
		descq_mm_n_h2c_cmpl_status(descq);
		unlock_descq(descq);
		return 0;
	}
	if (unlikely(descq->q_state != Q_STATE_ONLINE)) {
		unlock_descq(descq);
		return 0;
	}

	if (descq->proc_req_running) {
		unlock_descq(descq);
		return 0;
	}

	pidx = descq->pidx;
	desc = (struct qdma_mm_desc *)descq->desc + pidx;

	descq_poll_mm_n_h2c_cmpl_status(descq);

	while (!list_empty(&descq->work_list)) {
		struct qdma_sgt_req_cb *cb = list_first_entry(&descq->work_list,
						struct qdma_sgt_req_cb, list);
		struct qdma_request *req = (struct qdma_request *)cb;
		struct qdma_sw_sg *sg = req->sgl;
		unsigned int sg_offset = 0;
		unsigned int sg_max = req->sgcnt;
		u64 ep_addr = req->ep_addr + cb->offset;
		struct qdma_mm_desc *desc_start = NULL;
		struct qdma_mm_desc *desc_end = NULL;
		unsigned int desc_max = descq->avail;
		unsigned int data_cnt = 0;
		unsigned int desc_cnt = 0;
		unsigned int len = 0;
		int i = 0;
		int rv;

		if (!desc_max) {
			descq_poll_mm_n_h2c_cmpl_status(descq);
			desc_max = descq->avail;
		}

		if (!desc_max)
			break;

		if (is_ul_ext) {
			int desc_consumed =
				qconf->fp_bypass_desc_fill(descq,
							   QDMA_Q_MODE_MM,
							   qconf->c2h ?
							   QDMA_Q_DIR_C2H :
							   QDMA_Q_DIR_H2C,
							   req);
			if (desc_consumed > 0)
				desc_cnt += desc_consumed;
			goto update_pidx;
		}
		rv = qdma_sgl_find_offset(req, &sg, &sg_offset);
		if (rv < 0) {
			pr_info("descq %s, req 0x%p, OOR %u/%u, %d/%u.\n",
				descq->conf.name, req, cb->offset,
				req->count, rv, sg_max);
			qdma_update_request(descq, req, 0, 0, 0, NULL);
			continue;
		}
		i = rv;
		pr_debug("%s, req 0x%p, offset %u/%u -> sg %d, 0x%p,%u.\n",
			descq->conf.name, req, cb->offset, req->count,
			i, sg, sg_offset);

		desc_start = desc;
		for (; i < sg_max && desc_cnt < desc_max; i++, sg++) {
			unsigned int tlen = sg->len;
			dma_addr_t addr = sg->dma_addr;
			unsigned int pg_off = sg->offset;

			pr_debug("desc %u/%u, sgl %d, len %u,%u, offset %u.\n",
				desc_cnt, desc_max, i, len, tlen, sg_offset);

			desc->flag_len = 0;
			if (sg_offset) {
				tlen -= sg_offset;
				addr += sg_offset;
				pg_off += sg_offset;
				sg_offset = 0;
			}

			do {
				unsigned int len = min_t(unsigned int, tlen,
							QDMA_DESC_BLEN_MAX);
				desc_end = desc;
				sg_offset += len;

				desc->rsvd1 = 0UL;
				desc->rsvd0 = 0U;

				if (descq->conf.c2h) {
					desc->src_addr = ep_addr;
					desc->dst_addr = addr;
				} else {
					desc->dst_addr = ep_addr;
					desc->src_addr = addr;
				}

				desc->flag_len = len;
				desc->flag_len |= (1 << S_DESC_F_DV);

				ep_addr += len;
				data_cnt += len;
				addr += len;
				tlen -= len;
				pg_off += len;

				if (++pidx == rngsz) {
					pidx = 0;
					desc =
					(struct qdma_mm_desc *)descq->desc;
				} else {
					desc++;
				}

				desc_cnt++;
				if (desc_cnt == desc_max)
					break;
			} while (tlen);
			if (!tlen)
				sg_offset = 0;
		}
		if (i == sg_max) {
			sg = NULL;
			sg_offset = 0;
		}

		if (!desc_end || !desc_start) {
			pr_info("descq %s, %u, pidx 0x%x, desc 0x%p ~ 0x%p.\n",
				descq->conf.name, descq->qidx_hw, pidx,
				desc_start, desc_end);
			break;
		}

		/* set eop */
		desc_end->flag_len |= (1 << S_DESC_F_EOP);
		/* set sop */
		desc_start->flag_len |= (1 << S_DESC_F_SOP);
		qdma_update_request(descq, req, desc_cnt, data_cnt, sg_offset,
				    sg);
		descq->pidx = pidx;
		descq->avail -= desc_cnt;
update_pidx:

		desc_written += desc_cnt;

		pr_debug("descq %s, +%u,%u, avail %u, ep_addr 0x%llx + 0x%x(%u).\n",
			descq->conf.name, desc_cnt, pidx, descq->avail,
			req->ep_addr, data_cnt, data_cnt);

		descq->pend_req_desc -= desc_cnt;
	}

	if (desc_written) {
		descq->pend_list_empty = 0;
		descq->pidx_info.pidx = descq->pidx;
		rv = queue_pidx_update(descq->xdev, descq->conf.qidx,
				descq->conf.c2h, &descq->pidx_info);
		if (rv < 0) {
			pr_err("%s: Failed to update pidx\n",
					descq->conf.name);
			unlock_descq(descq);
			return -EINVAL;
		}

		descq_poll_mm_n_h2c_cmpl_status(descq);
	}

	descq->proc_req_running = 0;
	unlock_descq(descq);

	if (desc_written && descq->cmplthp)
		qdma_kthread_wakeup(descq->cmplthp);

	return 0;
}

static ssize_t descq_proc_st_h2c_request(struct qdma_descq *descq)
{
	int ret = 0;
	struct qdma_h2c_desc *desc;
	unsigned int rngsz = descq->conf.rngsz;
	unsigned int pidx;
	unsigned int desc_written = 0;
	struct qdma_queue_conf *qconf = &descq->conf;
	unsigned char is_ul_ext = (qconf->desc_bypass &&
			qconf->fp_bypass_desc_fill) ? 1 : 0;


	lock_descq(descq);
	/* process completion of submitted requests */
	if (descq->q_stop_wait) {
		descq_mm_n_h2c_cmpl_status(descq);
		unlock_descq(descq);
		return 0;
	}
	if (unlikely(descq->q_state != Q_STATE_ONLINE)) {
		unlock_descq(descq);
		return 0;
	}

	if (descq->proc_req_running) {
		unlock_descq(descq);
		return 0;
	}

	/* service completion first */
	descq_poll_mm_n_h2c_cmpl_status(descq);

	pidx = descq->pidx;
	desc = (struct qdma_h2c_desc *)descq->desc + pidx;
	while (!list_empty(&descq->work_list)) {
		struct qdma_sgt_req_cb *cb = list_first_entry(&descq->work_list,
						struct qdma_sgt_req_cb, list);
		struct qdma_request *req = (struct qdma_request *)cb;
		struct qdma_sw_sg *sg = req->sgl;
		unsigned int sg_offset = 0;
		unsigned int sg_max = req->sgcnt;
		unsigned int desc_max = descq->avail;
		unsigned int data_cnt = 0;
		unsigned int desc_cnt = 0;
		unsigned int pktsz = req->ep_addr ?
				min_t(unsigned int, req->ep_addr, PAGE_SIZE) :
				PAGE_SIZE;
		int i = 0;
		int rv;

		if (!desc_max) {
			descq_poll_mm_n_h2c_cmpl_status(descq);
			desc_max = descq->avail;
		}

		if (!desc_max)
			break;

#ifdef DEBUG
		pr_info("%s, req %u.\n", descq->conf.name, req->count);
		sgl_dump(req->sgl, sg_max);
#endif
		if (is_ul_ext) {
			int desc_consumed =
				qconf->fp_bypass_desc_fill(descq,
							   QDMA_Q_MODE_ST,
							   QDMA_Q_DIR_H2C,
							   req);
			if (desc_consumed > 0)
				desc_cnt += desc_consumed;
			goto update_pidx;
		} else
			pr_warn("ul ext not set");

		rv = qdma_sgl_find_offset(req, &sg,
					      &sg_offset);

		if (rv < 0) {
			pr_err("descq %s, req 0x%p, OOR %u/%u, %d/%u.\n",
			descq->conf.name, req, cb->offset,
			req->count, rv, sg_max);
			qdma_update_request(descq, req, 0, 0, 0, NULL);
			continue;
		}
		i = rv;
		pr_debug("%s, req 0x%p, offset %u/%u -> sg %d, 0x%p,%u.\n",
			descq->conf.name, req, cb->offset, req->count,
			i, sg, sg_offset);
		desc->flags = 0;
		desc->cdh_flags = 0;
		if (i == 0)
			desc->flags |= S_H2C_DESC_F_SOP;

		for (; i < sg_max && desc_cnt < desc_max; i++, sg++) {
			unsigned int tlen = sg->len;
			dma_addr_t addr = sg->dma_addr;

			if (sg_offset) {
				tlen -= sg_offset;
				addr += sg_offset;
				sg_offset = 0;
			}

			do { /* to support zero byte transfer */
				unsigned int len = min_t(unsigned int, tlen,
							 pktsz);

				sg_offset += len;
				desc->src_addr = addr;
				desc->len = len;
				desc->pld_len = len;
				desc->cdh_flags |= S_H2C_DESC_F_ZERO_CDH;

				data_cnt += len;
				addr += len;
				tlen -= len;

				if ((i == sg_max - 1))
					desc->flags |= S_H2C_DESC_F_EOP;

#if 0
				pr_info("desc %d, pidx 0x%x, data_cnt %u, cb off %u:\n",
					i, pidx, data_cnt, cb->offset);
				print_hex_dump(KERN_INFO, "desc",
					       DUMP_PREFIX_OFFSET, 16, 1,
					       (void *)desc, 16, false);
#endif

				if (++pidx == rngsz) {
					pidx = 0;
					desc =
					(struct qdma_h2c_desc *)descq->desc;
				} else {
					desc++;
					desc->flags = 0;
					desc->cdh_flags = 0;
				}

				desc_cnt++;
				if (desc_cnt == desc_max)
					break;
			} while (tlen);
			if (!tlen)
				sg_offset = 0;

		}
		if (i == sg_max) {
			sg = NULL;
			sg_offset = 0;
		}
		qdma_update_request(descq, req, desc_cnt, data_cnt, sg_offset,
				    sg);
		descq->pidx = pidx;
		descq->avail -= desc_cnt;
update_pidx:
		if (!desc_cnt)
			break;
		desc_written += desc_cnt;

		pr_debug("descq %s, +%u,%u, avail %u, 0x%x(%u), cb off %u.\n",
			descq->conf.name, desc_cnt, pidx, descq->avail,
			data_cnt, data_cnt, cb->offset);

		descq->pend_req_desc -= desc_cnt;
	}

	if (desc_written) {
		descq->pend_list_empty = 0;
		descq->pidx_info.pidx = descq->pidx;
		ret = queue_pidx_update(descq->xdev, descq->conf.qidx,
				descq->conf.c2h, &descq->pidx_info);
		if (ret < 0) {
			pr_err("%s: Failed to update pidx\n",
					descq->conf.name);
			unlock_descq(descq);
			return -EINVAL;
		}

		descq_poll_mm_n_h2c_cmpl_status(descq);
	}

	descq->proc_req_running = 0;

	if (desc_written && descq->cmplthp)
		qdma_kthread_wakeup(descq->cmplthp);

	unlock_descq(descq);

	return 0;
}

/*
 * descriptor Queue
 */
static inline int get_desc_size(struct qdma_descq *descq)
{
	if (descq->conf.desc_bypass && (descq->conf.sw_desc_sz == DESC_SZ_64B))
		return DESC_SZ_64B_BYTES;
	if (!descq->conf.st)
		return (int)sizeof(struct qdma_mm_desc);

	if (descq->conf.c2h)
		return (int)sizeof(struct qdma_c2h_desc);

	return (int)sizeof(struct qdma_h2c_desc);
}

static inline int get_desc_cmpl_status_size(struct qdma_descq *descq)
{
	return (int)sizeof(struct qdma_desc_cmpl_status);
}

static inline void desc_ring_free(struct xlnx_dma_dev *xdev, int ring_sz,
			int desc_sz, int cs_sz, u8 *desc, dma_addr_t desc_bus)
{
	unsigned int len = ring_sz * desc_sz + cs_sz;

	pr_debug("free %u(0x%x)=%d*%u+%d, 0x%p, bus 0x%llx.\n",
		len, len, desc_sz, ring_sz, cs_sz, desc, desc_bus);

	dma_free_coherent(&xdev->conf.pdev->dev, ring_sz * desc_sz + cs_sz,
			desc, desc_bus);
}

static void *desc_ring_alloc(struct xlnx_dma_dev *xdev, int ring_sz,
			int desc_sz, int cs_sz, dma_addr_t *bus, u8 **cs_pp)
{
	unsigned int len = ring_sz * desc_sz + cs_sz;
	u8 *p = dma_alloc_coherent(&xdev->conf.pdev->dev, len, bus, GFP_KERNEL);

	if (!p) {
		pr_info("%s, OOM, sz ring %d, desc %d, cmpl status sz %d.\n",
			xdev->conf.name, ring_sz, desc_sz, cs_sz);
		return NULL;
	}

	*cs_pp = p + ring_sz * desc_sz;
	memset(p, 0, len);

	pr_debug("alloc %u(0x%x)=%d*%u+%d, 0x%p, bus 0x%llx, cmpl status 0x%p.\n",
		len, len, desc_sz, ring_sz, cs_sz, p, *bus, *cs_pp);

	return p;
}

static void desc_alloc_irq(struct qdma_descq *descq)
{
	struct xlnx_dma_dev *xdev = descq->xdev;
	unsigned long flags;
	int i, idx = 0, min = -1;

	if (!xdev->num_vecs)
		return;

	/** Pick the MSI-X vector that currently has the fewest queues
	 * on PF0, vector#0 is dedicated for Error interrupts and
	 * vector #1 is dedicated for User interrupts
	 * For all other PFs, vector#0 is dedicated for User interrupts
	 */

	idx = xdev->dvec_start_idx;
	if (xdev->conf.qdma_drv_mode == DIRECT_INTR_MODE) {
		for (i = xdev->dvec_start_idx; i < xdev->num_vecs; i++) {
			struct intr_info_t *intr_info_list =
					&xdev->dev_intr_info_list[i];

			spin_lock_irqsave(&intr_info_list->vec_q_list,
					flags);
			if (!intr_info_list->intr_list_cnt) {
				spin_unlock_irqrestore(
						&intr_info_list->vec_q_list,
						flags);
				idx = i;
				break;
			}
			if (min < 0)
				min = intr_info_list->intr_list_cnt;
			if (intr_info_list->intr_list_cnt < min) {
				min = intr_info_list->intr_list_cnt;
				idx = i;
			}

			spin_unlock_irqrestore(&intr_info_list->vec_q_list,
					flags);
		}
	}
	descq->intr_id = idx;
	pr_debug("descq->intr_id = %d allocated to qidx = %d\n",
		descq->intr_id, descq->conf.qidx);
}

/*
 * writeback handling
 */
static int descq_mm_n_h2c_cmpl_status(struct qdma_descq *descq)
{
	int rv = 0;
	unsigned int cidx, cidx_hw;
	unsigned int cr;

	pr_debug("descq 0x%p, %s, pidx %u, cidx %u.\n",
		descq, descq->conf.name, descq->pidx, descq->cidx);

	if (descq->pidx == descq->cidx) { /* queue empty? */
		pr_debug("descq %s empty, return.\n", descq->conf.name);
		return 0;
	}

	cidx = descq->cidx;
#ifdef __READ_ONCE_DEFINED__
	cidx_hw = READ_ONCE(((struct qdma_desc_cmpl_status *)
				descq->desc_cmpl_status)->cidx);
#else
	cidx_hw = ((struct qdma_desc_cmpl_status *)
					descq->desc_cmpl_status)->cidx;
#endif

	if (cidx_hw == cidx) /* no new writeback? */
		return 0;

	/* completion credits */
	cr = (cidx_hw < cidx) ? (descq->conf.rngsz - cidx) + cidx_hw :
				cidx_hw - cidx;

	pr_debug("%s descq %s, cidx 0x%x -> 0x%x, avail 0x%x + 0x%x.\n",__func__,
		descq->conf.name, cidx, cidx_hw, descq->avail, cr);

	descq->cidx = cidx_hw;
	descq->avail += cr;
	descq->credit += cr;

	incr_cmpl_desc_cnt(descq, cr);

	/* completes requests */
	pr_debug("%s %s, 0x%p, credit %u + %u.\n",__func__,descq->conf.name, descq, cr, descq->credit);

	cr = descq->credit;

	while (!list_empty(&descq->pend_list)) {
		struct qdma_sgt_req_cb *cb = list_first_entry(&descq->pend_list,
						struct qdma_sgt_req_cb, list);

		pr_debug("%s, 0x%p, cb 0x%p, desc_nr %u, credit %u.\n",
			descq->conf.name, descq, cb, cb->desc_nr, cr);

		if (cr >= cb->desc_nr) {
			pr_debug("%s, cb 0x%p done, credit %u > %u.\n",
				descq->conf.name, cb, cr, cb->desc_nr);
			cr -= cb->desc_nr;
			qdma_sgt_req_done(descq, cb, 0);
		} else {
			pr_debug("%s, cb 0x%p not done, credit %u < %u.\n",
				descq->conf.name, cb, cr, cb->desc_nr);
			cb->desc_nr -= cr;
			cr = 0;
		}

		if (!cr)
			break;
	}

	descq->credit = cr;
	pr_debug("%s, 0x%p, credit %u.\n",
		descq->conf.name, descq, descq->credit);
	rv = queue_pidx_update(descq->xdev, descq->conf.qidx,
			descq->conf.c2h, &descq->pidx_info);
	if (rv < 0) {
		pr_err("%s: Failed to update pidx\n", descq->conf.name);
		return -EINVAL;
	}

	/* Request thread may have only setup a fraction of the transfer (e.g.
	 * there wasn't enough space in desc ring). We now have more space
	 * available again so we can continue programming the
	 * dma transfer by resuming the thread here.
	 */


	return 1;
}

/* ************** public function definitions ******************************* */

void qdma_descq_init(struct qdma_descq *descq, struct xlnx_dma_dev *xdev,
			int idx_hw, int idx_sw)
{
	struct qdma_dev *qdev = xdev_2_qdev(xdev);

	memset(descq, 0, sizeof(struct qdma_descq));

	spin_lock_init(&descq->lock);
	INIT_LIST_HEAD(&descq->work_list);
	INIT_LIST_HEAD(&descq->pend_list);
	qdma_waitq_init(&descq->pend_list_wq);
	INIT_LIST_HEAD(&descq->intr_list);
	INIT_LIST_HEAD(&descq->legacy_intr_q_list);
	INIT_WORK(&descq->work, intr_work);
	descq->xdev = xdev;
	descq->channel = 0;
	descq->qidx_hw = qdev->qbase + idx_hw;
	descq->conf.qidx = idx_sw;
}

int qdma_q_desc_get(void *q_hndl, const unsigned int desc_cnt,
		    struct qdma_q_desc_list **desc_list)
{
	struct qdma_descq *descq = (struct qdma_descq *)q_hndl;

	if (desc_cnt >= descq->conf.rngsz)
		return -EINVAL; /* not possible to give so many desc */
	if (descq->avail < desc_cnt)
		return -EBUSY; /* curently not available */

	*desc_list = descq->desc_list + descq->pidx;
	descq->pidx = incr_pidx(descq->pidx,
				desc_cnt,
				descq->conf.rngsz);
	descq->avail -= desc_cnt;

	return 0;
}

int qdma_q_init_pointers(void *q_hndl)
{
	struct qdma_descq *descq = (struct qdma_descq *)q_hndl;
	int rv;

	if ((descq->conf.st && descq->conf.c2h) ||
		(!descq->conf.st && descq->mm_cmpt_ring_crtd)) {
		if (descq->conf.init_pidx_dis) { /* use qdma_q_init_pointers
						for updating the pidx and cidx
						later*/
			descq->cmpt_cidx_info.wrb_cidx = 0;
			rv = queue_cmpt_cidx_update(descq->xdev,
					descq->conf.qidx,
					&descq->cmpt_cidx_info);
			if (rv < 0) {
				pr_err("%s: Failed to update cmpt cidx\n",
						descq->conf.name);
				return -EINVAL;
			}

			descq->pidx_info.pidx = descq->conf.rngsz - 1;
			rv = queue_pidx_update(descq->xdev, descq->conf.qidx,
					descq->conf.c2h, &descq->pidx_info);
			if (rv < 0) {
				pr_err("%s: Failed to update pidx\n",
						descq->conf.name);
				return -EINVAL;
			}
		} else
			return -EINVAL;
	} else
		return -EINVAL;

	return 0;
}

int is_mm_cmpl_required(struct qdma_descq *descq)
{
	struct xlnx_dma_dev *xdev = descq->xdev;
	struct qdma_dev *qdev = xdev_2_qdev(xdev);
	struct qdma_descq *pairq;
	int mml_cmpl_needed = 0;

	if (!descq->conf.st && descq->conf.en_mm_cmpt) {
		if (xdev->dev_cap.mm_cmpt_en) {
			if (descq->conf.c2h)
				pairq = qdev->h2c_descq + descq->conf.qidx;
			else
				pairq = qdev->c2h_descq + descq->conf.qidx;

			/** Check if the completion ring needs to be created
			 *  completion ring is created only if the current
			 *  HW supports MM completions and ring not already
			 *  created in queue pair
			 */

			lock_descq(pairq);
			if (pairq->q_state != Q_STATE_ONLINE) {
			/**
			 * if Pairq q state is disabled, i.e pair queue is not
			 * already created then, when mm_cmpl is requested
			 * on present queue, go ahead and create it
			 * If Pairq is added but not started yet,
			 * i.e in enabled state then also, go ahead and create
			 * the cmpt ring if mm_cmpl is requested As,
			 * when pairq start is called, present queue will be in
			 * online state and cmpt ring would be already created
			 */
				mml_cmpl_needed = 1;
			} else {
				if (pairq->conf.en_mm_cmpt &&
					    pairq->mm_cmpt_ring_crtd) {
					pr_debug("pairq->q_state is online and cmpt already created");
					mml_cmpl_needed = 0;
				} else {
					pr_debug("pairq->q_state is online but cmpt is not created");
					mml_cmpl_needed = 1;
				}
			}
			unlock_descq(pairq);
		} else {
			pr_warn("MM Completion capability not supported");
			mml_cmpl_needed = 0;
		}
	}

	return mml_cmpl_needed;
}

int qdma_descq_alloc_resource(struct qdma_descq *descq)
{
	struct xlnx_dma_dev *xdev = descq->xdev;
	struct qdma_queue_conf *qconf = &descq->conf;
	unsigned char is_ul_ext = (qconf->desc_bypass &&
			qconf->fp_bypass_desc_fill) ? 1 : 0;
	int rv;
#ifdef TEST_64B_DESC_BYPASS_FEATURE
	int i = 0;
	u8 *desc_bypass;
	u8 bypass_data[DESC_SZ_64B_BYTES];
#endif
	/* descriptor ring */
	descq->desc = desc_ring_alloc(xdev, descq->conf.rngsz,
				get_desc_size(descq),
				get_desc_cmpl_status_size(descq),
				&descq->desc_bus, &descq->desc_cmpl_status);
	if (!descq->desc) {
		pr_info("dev %s, descq %s, sz %u, desc ring OOM.\n",
			xdev->conf.name, descq->conf.name, descq->conf.rngsz);
		goto err_out;
	}

	if (descq->conf.st && descq->conf.c2h) {
		struct qdma_flq *flq = (struct qdma_flq *)descq->flq;

		flq->desc = (struct qdma_c2h_desc *)descq->desc;
		flq->size = descq->conf.rngsz;
		flq->pg_shift = fls(descq->conf.c2h_bufsz) - 1;

		/* These code changes are to accomodate buf_sz
		 *  of less than 4096
		 */
		if (flq->pg_shift < PAGE_SHIFT)
			flq->pg_order = 0;
		else
			flq->pg_order = flq->pg_shift - PAGE_SHIFT;

		/* freelist / rx buffers */
		rv = descq_flq_alloc_resource(descq);
		if (rv < 0)
			goto err_out;
	} else if (is_ul_ext) {
		int i;
		unsigned int desc_sz = get_desc_size(descq);

		descq->desc_list = kcalloc(descq->conf.rngsz,
					   sizeof(struct qdma_q_desc_list),
					   GFP_KERNEL);
		if (!descq->desc_list) {
			pr_err("desc_list allocation failed.OOM");
			goto err_out;
		}
		for (i = 0; i < descq->conf.rngsz; i++) {
			int next = (i == (descq->conf.rngsz - 1)) ? 0 : (i + 1);

			descq->desc_list[i].desc = descq->desc + (i * desc_sz);
			descq->desc_list[i].next = &descq->desc_list[next];
		}
	}

	if ((descq->conf.st && descq->conf.c2h) ||
		    (!descq->conf.st && descq->mm_cmpt_ring_crtd)) {

		descq->color = 1;

		/* writeback ring */
		descq->desc_cmpt = desc_ring_alloc(xdev,
					descq->conf.rngsz_cmpt,
					descq->cmpt_entry_len,
					sizeof(struct
					       qdma_c2h_cmpt_cmpl_status),
					&descq->desc_cmpt_bus,
					&descq->desc_cmpt_cmpl_status);
		if (!descq->desc_cmpt) {
			pr_warn("dev %s, descq %s, sz %u, cmpt ring OOM.\n",
				xdev->conf.name, descq->conf.name,
				descq->conf.rngsz_cmpt);
			goto err_out;
		}
		descq->desc_cmpt_cur = descq->desc_cmpt;

	}

	pr_debug("%s: %u/%u, rng %u,%u, desc 0x%p, cmpl status 0x%p.\n",
		descq->conf.name, descq->conf.qidx, descq->qidx_hw,
		descq->conf.rngsz, descq->conf.rngsz_cmpt, descq->desc,
		descq->desc_cmpt);

	/* interrupt vectors */
	desc_alloc_irq(descq);

	/* Fill in the descriptors with some hard coded value for testing */
#ifdef TEST_64B_DESC_BYPASS_FEATURE
	desc_bypass = descq->desc;
	if (descq->conf.st && !(descq->conf.c2h)) {
		if (descq->conf.desc_bypass &&
				(descq->conf.sw_desc_sz == DESC_SZ_64B)) {
			for (i = 0; i < descq->conf.rngsz; i++) {
				memset(bypass_data, i+1, DESC_SZ_64B_BYTES);
				memcpy(&desc_bypass[i*DESC_SZ_64B_BYTES],
					bypass_data, DESC_SZ_64B_BYTES);
			}
		}
	}
#endif
	return 0;

err_out:
	qdma_descq_free_resource(descq);
	return QDMA_ERR_OUT_OF_MEMORY;
}

void qdma_descq_free_resource(struct qdma_descq *descq)
{
	if (!descq)
		return;

	if (descq->desc) {

		int desc_sz = get_desc_size(descq);
		int cs_sz = get_desc_cmpl_status_size(descq);

		pr_debug("%s: desc 0x%p, cmpt 0x%p.\n",
			descq->conf.name, descq->desc, descq->desc_cmpt);

		if (descq->conf.st && descq->conf.c2h)
			descq_flq_free_resource(descq);
		else
			kfree(descq->desc_list);

		desc_ring_free(descq->xdev, descq->conf.rngsz, desc_sz, cs_sz,
				descq->desc, descq->desc_bus);
		descq->desc_cmpl_status = NULL;
		descq->desc = NULL;
		descq->desc_bus = 0UL;
	}

	if (descq->desc_cmpt) {
		desc_ring_free(descq->xdev, descq->conf.rngsz_cmpt,
			descq->cmpt_entry_len,
			sizeof(struct qdma_c2h_cmpt_cmpl_status),
			descq->desc_cmpt, descq->desc_cmpt_bus);

		descq->desc_cmpt_cmpl_status = NULL;
		descq->desc_cmpt = NULL;
		descq->desc_cmpt_bus = 0UL;
	}
}

void qdma_descq_config(struct qdma_descq *descq, struct qdma_queue_conf *qconf,
		 int reconfig)
{
	if (!reconfig) {
		int len;

		memcpy(&descq->conf, qconf, sizeof(struct qdma_queue_conf));
		/* descq->conf.st = qconf->st;
		 * descq->conf.c2h = qconf->c2h;
		 */

		/* qdma[vf]<255>-MM/ST-H2C/C2H-Q[2048] */
#ifdef __QDMA_VF__
		len = sprintf(descq->conf.name, "qdmavf");
#else
		len = sprintf(descq->conf.name, "qdma");
#endif
		len += sprintf(descq->conf.name + len, "%05x-%s-%u",
			descq->xdev->conf.bdf, descq->conf.st ? "ST" : "MM",
			descq->conf.qidx);
		descq->conf.name[len] = '\0';

		descq->conf.st = qconf->st;
		descq->conf.c2h = qconf->c2h;

	} else {
		descq->conf.desc_rng_sz_idx = qconf->desc_rng_sz_idx;
		descq->conf.cmpl_rng_sz_idx = qconf->cmpl_rng_sz_idx;
		descq->conf.c2h_buf_sz_idx = qconf->c2h_buf_sz_idx;

		descq->conf.irq_en = (descq->xdev->conf.qdma_drv_mode !=
				POLL_MODE) ? 1 : 0;
		descq->conf.cmpl_en_intr = descq->conf.irq_en;
		descq->conf.wb_status_en = qconf->wb_status_en;
		descq->conf.cmpl_status_acc_en = qconf->cmpl_status_acc_en;
		descq->conf.cmpl_status_pend_chk = qconf->cmpl_status_pend_chk;
		descq->conf.cmpl_stat_en = qconf->cmpl_stat_en;
		descq->conf.cmpl_trig_mode = qconf->cmpl_trig_mode;
		descq->conf.cmpl_timer_idx = qconf->cmpl_timer_idx;
		descq->conf.fetch_credit = qconf->fetch_credit;
		descq->conf.cmpl_cnt_th_idx = qconf->cmpl_cnt_th_idx;

		descq->conf.desc_bypass = qconf->desc_bypass;
		descq->conf.pfetch_bypass = qconf->pfetch_bypass;
		descq->conf.pfetch_en = qconf->pfetch_en;
		descq->conf.cmpl_udd_en = qconf->cmpl_udd_en;
		descq->conf.cmpl_desc_sz = qconf->cmpl_desc_sz;
		descq->conf.sw_desc_sz = qconf->sw_desc_sz;
		descq->conf.pipe_gl_max = qconf->pipe_gl_max;
		descq->conf.pipe_flow_id = qconf->pipe_flow_id;
		descq->conf.pipe_slr_id = qconf->pipe_slr_id;
		descq->conf.pipe_tdest = qconf->pipe_tdest;
		descq->conf.cmpl_ovf_chk_dis = qconf->cmpl_ovf_chk_dis;
		descq->conf.en_mm_cmpt = qconf->en_mm_cmpt;
		descq->conf.adaptive_rx = qconf->adaptive_rx;
	}
}

int qdma_descq_config_complete(struct qdma_descq *descq)
{
	struct global_csr_conf *csr_info = &descq->xdev->csr_info;
	struct qdma_queue_conf *qconf = &descq->conf;
	struct xlnx_dma_dev *xdev = descq->xdev;

	qconf->rngsz = csr_info->ring_sz[qconf->desc_rng_sz_idx] - 1;

	/* <= 2018.2 IP
	 * make the cmpl ring size bigger if possible to avoid run out of
	 * cmpl entry while desc. ring still have free entries
	 */
	if ((qconf->st && qconf->c2h) ||
			(!qconf->st && qconf->en_mm_cmpt)) {
		int i;
		unsigned int v = csr_info->ring_sz[qconf->cmpl_rng_sz_idx];
		int best_fit_idx = -1;

		for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
			if (csr_info->ring_sz[i] > v) {
				if (best_fit_idx < 0)
					best_fit_idx = i;
				else if ((best_fit_idx >= 0) &&
						(csr_info->ring_sz[i] <
					csr_info->ring_sz[best_fit_idx]))
					best_fit_idx = i;
			}
		}

		if (best_fit_idx >= 0)
			qconf->cmpl_rng_sz_idx = best_fit_idx;

		qconf->rngsz_cmpt = csr_info->ring_sz[qconf->cmpl_rng_sz_idx] -
				1;
		qconf->c2h_bufsz = csr_info->c2h_buf_sz[qconf->c2h_buf_sz_idx];
		descq->cmpt_cidx_info.irq_en = qconf->cmpl_en_intr;
		descq->cmpt_cidx_info.trig_mode = qconf->cmpl_trig_mode;
		descq->cmpt_cidx_info.timer_idx = qconf->cmpl_timer_idx;
		descq->cmpt_cidx_info.counter_idx = qconf->cmpl_cnt_th_idx;
		descq->cmpt_cidx_info.wrb_en = qconf->cmpl_stat_en;
	}
	if (qconf->st && qconf->c2h)
		descq->pidx_info.irq_en = 0;
	else
		descq->pidx_info.irq_en = descq->conf.irq_en;

	/* we can never use the full ring because then cidx would equal pidx
	 * and thus the ring would be interpreted as empty. Thus max number of
	 * usable entries is ring_size - 1
	 */
	descq->avail = descq->conf.rngsz - 1;
	descq->pend_list_empty = 1;

	descq->pidx = 0;
	descq->cidx = 0;
	descq->cidx_cmpt = 0;
	descq->pidx_cmpt = 0;
	descq->credit = 0;
	descq->pend_req_desc = 0;

	descq->mm_cmpt_ring_crtd = is_mm_cmpl_required(descq);

	/* ST C2H only */
	if ((qconf->c2h && qconf->st) ||
			(!qconf->st && descq->mm_cmpt_ring_crtd)) {
		int i;

		descq->cmpt_entry_len = 8 << qconf->cmpl_desc_sz;

		pr_debug("%s: cmpl sz %u(%d), udd_en %d.\n",
			descq->conf.name, descq->cmpt_entry_len,
			descq->conf.cmpl_desc_sz, descq->conf.cmpl_udd_en);

		descq->c2h_pend_pkt_moving_avg =
				csr_info->c2h_cnt_th[descq->conf.
						     cmpl_cnt_th_idx];
		for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
			if (descq->conf.cmpl_cnt_th_idx ==
					descq->xdev->sorted_c2h_cntr_idx[i]) {
				descq->sorted_c2h_cntr_idx = i;
				break;
			}
		}
		i = xdev->sorted_c2h_cntr_idx[descq->sorted_c2h_cntr_idx + 1];
		descq->c2h_pend_pkt_avg_thr_hi =
				(descq->c2h_pend_pkt_moving_avg +
				csr_info->c2h_cnt_th[i]);
		i = xdev->sorted_c2h_cntr_idx[descq->sorted_c2h_cntr_idx - 1];
		descq->c2h_pend_pkt_avg_thr_lo =
				(descq->c2h_pend_pkt_moving_avg +
				csr_info->c2h_cnt_th[i]);
		descq->c2h_pend_pkt_avg_thr_hi >>= 1;
		descq->c2h_pend_pkt_avg_thr_lo >>= 1;
		pr_debug("q%u: sorted idx =  %u %u %u", descq->conf.qidx,
			descq->sorted_c2h_cntr_idx, descq->c2h_pend_pkt_avg_thr_lo, descq->c2h_pend_pkt_avg_thr_hi);
	}

	return 0;
}

int qdma_descq_prog_hw(struct qdma_descq *descq)
{
	int rv = qdma_descq_context_setup(descq);

	if (rv < 0) {
		pr_warn("%s failed to program contexts", descq->conf.name);
		return rv;
	}

	/* update pidx/cidx */
	if ((descq->conf.st && descq->conf.c2h) ||
		(!descq->conf.st && descq->mm_cmpt_ring_crtd)) {
		if (!descq->conf.init_pidx_dis) { /* use qdma_q_init_pointers
		 	 	 	 	for updating the pidx and cidx
		 	 	 	 	later*/
			descq->cmpt_cidx_info.wrb_cidx = 0;
			rv = queue_cmpt_cidx_update(descq->xdev,
					descq->conf.qidx,
					&descq->cmpt_cidx_info);
			if (rv < 0) {
				pr_err("%s: Failed to update cmpt cidx\n",
						descq->conf.name);
				return -EINVAL;
			}

			descq->pidx_info.pidx = descq->conf.rngsz - 1;
			rv = queue_pidx_update(descq->xdev, descq->conf.qidx,
					descq->conf.c2h, &descq->pidx_info);
			if (rv < 0) {
				pr_err("%s: Failed to update pidx\n",
						descq->conf.name);
				return -EINVAL;
			}
		}
	}

	return rv;
}

void qdma_descq_service_cmpl_update(struct qdma_descq *descq, int budget,
				bool c2h_upd_cmpl)
{
	if (descq->conf.st && descq->conf.c2h) {
		lock_descq(descq);
		if (descq->q_state == Q_STATE_ONLINE) {
			pr_debug("st-c2h for qid%u", descq->conf.qidx);
			descq_process_completion_st_c2h(descq, budget,
							c2h_upd_cmpl);
			pr_debug("completed st-c2h for qid%u", descq->conf.qidx);
		}
		unlock_descq(descq);
	} else if ((descq->xdev->conf.qdma_drv_mode == POLL_MODE) ||
			(descq->xdev->conf.qdma_drv_mode == AUTO_MODE)) {
		if (!descq->proc_req_running)
 			qdma_descq_proc_sgt_request(descq);
	} else {
		lock_descq(descq);
		descq_mm_n_h2c_cmpl_status(descq);
		pr_debug(" streaming H2C Completion");
		if (descq->pend_req_desc) {
			unlock_descq(descq);
			qdma_descq_proc_sgt_request(descq);
			return;
		}
		unlock_descq(descq);
	}
}


ssize_t qdma_descq_proc_sgt_request(struct qdma_descq *descq)
{
	if (!descq->conf.st) /* MM H2C/C2H */
		return descq_mm_proc_request(descq);
	else if (descq->conf.st && !descq->conf.c2h) /* ST H2C */
		return descq_proc_st_h2c_request(descq);
	else	/* ST C2H - should not happen - handled separately */
		return -1;
}

void incr_cmpl_desc_cnt(struct qdma_descq *descq, unsigned int cnt)
{
	descq->total_cmpl_descs += cnt;
	switch ((descq->conf.st << 1) | descq->conf.c2h) {
	case 0:
		descq->xdev->total_mm_h2c_pkts += cnt;
		break;
	case 1:
		descq->xdev->total_mm_c2h_pkts += cnt;
		break;
	case 2:
		descq->xdev->total_st_h2c_pkts += cnt;
		break;
	case 3:
		descq->xdev->total_st_c2h_pkts += cnt;
		break;
	default:
		break;
	}
}

void qdma_sgt_req_done(struct qdma_descq *descq, struct qdma_sgt_req_cb *cb,
			int error)
{
	struct qdma_request *req = (struct qdma_request *)cb;

	if (error)
		pr_info("req 0x%p, cb 0x%p, fp_done 0x%p done, err %d.\n",
			req, cb, req->fp_done, error);

	list_del(&cb->list);
	if (cb->unmap_needed) {
		sgl_unmap(descq->xdev->conf.pdev, req->sgl, req->sgcnt,
			descq->conf.c2h ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
		cb->unmap_needed = 0;
	}
	if (req->fp_done) {
		if ((cb->offset != req->count) &&
				!(descq->conf.st && descq->conf.c2h)) {
			pr_info("req 0x%p not completed %u != %u.\n",
				req, cb->offset, req->count);
			error = -EINVAL;
		}
		cb->status = error;
		cb->done = 1;
		req->fp_done(req, cb->offset, error);
	} else {
		pr_debug("req 0x%p, cb 0x%p, wake up.\n", req, cb);
		cb->status = error;
		cb->done = 1;
		qdma_waitq_wakeup(&cb->wq);
	}

	if (descq->conf.st && descq->conf.c2h)
		descq->pend_list_empty = (descq->avail == 0);
	else
		descq->pend_list_empty = (descq->avail ==
				(descq->conf.rngsz - 1));

	if (descq->q_stop_wait && descq->pend_list_empty)
		qdma_waitq_wakeup(&descq->pend_list_wq);
}

int qdma_descq_dump_desc(struct qdma_descq *descq, int start,
			int end, char *buf, int buflen)
{
	struct qdma_flq *flq = (struct qdma_flq *)descq->flq;
	int desc_sz = get_desc_size(descq);
	u8 *p = descq->desc + start * desc_sz;
	struct qdma_sw_sg *fl = (descq->conf.st && descq->conf.c2h) ?
				flq->sdesc + start : NULL;
	int i = start;
	int len = strlen(buf);

	if (!descq->desc)
		return 0;

	for (; i < end && i < descq->conf.rngsz; i++, p += desc_sz) {
		len += sprintf(buf + len, "%d: 0x%p ", i, p);
		hex_dump_to_buffer(p, desc_sz,
				  (desc_sz < DESC_SZ_16B_BYTES) ? 16 : 32,
				  4, buf + len, buflen - len, 0);
		len = strlen(buf);
		if (desc_sz > DESC_SZ_32B_BYTES) {
			len += sprintf(buf + len, " ");
			hex_dump_to_buffer(p + DESC_SZ_32B_BYTES, desc_sz,
					32, 4, buf + len, buflen - len, 0);
			len = strlen(buf);
		}
		if (fl) {
			len += sprintf(buf + len, " fl pg 0x%p, 0x%llx.\n",
				fl->pg, fl->dma_addr);
			fl++;
		} else
			buf[len++] = '\n';
	}

	p = descq->desc_cmpl_status;

	dma_rmb();

	len += sprintf(buf + len, "CMPL STATUS: 0x%p ", p);
	hex_dump_to_buffer(p, get_desc_cmpl_status_size(descq), 16, 4,
			buf + len, buflen - len, 0);
	len = strlen(buf);
	buf[len++] = '\n';

	if (descq->conf.st && descq->conf.c2h) {
		p = page_address(fl->pg);
		len += sprintf(buf + len, "data 0: 0x%p ", p);
		hex_dump_to_buffer(p, descq->cmpt_entry_len,
			(descq->cmpt_entry_len < DESC_SZ_16B_BYTES) ? 16 : 32,
			4, buf + len,
			buflen - len, 0);
		len = strlen(buf);
		if (descq->cmpt_entry_len > DESC_SZ_32B_BYTES) {
			len += sprintf(buf + len, " ");
			hex_dump_to_buffer(p + DESC_SZ_32B_BYTES,
					descq->cmpt_entry_len, 32, 4,
					buf + len, buflen - len, 0);
			len = strlen(buf);
		}
		buf[len++] = '\n';
	}

	return len;
}

int qdma_descq_dump_cmpt(struct qdma_descq *descq, int start,
			int end, char *buf, int buflen)
{
	uint8_t *cmpt = descq->desc_cmpt;
	u8 *p;
	int i = start;
	int len = strlen(buf);
	int stride = descq->cmpt_entry_len;
	struct qdma_descq *pairq;
	char *cur = buf;
	char * const buf_end = buf + buflen;
	struct qdma_dev *qdev = xdev_2_qdev(descq->xdev);
	int  rv = 0;

	if (!descq->desc_cmpt)
		return 0;

	if (!descq->conf.st) {
		if (descq->conf.c2h)
			pairq = qdev->h2c_descq + descq->conf.qidx;
		else
			pairq = qdev->c2h_descq + descq->conf.qidx;

		if (!((descq->conf.en_mm_cmpt ||
			pairq->conf.en_mm_cmpt) &&
			(descq->mm_cmpt_ring_crtd ||
			pairq->mm_cmpt_ring_crtd))) {
			pr_err("MM Completion not enabled for this queue");
			rv = QDMA_ERR_INVALID_INPUT_PARAM;
			cur += snprintf(cur, buf_end - cur,
					"%s, MM Completion not enabled for this queue\n",
					descq->xdev->conf.name);
			if (cur >= buf_end) {
				*buf = '\0';
				return rv;
			}
			return rv;
		}

		if (!descq->mm_cmpt_ring_crtd && pairq->mm_cmpt_ring_crtd)
			descq = pairq;
	}

	for (cmpt += (start * stride);
			i < end && i < descq->conf.rngsz_cmpt; i++,
			cmpt += stride) {
		len += sprintf(buf + len, "%d: 0x%p ", i, cmpt);
		hex_dump_to_buffer(cmpt, descq->cmpt_entry_len,
				32, 4, buf + len, buflen - len, 0);
		len = strlen(buf);
		if (descq->cmpt_entry_len > DESC_SZ_32B_BYTES) {
			len += sprintf(buf + len, " ");
			hex_dump_to_buffer(cmpt + DESC_SZ_32B_BYTES,
					   descq->cmpt_entry_len,
					   32, 4, buf + len, buflen - len, 0);
			len = strlen(buf);
		}
		buf[len++] = '\n';
	}

	len += sprintf(buf + len,
			"CMPL STATUS: 0x%p ",
			descq->desc_cmpt_cmpl_status);

	p = descq->desc_cmpt_cmpl_status;
	dma_rmb();
	hex_dump_to_buffer(p, sizeof(struct qdma_c2h_cmpt_cmpl_status),
			16, 4, buf + len, buflen - len, 0);
	len = strlen(buf);
	buf[len++] = '\n';

	return len;
}

int qdma_descq_dump_state(struct qdma_descq *descq, char *buf, int buflen)
{
	char *cur = buf;
	char *const end = buf + buflen;

	if (!buf || !buflen) {
		pr_warn("incorrect arguments buf=%p buflen=%d", buf, buflen);
		return 0;
	}

	cur += snprintf(cur, end - cur, "%s %s ",
			descq->conf.name, descq->conf.c2h ? "C2H" : "H2C");
	if (cur >= end)
		goto handle_truncation;

	if (descq->err)
		cur += snprintf(cur, end - cur, "ERR\n");
	else if (descq->q_state == Q_STATE_ONLINE)
		cur += snprintf(cur, end - cur, "online\n");
	else if (descq->q_state == Q_STATE_ENABLED)
		cur += snprintf(cur, end - cur, "cfg'ed\n");
	else
		cur += snprintf(cur, end - cur, "un-initialized\n");
	if (cur >= end)
		goto handle_truncation;

	return cur - buf;

handle_truncation:
	*buf = '\0';
	return cur - buf;
}

int qdma_descq_dump(struct qdma_descq *descq, char *buf, int buflen, int detail)
{
	char *cur = buf;
	char *const end = buf + buflen;

	if (!buf || !buflen) {
		pr_info("%s:%s 0x%x/0x%x, desc sz %u/%u, pidx %u, cidx %u\n",
			descq->conf.name, descq->err ? "ERR" : "",
			descq->conf.qidx, descq->qidx_hw, descq->conf.rngsz,
			descq->avail, descq->pidx, descq->cidx);
		return 0;
	}

	cur += qdma_descq_dump_state(descq, cur, end - cur);
	if (cur >= end)
		goto handle_truncation;

	if (descq->q_state == Q_STATE_DISABLED)
		return cur - buf;

	cur += snprintf(cur, end - cur,
		"\thw_ID %d, thp %s, desc 0x%p/0x%llx, %u\n",
		descq->qidx_hw,
		descq->cmplthp ? descq->cmplthp->name : "?",
		descq->desc, descq->desc_bus, descq->conf.rngsz);
	if (cur >= end)
		goto handle_truncation;

	if (descq->conf.st && descq->conf.c2h) {
		cur += snprintf(cur, end - cur,
			"\tcmpt desc 0x%p/0x%llx, %u\n",
			descq->desc_cmpt, descq->desc_cmpt_bus,
			descq->conf.rngsz_cmpt);
		if (cur >= end)
			goto handle_truncation;
	}

	if (!detail)
		return cur - buf;

	if (descq->desc_cmpl_status) {
		u8 *cs = descq->desc_cmpl_status;

		cur += snprintf(cur, end - cur, "\n\tcmpl status: 0x%p, ", cs);
		if (cur >= end)
			goto handle_truncation;

		dma_rmb();
#if KERNEL_VERSION(4, 0, 0) <= LINUX_VERSION_CODE
		cur += hex_dump_to_buffer(cs,
					  sizeof(struct qdma_desc_cmpl_status),
					  16, 4, cur, end - cur, 0);
#else
		hex_dump_to_buffer(cs, sizeof(struct qdma_desc_cmpl_status),
					  16, 4, cur, end - cur, 0);
		cur += strlen(cur);
#endif
		if (cur >= end)
			goto handle_truncation;

		cur += snprintf(cur, end - cur, "\n");
		if (cur >= end)
			goto handle_truncation;
	}
	if (descq->desc_cmpt_cmpl_status) {
		u8 *cs = descq->desc_cmpt_cmpl_status;

		cur += snprintf(cur, end - cur, "\tCMPT CMPL STATUS: 0x%p, ",
				cs);
		if (cur >= end)
			goto handle_truncation;

		dma_rmb();
#if KERNEL_VERSION(4, 0, 0) <= LINUX_VERSION_CODE
		cur += hex_dump_to_buffer(cs,
				sizeof(struct qdma_c2h_cmpt_cmpl_status),
				16, 4, cur, end - cur, 0);
#else
		hex_dump_to_buffer(cs, sizeof(struct qdma_c2h_cmpt_cmpl_status),
					  16, 4, cur, end - cur, 0);
		cur += strlen(cur);
#endif
		if (cur >= end)
			goto handle_truncation;

		cur += snprintf(cur, end - cur, "\n");
		if (cur >= end)
			goto handle_truncation;
	}

	return cur - buf;

handle_truncation:
	*buf = '\0';
	return cur - buf;
}

int qdma_queue_avail_desc(unsigned long dev_hndl, unsigned long id)
{
	struct qdma_descq *descq = qdma_device_get_descq_by_id(
					(struct xlnx_dma_dev *)dev_hndl,
					id, NULL, 0, 1);
	int avail;

	if (!descq)
		return QDMA_ERR_INVALID_QIDX;

	lock_descq(descq);
	avail = descq->avail;
	unlock_descq(descq);

	return avail;
}

#ifdef ERR_DEBUG
int qdma_queue_set_err_induction(unsigned long dev_hndl, unsigned long id,
			u32 err, char *buf, int buflen)
{
	struct qdma_descq *descq = qdma_device_get_descq_by_id(
					(struct xlnx_dma_dev *)dev_hndl,
					id, buf, buflen, 1);
	const char *dummy; /* to avoid compiler warnings */
	unsigned int err_en = (err >> 31);
	unsigned int err_no = (err & 0x7FFFFFFF);

	dummy = xnl_attr_str[0];
	dummy = xnl_op_str[0];
	if (!descq)
		return QDMA_ERR_INVALID_QIDX;
	if (err_no < sb_mi_h2c0_dat) {
		descq->induce_err &= ~(1 << err_no);
		descq->induce_err |= (err_en << err_no);
	} else {
		descq->ecc_err &= ~(1 << (err_no - sb_mi_h2c0_dat));
		descq->ecc_err |= (err_en << (err_no - sb_mi_h2c0_dat));
	}
	pr_info("Errs enabled = [QDMA]: 0x%08x 0x%08x\n [ECC]: 0x%08x 0x%08x",
		(u32)(descq->induce_err >> 32),
		(u32)(descq->induce_err & 0xFFFFFFFF),
		(u32)(descq->ecc_err >> 32),
		(u32)(descq->ecc_err & 0xFFFFFFFF));

	return 0;
}
#endif

int qdma_queue_packet_write(unsigned long dev_hndl, unsigned long id,
				struct qdma_request *req)
{
	struct qdma_descq *descq = qdma_device_get_descq_by_id(
					(struct xlnx_dma_dev *)dev_hndl,
					id, NULL, 0, 1);
	struct qdma_sgt_req_cb *cb = qdma_req_cb_get(req);
	int rv;

	if (!descq)
		return QDMA_ERR_INVALID_QIDX;

	if (!descq->conf.st || descq->conf.c2h) {
		pr_err("%s: st %d, c2h %d.\n",
			descq->conf.name, descq->conf.st, descq->conf.c2h);
		return -EINVAL;
	}

	memset(cb, 0, QDMA_REQ_OPAQUE_SIZE);
	qdma_waitq_init(&cb->wq);

	if (!req->dma_mapped) {
		rv = sgl_map(descq->xdev->conf.pdev, req->sgl, req->sgcnt,
				DMA_TO_DEVICE);
		if (rv < 0) {
			pr_err("%s map sgl %u failed, %u.\n",
				descq->conf.name, req->sgcnt, req->count);
			goto unmap_sgl;
		}
		cb->unmap_needed = 1;
	}

	lock_descq(descq);
	if (descq->q_state != Q_STATE_ONLINE) {
		unlock_descq(descq);
		pr_err("%s descq %s NOT online.\n",
			descq->xdev->conf.name, descq->conf.name);
		rv = -EINVAL;
		goto unmap_sgl;
	}

	list_add_tail(&cb->list, &descq->work_list);
	descq->pend_req_desc += ((req->count + PAGE_SIZE - 1) >> PAGE_SHIFT);
	unlock_descq(descq);
	qdma_descq_proc_sgt_request(descq);

	pr_debug("%s: cb 0x%p submitted for bytes %u.\n", descq->conf.name, cb, req->count);

	return req->count;

unmap_sgl:
	if (cb->unmap_needed)
		sgl_unmap(descq->xdev->conf.pdev, req->sgl, req->sgcnt,
			DMA_TO_DEVICE);
	return rv;
}

int qdma_descq_get_cmpt_udd(unsigned long dev_hndl, unsigned long id,
		char *buf, int buflen)
{
	uint8_t *cmpt;
	uint8_t i = 0;
	int len = 0;
	int print_len = 0;
	struct qdma_descq *descq = qdma_device_get_descq_by_id(
					(struct xlnx_dma_dev *)dev_hndl,
					id, buf, buflen, 1);
	struct qdma_c2h_cmpt_cmpl_status *cs;

	if (!descq)
		return QDMA_ERR_INVALID_QIDX;

	if (!buf || !buflen)
		return QDMA_ERR_INVALID_INPUT_PARAM;

	cs = (struct qdma_c2h_cmpt_cmpl_status *)
						descq->desc_cmpt_cmpl_status;

	cmpt = descq->desc_cmpt + ((cs->pidx - 1) * descq->cmpt_entry_len);

	/*
	 * Ignoring the first 4 bits of the completion entry as they represent
	 * the error and color bits.
	 * TODO: May need to change the masking logic and move that in thegtest,
	 * as error and color bit positions may change in the future releases.
	 */
	for (i = 0; i < descq->cmpt_entry_len; i++) {
		if (buf && buflen) {
			if (i == 0)
				print_len = sprintf(buf + len, "%02x",
						(cmpt[i] & 0xF0));
			else
				print_len = sprintf(buf + len, "%02x",
						    cmpt[i]);

		}
		buflen -= print_len;
		len += print_len;
	}
	buf[len] = '\0';

	return 0;
}

int qdma_descq_read_cmpt(struct qdma_descq *descq,
				u32 num_entries, u8 *cmpt_data)
{
	uint8_t *cmpt = descq->desc_cmpt;
	int i = 0;
	int stride = descq->cmpt_entry_len;
	struct qdma_c2h_cmpt_cmpl_status *cs;
	u8 *temp = cmpt_data;

	if (!descq->desc_cmpt)
		return 0;

	lock_descq(descq);
	cs = (struct qdma_c2h_cmpt_cmpl_status *)
				descq->desc_cmpt_cmpl_status;
	cmpt = descq->desc_cmpt + (descq->cidx_cmpt * stride);
	for (i = 0; i < num_entries;
			i++, cmpt += stride, cmpt_data += stride) {
		memcpy(cmpt_data, cmpt, stride);
	}
	*(temp) = (*(temp) & 0xF0);
	unlock_descq(descq);

	return i;
}

int qdma_descq_read_cmpt_data(unsigned long dev_hndl, unsigned long id,
				u32 *num_entries,  u8 **cmpt_entries,
				char *buf, int buflen)
{
	struct qdma_descq *descq = qdma_device_get_descq_by_id(
					(struct xlnx_dma_dev *)dev_hndl,
					id, buf, buflen, 1);
	struct qdma_c2h_cmpt_cmpl_status *cs = NULL;
	unsigned int pidx_cmpt = 0;
	unsigned int cidx_cmpt = 0;
	int  rv = 0;
	u32 pend = 0;
	struct qdma_descq *pairq = NULL;
	char *cur = buf;
	char * const end = buf + buflen;
	struct qdma_dev *qdev = NULL;

	if (!descq)
		return QDMA_ERR_INVALID_INPUT_PARAM;

	if (!buf || !buflen)
		return QDMA_ERR_INVALID_INPUT_PARAM;

	qdev = xdev_2_qdev(descq->xdev);

	pr_debug("descq->conf.rngsz = %d, descq->conf.rngsz_cmpt = %d",
		descq->conf.rngsz, descq->conf.rngsz_cmpt);
	if (!descq->conf.st) {
		if (descq->conf.c2h)
			pairq = qdev->h2c_descq + descq->conf.qidx;
		else
			pairq = qdev->c2h_descq + descq->conf.qidx;

		if (!((descq->conf.en_mm_cmpt ||
			pairq->conf.en_mm_cmpt) &&
			(descq->mm_cmpt_ring_crtd ||
			pairq->mm_cmpt_ring_crtd))) {
			pr_err("MM Completion not enabled for this queue");
			rv = QDMA_ERR_INVALID_INPUT_PARAM;
			cur += snprintf(cur, end - cur,
					"%s, MM Completion not enabled for this queue\n",
					descq->xdev->conf.name);
			if (cur >= end) {
				*buf = '\0';
				*num_entries = 0;
			}
			return rv;
		}

		if (!descq->mm_cmpt_ring_crtd && pairq->mm_cmpt_ring_crtd)
			descq = pairq;
	} else {
		pr_err("Not supported for ST mode");
		rv = QDMA_ERR_INVALID_INPUT_PARAM;
		return rv;
	}

	lock_descq(descq);
	cs = (struct qdma_c2h_cmpt_cmpl_status *)
				descq->desc_cmpt_cmpl_status;
	cidx_cmpt = descq->cidx_cmpt;
	pidx_cmpt = cs->pidx;
	unlock_descq(descq);
	pend = ring_idx_delta(pidx_cmpt,
			      cidx_cmpt,
				descq->conf.rngsz_cmpt);
	pr_debug("pend = %d, cs->pidx = %d cs->cidx = %d, descq->cidx_cmpt = %d, descq->pidx_cmpt = %d",
				    pend, cs->pidx, cs->cidx,
				    cidx_cmpt,
				    pidx_cmpt);
	*num_entries = min(pend, descq->conf.rngsz_cmpt);


	if (!pend) {
		int l = 0;

		pr_debug("No pending cmpt entries");
		/* SW work around where next interrupt could be missed when
		 * there are no entries as of now
		 */
		if (descq->xdev->conf.qdma_drv_mode != POLL_MODE) {
			lock_descq(descq);
			descq->cmpt_cidx_info.wrb_cidx = cidx_cmpt;
			unlock_descq(descq);
			rv = queue_cmpt_cidx_update(descq->xdev,
					descq->conf.qidx,
					&descq->cmpt_cidx_info);
			if (rv < 0) {
				pr_err("%s: Failed to update cmpt cidx\n",
						descq->conf.name);
				return -EINVAL;
			}

		}

		l = strlen(buf);

		l += snprintf(buf + l, buflen,
			"%s no pending entries in cmpt ring\n",
			descq->conf.name);
		buf[l] = '\0';
		*num_entries = 0;
		return 0;
	}

	if (*num_entries > 0) {
		/** Note: cmpt_entries will be freed by caller in nl.c
		 *  do not free it here
		 */
		*cmpt_entries = kzalloc((*num_entries * descq->cmpt_entry_len),
								GFP_KERNEL);
		if (!*cmpt_entries) {
			int l = snprintf(buf, buflen, "OOM for cmpt_entries\n");

			buf[l] = '\0';
			*num_entries = 0;
			return 0;
		}

		rv = qdma_descq_read_cmpt(descq, *num_entries, *cmpt_entries);
		lock_descq(descq);
		descq->cidx_cmpt += *num_entries;
		descq->cmpt_cidx_info.wrb_cidx = descq->cidx_cmpt;
		unlock_descq(descq);
		rv = queue_cmpt_cidx_update(descq->xdev,
				descq->conf.qidx,
				&descq->cmpt_cidx_info);
		if (rv < 0) {
			pr_err("%s: Failed to update cmpt cidx\n",
				  descq->conf.name);
			return -EINVAL;
		}
	}

	return rv;
}
