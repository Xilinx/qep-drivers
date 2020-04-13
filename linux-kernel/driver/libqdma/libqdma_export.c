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

/**
 * @file
 * @brief This file contains the definitions for libqdma interfaces
 *
 */
#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__

#include "libqdma_export.h"

#include "qdma_descq.h"
#include "qdma_device.h"
#include "qdma_thread.h"
#include "qdma_regs.h"
#include "qdma_context.h"
#include "qdma_intr.h"
#include "qdma_st_c2h.h"
#include "thread.h"
#include "version.h"
#include "qdma_resource_mgmt.h"
#include "qdma_mbox.h"
#include "qdma_platform.h"

#ifdef DEBUGFS
#include "qdma_debugfs_queue.h"

/** debugfs root */
struct dentry *qdma_debugfs_root;
static bool qdma_debufs_cleanup = true;
#endif


static struct qctxt_entry sw_ctxt_entries[] = {
	{"PIDX", 0},
	{"IRQ Arm", 0},
	{"Function Id", 0},
	{"Queue Enable", 0},
	{"Fetch Credit Enable", 0},
	{"Write back/Intr Check", 0},
	{"Write back/Intr Interval", 0},
	{"Address Translation", 0},
	{"Fetch Max", 0},
	{"Ring Size", 0},
	{"Descriptor Size", 0},
	{"Bypass Enable", 0},
	{"MM Channel", 0},
	{"Writeback Enable", 0},
	{"Interrupt Enable", 0},
	{"Port Id", 0},
	{"Interrupt No Last", 0},
	{"Error", 0},
	{"Writeback Error Sent", 0},
	{"IRQ Request", 0},
	{"Marker Disable", 0},
	{"Is Memory Mapped", 0},
	{"Descriptor Ring Base Addr (Low)", 0},
	{"Descriptor Ring Base Addr (High)", 0},
	{"Interrupt Vector", 0},
	{"Interrupt Aggregation", 0},
};

static struct qctxt_entry hw_ctxt_entries[] = {
	{"CIDX", 0},
	{"Credits Consumed", 0},
	{"Descriptors Pending", 0},
	{"Queue Invalid No Desc Pending", 0},
	{"Eviction Pending", 0},
	{"Fetch Pending", 0},
};

static struct qctxt_entry credit_ctxt_entries[] = {
	{"Credit", 0},
};

static struct qctxt_entry cmpt_ctxt_entries[] = {
	{"Enable Status Desc Update", 0},
	{"Enable Interrupt", 0},
	{"Trigger Mode", 0},
	{"Function Id", 0},
	{"Counter Index", 0},
	{"Timer Index", 0},
	{"Interrupt State", 0},
	{"Color", 0},
	{"Ring Size", 0},
	{"Base Address (Low)", 0},
	{"Base Address (High)", 0},
	{"Descriptor Size", 0},
	{"PIDX", 0},
	{"CIDX", 0},
	{"Valid", 0},
	{"Error", 0},
	{"Trigger Pending", 0},
	{"Timer Running", 0},
	{"Full Update", 0},
	{"Over Flow Check Disable", 0},
	{"Address Translation", 0},
	{"Interrupt Vector", 0},
	{"Interrupt Aggregation", 0},
};

static struct qctxt_entry c2h_pftch_ctxt_entries[] = {
	{"Bypass", 0},
	{"Buffer Size Index", 0},
	{"Port Id", 0},
	{"Error", 0},
	{"Prefetch Enable", 0},
	{"In Prefetch", 0},
	{"Software Credit", 0},
	{"Valid", 0},
};

static struct qctxt_entry ind_intr_ctxt_entries[] = {
	{"valid", 0},
	{"vec", 0},
	{"int_st", 0},
	{"color", 0},
	{"baddr_4k (Low)", 0},
	{"baddr_4k (High)", 0},
	{"page_size", 0},
	{"pidx", 0},
	{"at", 0},
};

static const int num_entries_sw_ctxt = ARRAY_SIZE(sw_ctxt_entries);
static const int num_entries_hw_ctxt = ARRAY_SIZE(hw_ctxt_entries);
static const int num_entries_credit_ctxt = ARRAY_SIZE(credit_ctxt_entries);
static const int num_entries_cmpt_ctxt = ARRAY_SIZE(cmpt_ctxt_entries);
static const int num_entries_c2h_pftch_ctxt =
		ARRAY_SIZE(c2h_pftch_ctxt_entries);
static const int num_entries_ind_intr_ctxt = ARRAY_SIZE(ind_intr_ctxt_entries);

#define QDMA_Q_PEND_LIST_COMPLETION_TIMEOUT 1000 /* msec */

struct drv_mode_name mode_name_list[] = {
	{ AUTO_MODE,			"auto"},
	{ POLL_MODE,			"poll"},
	{ DIRECT_INTR_MODE,		"direct interrupt"},
	{ INDIRECT_INTR_MODE,	"indirect interrupt"},
	{ LEGACY_INTR_MODE,		"legacy interrupt"}
};


struct qdma_q_type q_type_list[] = {
	{"H2C", Q_H2C},
	{"C2H", Q_C2H},
	{"CMPT", Q_CMPT},
};

/* ********************* static function definitions ************************ */
#ifdef __QDMA_VF__
static int qdma_dev_notify_qadd(struct qdma_descq *descq,
		enum queue_type_t q_type)
{
	struct mbox_msg *m;
	int rv = 0;
	struct xlnx_dma_dev *xdev = descq->xdev;

	m = qdma_mbox_msg_alloc();
	if (!m) {
		pr_err("Failed to allocate mbox msg");
		return -ENOMEM;
	}


	qdma_mbox_compose_vf_notify_qadd(xdev->func_id,
		descq->qidx_hw, q_type, m->raw);

	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		pr_err("%s, mbox failed for queue add %d.\n",
				xdev->conf.name, rv);
		goto free_msg;
	}
	rv = qdma_mbox_vf_response_status(m->raw);
	if (rv < 0) {
		pr_err("mbox_vf_response_status failed, err = %d", rv);
		rv = -EINVAL;
	}

free_msg:
	qdma_mbox_msg_free(m);
	return rv;
}

static int qdma_dev_notify_qdel(struct qdma_descq *descq,
		enum queue_type_t q_type)
{
	struct mbox_msg *m;
	int rv = 0;
	struct xlnx_dma_dev *xdev = descq->xdev;

	m = qdma_mbox_msg_alloc();
	if (!m) {
		pr_err("Failed to allocate mbox msg");
		return -ENOMEM;
	}

	qdma_mbox_compose_vf_notify_qdel(xdev->func_id, descq->qidx_hw,
			q_type, m->raw);

	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		pr_err("%s, mbox failed for queue add %d.\n",
				xdev->conf.name, rv);
		goto free_msg;
	}
	rv = qdma_mbox_vf_response_status(m->raw);
	if (rv < 0) {
		pr_err("mbox_vf_response_status failed, err = %d", rv);
		rv = -EINVAL;
	}

free_msg:
	qdma_mbox_msg_free(m);
	return rv;
}

static int qdma_dev_get_active_qcnt(struct xlnx_dma_dev *xdev,
		int *h2c_qs, int *c2h_qs, int *cmpt_qs)
{
	struct mbox_msg *m;
	int rv = 0;

	m = qdma_mbox_msg_alloc();
	if (!m) {
		pr_err("Failed to allocate mbox msg");
		return -ENOMEM;
	}

	qdma_mbox_compose_vf_get_device_active_qcnt(xdev->func_id, m->raw);

	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		pr_err("%s, mbox failed for queue add %d.\n",
				xdev->conf.name, rv);
		goto free_msg;
	}
	rv = qdma_mbox_vf_response_status(m->raw);
	if (rv < 0) {
		pr_err("mbox_vf_response_status failed, err = %d", rv);
		rv = -EINVAL;
		goto free_msg;
	}

	*h2c_qs = qdma_mbox_vf_active_queues_get(m->raw, Q_H2C);
	*c2h_qs = qdma_mbox_vf_active_queues_get(m->raw, Q_C2H);
	*cmpt_qs = qdma_mbox_vf_active_queues_get(m->raw, Q_CMPT);

	return rv;

free_msg:
	qdma_mbox_msg_free(m);
	return rv;
}

#endif

static int qdma_validate_qconfig(struct xlnx_dma_dev *xdev,
				struct qdma_queue_conf *qconf,
				char *buf, int buflen)
{

	/** If xdev is NULL return error as Invalid parameter */
	if (!xdev  || !qconf) {
		pr_err("Invalid input received, xdev=%p, qconf =%p",
				xdev, qconf);
		return -EINVAL;
	}

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	if (qconf->cmpl_trig_mode > TRIG_MODE_COMBO) {
		pr_err("Invalid trigger mode :%d",
				qconf->cmpl_trig_mode);
		snprintf(buf, buflen,
			"qdma%05x : Invalid trigger mode %d passed\n",
			xdev->conf.bdf, qconf->cmpl_trig_mode);
		return -EINVAL;
	}

	if ((xdev->version_info.device_type == QDMA_DEVICE_VERSAL) &&
			(xdev->version_info.versal_ip_type ==
			QDMA_VERSAL_HARD_IP)) {
		/* 64B desc size is not supported in 2018.2 release */
		if ((qconf->cmpl_desc_sz == CMPT_DESC_SZ_64B) ||
				(qconf->sw_desc_sz == DESC_SZ_64B)) {
			pr_err("Invalid descriptor sw desc :%d, cmpl desc :%d",
					qconf->sw_desc_sz, qconf->sw_desc_sz);
			snprintf(buf, buflen,
				"qdma%05x : 64B desc size is not supported\n",
				xdev->conf.bdf);
			return -EINVAL;
		}

		if (qconf->cmpl_trig_mode == TRIG_MODE_COMBO) {
			pr_err("Invalid trigger mode :%d",
					qconf->cmpl_trig_mode);
			snprintf(buf, buflen,
				"qdma%05x : Trigger mode COMBO is not supported\n",
				xdev->conf.bdf);
			return -EINVAL;
		}
	}

	return 0;
}

void qdma_fill_sw_ctxt(struct qdma_descq_sw_ctxt *sw_ctxt, u8 ind_mode)
{
	sw_ctxt_entries[0].value = sw_ctxt->pidx;
	sw_ctxt_entries[1].value = sw_ctxt->irq_arm;
	sw_ctxt_entries[2].value = sw_ctxt->fnc_id;
	sw_ctxt_entries[3].value = sw_ctxt->qen;
	sw_ctxt_entries[4].value = sw_ctxt->frcd_en;
	sw_ctxt_entries[5].value = sw_ctxt->wbi_chk;
	sw_ctxt_entries[6].value = sw_ctxt->wbi_intvl_en;
	sw_ctxt_entries[7].value = sw_ctxt->at;
	sw_ctxt_entries[8].value = sw_ctxt->fetch_max;
	sw_ctxt_entries[9].value = sw_ctxt->rngsz_idx;
	sw_ctxt_entries[10].value = sw_ctxt->desc_sz;
	sw_ctxt_entries[11].value = sw_ctxt->bypass;
	sw_ctxt_entries[12].value = sw_ctxt->mm_chn;
	sw_ctxt_entries[13].value = sw_ctxt->wbk_en;
	sw_ctxt_entries[14].value = sw_ctxt->irq_en;
	sw_ctxt_entries[15].value = sw_ctxt->port_id;
	sw_ctxt_entries[16].value = sw_ctxt->irq_no_last;
	sw_ctxt_entries[17].value = sw_ctxt->err;
	sw_ctxt_entries[18].value = sw_ctxt->err_wb_sent;
	sw_ctxt_entries[19].value = sw_ctxt->irq_req;
	sw_ctxt_entries[20].value = sw_ctxt->mrkr_dis;
	sw_ctxt_entries[21].value = sw_ctxt->is_mm;
	sw_ctxt_entries[22].value = sw_ctxt->ring_bs_addr & 0xFFFFFFFF;
	sw_ctxt_entries[23].value =
			(sw_ctxt->ring_bs_addr >> 32) & 0xFFFFFFFF;
	sw_ctxt_entries[24].value = sw_ctxt->vec;
	if (ind_mode == 1)
		strcpy(sw_ctxt_entries[24].name, "Ring Index");
	sw_ctxt_entries[25].value = sw_ctxt->intr_aggr;
}

void qdma_fill_cmpt_ctxt(struct qdma_descq_cmpt_ctxt *cmpt_ctxt, u8 ind_mode)
{
	cmpt_ctxt_entries[0].value = cmpt_ctxt->en_stat_desc;
	cmpt_ctxt_entries[1].value = cmpt_ctxt->en_int;
	cmpt_ctxt_entries[2].value = cmpt_ctxt->trig_mode;
	cmpt_ctxt_entries[3].value = cmpt_ctxt->fnc_id;
	cmpt_ctxt_entries[4].value = cmpt_ctxt->counter_idx;
	cmpt_ctxt_entries[5].value = cmpt_ctxt->timer_idx;
	cmpt_ctxt_entries[6].value = cmpt_ctxt->in_st;
	cmpt_ctxt_entries[7].value = cmpt_ctxt->color;
	cmpt_ctxt_entries[8].value = cmpt_ctxt->ringsz_idx;
	cmpt_ctxt_entries[9].value = cmpt_ctxt->bs_addr & 0xFFFFFFFF;
	cmpt_ctxt_entries[10].value =
			(cmpt_ctxt->bs_addr >> 32) & 0xFFFFFFFF;
	cmpt_ctxt_entries[11].value = cmpt_ctxt->desc_sz;
	cmpt_ctxt_entries[12].value = cmpt_ctxt->pidx;
	cmpt_ctxt_entries[13].value = cmpt_ctxt->cidx;
	cmpt_ctxt_entries[14].value = cmpt_ctxt->valid;
	cmpt_ctxt_entries[15].value = cmpt_ctxt->err;
	cmpt_ctxt_entries[16].value = cmpt_ctxt->user_trig_pend;
	cmpt_ctxt_entries[17].value = cmpt_ctxt->timer_running;
	cmpt_ctxt_entries[18].value = cmpt_ctxt->full_upd;
	cmpt_ctxt_entries[19].value = cmpt_ctxt->ovf_chk_dis;
	cmpt_ctxt_entries[20].value = cmpt_ctxt->at;
	cmpt_ctxt_entries[21].value = cmpt_ctxt->vec;
	if (ind_mode == 1)
		strcpy(cmpt_ctxt_entries[21].name, "Ring Index");
	cmpt_ctxt_entries[22].value = cmpt_ctxt->int_aggr;
}

void qdma_fill_hw_ctxt(struct qdma_descq_hw_ctxt *hw_ctxt)
{
	hw_ctxt_entries[0].value = hw_ctxt->cidx;
	hw_ctxt_entries[1].value = hw_ctxt->crd_use;
	hw_ctxt_entries[2].value = hw_ctxt->dsc_pend;
	hw_ctxt_entries[3].value = hw_ctxt->idl_stp_b;
	hw_ctxt_entries[4].value = hw_ctxt->evt_pnd;
	hw_ctxt_entries[5].value = hw_ctxt->fetch_pnd;
}

void qdma_fill_credit_ctxt(struct qdma_descq_credit_ctxt *cr_ctxt)
{
	credit_ctxt_entries[0].value = cr_ctxt->credit;
}

void qdma_fill_pfetch_ctxt(struct qdma_descq_prefetch_ctxt *pfetch_ctxt)
{
	c2h_pftch_ctxt_entries[0].value = pfetch_ctxt->bypass;
	c2h_pftch_ctxt_entries[1].value = pfetch_ctxt->bufsz_idx;
	c2h_pftch_ctxt_entries[2].value = pfetch_ctxt->port_id;
	c2h_pftch_ctxt_entries[3].value = pfetch_ctxt->err;
	c2h_pftch_ctxt_entries[4].value = pfetch_ctxt->pfch_en;
	c2h_pftch_ctxt_entries[5].value = pfetch_ctxt->pfch;
	c2h_pftch_ctxt_entries[6].value = pfetch_ctxt->sw_crdt;
	c2h_pftch_ctxt_entries[7].value = pfetch_ctxt->valid;
}

void qdma_fill_intr_ctxt(struct qdma_indirect_intr_ctxt *intr_ctxt)
{
	ind_intr_ctxt_entries[0].value = intr_ctxt->valid;
	ind_intr_ctxt_entries[1].value = intr_ctxt->vec;
	ind_intr_ctxt_entries[2].value = intr_ctxt->int_st;
	ind_intr_ctxt_entries[3].value = intr_ctxt->color;
	ind_intr_ctxt_entries[4].value =
			intr_ctxt->baddr_4k & 0xFFFFFFFF;
	ind_intr_ctxt_entries[5].value =
			(intr_ctxt->baddr_4k >> 32) & 0xFFFFFFFF;
	ind_intr_ctxt_entries[6].value = intr_ctxt->page_size;
	ind_intr_ctxt_entries[7].value = intr_ctxt->pidx;
	ind_intr_ctxt_entries[8].value = intr_ctxt->at;
}

int qdma_parse_ctxt_to_buf(enum qdma_q_cntxt cntxt,
		struct qdma_hw_version_info *version_info,
		char *buf, int buflen)
{
	int i = 0;
	int len = 0;
	int num_entries;
	struct qctxt_entry *entries;

	/** Make sure that buf and buflen is not invalid */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	switch (cntxt) {
	case QDMA_SW_CNTXT:
		entries = sw_ctxt_entries;
		num_entries = num_entries_sw_ctxt;
		break;
	case QDMA_HW_CNTXT:
		entries = hw_ctxt_entries;
		num_entries = num_entries_hw_ctxt;
		break;
	case QDMA_CMPT_CNTXT:
		entries = cmpt_ctxt_entries;
		num_entries = num_entries_cmpt_ctxt;
		break;
	case QDMA_PFETCH_CNTXT:
		entries = c2h_pftch_ctxt_entries;
		num_entries = num_entries_c2h_pftch_ctxt;
		break;
	case QDMA_CR_CNTXT:
		entries = credit_ctxt_entries;
		num_entries = num_entries_credit_ctxt;
		break;
	case QDMA_INTR_CNTXT:
		entries = ind_intr_ctxt_entries;
		num_entries = num_entries_ind_intr_ctxt;
		break;
	default:
		entries = NULL;
		break;
	}

	if (!entries) {
		pr_err("Invalid qctxt entries");
		return -EINVAL;
	}

	for (i = num_entries - 1; i >= 0; i--) {

		if ((strcmp(entries[i].name, "Address Translation") == 0) ||
		(strcmp(entries[i].name, "Fetch Max") == 0) ||
		(strcmp(entries[i].name, "Over Flow Check Disable") == 0) ||
		(strcmp(entries[i].name, "at") == 0) ||
		(strcmp(entries[i].name, "Eviction Pending") == 0)) {
			if ((version_info->device_type ==
					QDMA_DEVICE_VERSAL) &&
				(version_info->versal_ip_type ==
					QDMA_VERSAL_HARD_IP))
				continue;
		}

		len += snprintf(buf + len, buflen - len,
				"\t\t%-47s %#-10x %u\n",
				entries[i].name,
				entries[i].value,
				entries[i].value);
		if (len >= buflen)
			return buflen - len;
	}

	len += snprintf(buf + len, buflen - len, "\n");
	if (len >= buflen)
		return buflen - len;

	return len;
}

/*****************************************************************************/
/**
 * qdma_request_wait_for_cmpl() - static function to monitor the
 *								wait completion
 *
 * @param[in]	xdev:	pointer to xlnx_dma_dev structure
 * @param[in]	descq:	pointer to qdma_descq structure
 * @param[in]	req:	qdma request
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static int qdma_request_wait_for_cmpl(struct xlnx_dma_dev *xdev,
			struct qdma_descq *descq, struct qdma_request *req)
{
	struct qdma_sgt_req_cb *cb = qdma_req_cb_get(req);

	/** if timeout is mentioned in the request,
	 *  wait until the timeout occurs or wait until the
	 *  call back is completed
	 */

	if (req->timeout_ms)
		qdma_waitq_wait_event_timeout(cb->wq, cb->done,
			msecs_to_jiffies(req->timeout_ms));
	else
		qdma_waitq_wait_event(cb->wq, cb->done);

	lock_descq(descq);
	/** if the call back is not done, request timed out
	 *  delete the request list
	 */
	if (!cb->done)
		list_del(&cb->list);

	/** if the call back is not done but the status is updated
	 *  return i/o error
	 */
	if (!cb->done || cb->status) {
		pr_err("%s: req 0x%p, %c,%u,%u/%u,0x%llx, done %d, err %d, tm %u.\n",
				descq->conf.name,
				req, req->write ? 'W' : 'R',
				cb->offset,
				cb->left,
				req->count,
				req->ep_addr,
				cb->done,
				cb->status,
			req->timeout_ms);
		qdma_descq_dump(descq, NULL, 0, 1);
		unlock_descq(descq);

		return -EIO;
	}

	unlock_descq(descq);
	return 0;
}

/*****************************************************************************/
/**
 * qdma_request_submit_st_c2h() - static function to handle the
 *							st c2h request
 *
 * @param[in]	xdev:	pointer to xlnx_dma_dev structure
 * @param[in]	descq:	pointer to qdma_descq structure
 * @param[in]	req:	qdma request
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
static ssize_t qdma_request_submit_st_c2h(struct xlnx_dma_dev *xdev,
			struct qdma_descq *descq, struct qdma_request *req)
{
	struct qdma_dev *qdev;
	struct qdma_sgt_req_cb *cb = qdma_req_cb_get(req);
	int wait = req->fp_done ? 0 : 1;
	int rv = 0;

	/** If xdev is NULL return error as Invalid parameter */
	if (!xdev) {
		pr_err("xdev is invalid");
		return -EINVAL;
	}

	qdev = xdev_2_qdev(xdev);
	/** make sure that qdev is not NULL, else return error */
	if  (!qdev) {
		pr_err("dev %s, qdev null.\n",
			dev_name(&xdev->conf.pdev->dev));
		return -EINVAL;
	}

	pr_debug("%s: data len %u, sgl 0x%p, sgl cnt %u, tm %u ms.\n",
			descq->conf.name,
			req->count, req->sgl, req->sgcnt, req->timeout_ms);

	/** get the request count */
	cb->left = req->count;

	lock_descq(descq);
	if (descq->q_stop_wait) {
		unlock_descq(descq);
		return 0;
	}
	if ((descq->q_state == Q_STATE_ONLINE) &&
			!descq->q_stop_wait) {
		/* add to pend list even before cidx/pidx update as it could
		 *  cause an interrupt and may miss processing of writeback
		 */
		list_add_tail(&cb->list, &descq->pend_list);
		/* any rcv'ed packet not yet read ? */
		/** read the data from the device */
		descq_st_c2h_read(descq, req, 1, 1);
		if (!cb->left) {
			list_del(&cb->list);
			unlock_descq(descq);
			return req->count;
		}
		descq->pend_list_empty = 0;
		unlock_descq(descq);
	} else {
		unlock_descq(descq);
		pr_err("%s descq %s NOT online.\n",
			xdev->conf.name, descq->conf.name);
		return -EINVAL;
	}

	/** if there is a completion thread associated,
	 *  wake up the completion thread to process the
	 *  completion status
	 */
	if (descq->cmplthp)
		qdma_kthread_wakeup(descq->cmplthp);

	if (!wait) {
		pr_debug("%s: cb 0x%p, data len 0x%x NO wait.\n",
			descq->conf.name, cb, req->count);
		return 0;
	}

	/** wait for the request completion */
	rv = qdma_request_wait_for_cmpl(xdev, descq, req);
	if (rv < 0) {
		if (!req->dma_mapped)
			sgl_unmap(xdev->conf.pdev, req->sgl, req->sgcnt,
				DMA_FROM_DEVICE);
		return rv;
	}

	/** Once the request completion received,
	 *  return with the number of processed requests
	 */
	return req->count - cb->left;
}

/* ********************* public function definitions ************************ */

/*****************************************************************************/
/**
 * qdma_queue_get_config() - retrieve the configuration of a queue
 *
 * @param[in]	dev_hndl:	dev_hndl retured from qdma_device_open()
 * @param[in]	id:		queue index
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 * @param[out]	qconf:		pointer to hold the qdma_queue_conf structure.
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_get_config(unsigned long dev_hndl, unsigned long id,
		struct qdma_queue_conf *qconf,
		char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 0);
	/** make sure that descq is not NULL, else return error */
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		snprintf(buf, buflen,
			"Invalid qid(%ld)\n", id);
		return -EINVAL;
	}

	memcpy(qconf, &descq->conf, sizeof(struct qdma_queue_conf));

	snprintf(buf, buflen,
		"Queue configuration for %s id %u is stored in qconf param",
		descq->conf.name,
		descq->conf.qidx);

	return 0;
}

/*****************************************************************************/
/**
 * qdma_device_capabilities_info() - retrieve the capabilities of a device.
 *
 * @dev_hndl:	handle returned from qdma_device_open()
 * @dev_attr: pointer to hold all the device attributes
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_capabilities_info(unsigned long dev_hndl,
		struct qdma_dev_attributes *dev_attr)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_dev_attributes *dev_cap;

	if (!xdev) {
		pr_err("dev_hndl is NULL");
		return -EINVAL;
	}

	if (!dev_attr) {
		pr_err("dev_attr is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		return -EINVAL;
	}

	qdma_get_device_attr(xdev, &dev_cap);

	memcpy(dev_attr, dev_cap, sizeof(struct qdma_dev_attributes));

	return 0;
}


/*****************************************************************************/
/**
 * qdma_config_reg_dump() - display a config registers in a string buffer
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	success: if optional message buffer used then strlen of buf,
 *	otherwise 0
 * @return	<0: error
 *****************************************************************************/

int qdma_config_reg_dump(unsigned long dev_hndl, char *buf,
		int buflen)
{

	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

#ifndef __QDMA_VF__
	rv = xdev->hw.qdma_dump_config_regs((void *)dev_hndl, 0,
			buf, buflen);
#else
	rv = xdev->hw.qdma_dump_config_regs((void *)dev_hndl, 1,
			buf, buflen);
#endif


	return rv;

}



/*****************************************************************************/
/**
 * qdma_queue_dump() - display a queue's state in a string buffer
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	id:		queue index
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	success: if optional message buffer used then strlen of buf,
 *	otherwise 0
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_dump(unsigned long dev_hndl, unsigned long id, char *buf,
				int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	struct qdma_descq_context ctxt;
	int len = 0, slen = 0;
	int rv;
	int ring_index = 0;
	struct qdma_indirect_intr_ctxt intr_ctxt;
	int i = 0;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 0);
	/** make sure that descq is not NULL, else return error */
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		snprintf(buf, buflen,
			"Invalid qid(%ld)\n", id);
		return -EINVAL;
	}

	/** read the descq data to dump */
	if (descq->conf.q_type != Q_CMPT) {
		qdma_descq_dump(descq, buf, buflen, 1);
		len = strlen(buf);
	}

	/** read the descq context for the given qid */
	rv = qdma_descq_context_read(descq->xdev, descq->qidx_hw,
				descq->conf.st, descq->conf.q_type, &ctxt);
	if (rv < 0) {
		snprintf(buf + len, buflen - len,
				"%s read context failed %d.\n",
				descq->conf.name, rv);
		return rv;
	}

	if (descq->conf.q_type != Q_CMPT) {
		/** format the output for all contexts */
		slen = snprintf(buf + len, buflen - len, "\tSOFTWARE CTXT:\n");
		if (slen >= (buflen - len)) {
			pr_err("sw ctxt: insufficient buffer provided");
			return -ENOMEM;
		}
		len += slen;

		if (descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE ||
				descq->xdev->conf.qdma_drv_mode == AUTO_MODE)
			qdma_fill_sw_ctxt(&ctxt.sw_ctxt, 1);
		else
			qdma_fill_sw_ctxt(&ctxt.sw_ctxt, 0);

		slen = qdma_parse_ctxt_to_buf(QDMA_SW_CNTXT,
				&xdev->version_info,
				buf + len, buflen - len);
		if (slen < 0) {
			pr_err("sw ctxt data: insufficient buffer provided");
			return -ENOMEM;
		}

		len += slen;

		slen = snprintf(buf + len, buflen - len, "\tHARDWARE CTXT:\n");
		if (slen >= (buflen - len)) {
			pr_err("hw ctxt: insufficient buffer provided");
			return -ENOMEM;
		}
		len += slen;

		qdma_fill_hw_ctxt(&ctxt.hw_ctxt);
		slen = qdma_parse_ctxt_to_buf(QDMA_HW_CNTXT,
				&xdev->version_info,
				buf + len, buflen - len);
		if (slen < 0) {
			pr_err("hw ctxt data: insufficient buffer provided");
			return -ENOMEM;
		}
		len += slen;

		slen = snprintf(buf + len, buflen - len, "\tCREDIT CTXT:\n");
		if (slen >= (buflen - len)) {
			pr_err("cr ctxt: insufficient buffer provided");
			return -ENOMEM;
		}
		len += slen;

		qdma_fill_credit_ctxt(&ctxt.cr_ctxt);
		slen = qdma_parse_ctxt_to_buf(QDMA_CR_CNTXT,
				&xdev->version_info,
				buf + len, buflen - len);
		if (slen < 0) {
			pr_err("cr ctxt data: insufficient buffer provided");
			return -ENOMEM;
		}
		len += slen;
	}
	/** incase of ST C2H or MM CMPT enabled,
	 *  add completion and prefetch context to the output
	 */
	if ((descq->conf.st && (descq->conf.q_type == Q_C2H)) ||
	    (!descq->conf.st && (descq->conf.q_type == Q_CMPT))) {
		slen = snprintf(buf + len, buflen - len, "\tCMPT CTXT:\n");
		if (slen >= (buflen - len)) {
			pr_err("cmpt ctxt: insufficient buffer provided");
			return -ENOMEM;
		}
		len += slen;
		if (descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE ||
				descq->xdev->conf.qdma_drv_mode == AUTO_MODE)
			qdma_fill_cmpt_ctxt(&ctxt.cmpt_ctxt, 1);
		else
			qdma_fill_cmpt_ctxt(&ctxt.cmpt_ctxt, 0);
		slen = qdma_parse_ctxt_to_buf(QDMA_CMPT_CNTXT,
				&xdev->version_info,
				buf + len, buflen - len);
		if (slen < 0) {
			pr_err("cmpt ctxt data: insufficient buffer provided");
			return -ENOMEM;
		}
		len += slen;
	}

	if (descq->conf.st && (descq->conf.q_type == Q_C2H)) {
		slen = snprintf(buf + len, buflen - len,
						"\tPREFETCH CTXT:\n");
		if (slen >= (buflen - len)) {
			pr_err("pftch ctxt: insufficient buffer provided");
			return -ENOMEM;
		}
		len += slen;
		qdma_fill_pfetch_ctxt(&ctxt.pfetch_ctxt);
		slen = qdma_parse_ctxt_to_buf(QDMA_PFETCH_CNTXT,
				&xdev->version_info,
				buf + len, buflen - len);
		if (slen < 0) {
			pr_err("pftch ctxt data: insufficient buffer provided");
			return -ENOMEM;
		}
		len += slen;
	}

	/** if interrupt aggregation is enabled
	 *  add the interrupt context
	 */
	if ((descq->xdev->conf.qdma_drv_mode == INDIRECT_INTR_MODE) ||
			(descq->xdev->conf.qdma_drv_mode == AUTO_MODE)) {
		for (i = 0; i < QDMA_NUM_DATA_VEC_FOR_INTR_CXT; i++) {
			ring_index = get_intr_ring_index(
					descq->xdev,
					(i + descq->xdev->dvec_start_idx));
			rv = qdma_intr_context_read(descq->xdev,
						ring_index,
						&intr_ctxt);
			if (rv < 0) {
				sprintf(buf + len,
					"%s read intr context failed %d.\n",
					descq->conf.name, rv);
				return rv;
			}
			slen = snprintf(buf + len, buflen - len,
							"\tINTR CTXT:\n");
			if (slen >= (buflen - len)) {
				pr_err("intr ctxt: insufficient buffer provided");
				return -ENOMEM;
			}
			len += slen;

			qdma_fill_intr_ctxt(&intr_ctxt);
			slen = qdma_parse_ctxt_to_buf(QDMA_INTR_CNTXT,
					&xdev->version_info,
					buf + len, buflen - len);
			if (slen < 0) {
				pr_err("intr ctxt data: insufficient buffer provided");
				return -ENOMEM;
			}
			len += slen;
		}
	}

	len += sprintf(buf + len,
			"\ttotal descriptor processed:    %llu\n",
			descq->total_cmpl_descs);

	/** set the buffer end with \0 and return the buffer length */
	return len;
}
/*****************************************************************************/
/**
 * qdma_queue_dump_desc() - display a queue's descriptor ring from index start
 *							~ end in a string buffer
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	id:		queue index
 * @param[in]	start:		start index
 * @param[in]	end:		end index
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	success: if optional message buffer used then strlen of buf,
 *	otherwise 0
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_dump_desc(unsigned long dev_hndl, unsigned long id,
			unsigned int start, unsigned int end, char *buf,
			int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	int len = 0;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/** get the descq details based on the qid provided */
	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		snprintf(buf, buflen,
			"Invalid qid(%ld)\n", id);
		return -EINVAL;
	}

	/** make sure that intr ring entry indexes
	 *  given are with in the range
	 */
	if (start > end) {
		pr_err("start/end param passed is invalid ,<start> shall be less than <end>");
		snprintf(buf, buflen,
			"start/end param passed is invalid, <start> shall be less than <end>");
		return -EINVAL;
	}

	/** dump the queue state */
	len = qdma_descq_dump_state(descq, buf, buflen);
	if (descq->q_state != Q_STATE_ONLINE)
		return len;

	/** dump the queue descriptor state */
	len += qdma_descq_dump_desc(descq, start, end, buf + len, buflen - len);
	return len;
}

/*****************************************************************************/
/**
 * qdma_queue_dump_cmpt() - display a queue's descriptor ring from index start
 *							~ end in a string buffer
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	id:		queue index
 * @param[in]	start:		start index
 * @param[in]	end:		end index
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	success: if optional message buffer used then strlen of buf,
 *	otherwise 0
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_dump_cmpt(unsigned long dev_hndl, unsigned long id,
			unsigned int start, unsigned int end, char *buf,
			int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	int len = 0;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/** get the descq details based on the qid provided */
	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		snprintf(buf, buflen,
			"Invalid qid(%ld)\n", id);
		return -EINVAL;
	}

	/** make sure that intr ring entry indexes
	 *  given are with in the range
	 */
	if (start > end) {
		pr_err("start/end param passed is invalid, <start> shall be less than <end>");
		snprintf(buf, buflen,
			"start/end param passed is invalid, <start> shall be less than <end>");
		return -EINVAL;
	}

	/** dump the descriptor state */
	len = qdma_descq_dump_state(descq, buf, buflen);
	/** if the descriptor is in online state,
	 *  then, dump the completion state
	 */
	if (descq->q_state == Q_STATE_ONLINE)
		len += qdma_descq_dump_cmpt(descq, start, end, buf + len,
				buflen - len);

	return len;
}

/*****************************************************************************/
/**
 * qdma_queue_remove() - remove a queue (i.e., offline, NOT ready for dma)
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	id:		queue index
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_remove(unsigned long dev_hndl, unsigned long id, char *buf,
			int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
#ifdef DEBUGFS
	struct qdma_descq *pair_descq;
#endif
	struct qdma_dev *qdev;
	int rv = 0;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
#ifdef DEBUGFS
	pair_descq = qdma_device_get_pair_descq_by_id(xdev, id, buf, buflen, 1);
#endif

	qdev = xdev_2_qdev(xdev);
	/** make sure that qdev is not NULL, else return error */
	if  (!qdev) {
		pr_err("dev %s, qdev null.\n",
			dev_name(&xdev->conf.pdev->dev));
		return -EINVAL;
	}

	/** make sure that descq is not NULL, else return error */
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		snprintf(buf, buflen,
			"Invalid qid(%ld)\n", id);
		return -EINVAL;
	}

	if (descq->q_state != Q_STATE_ENABLED) {
		snprintf(buf, buflen,
			"queue %s, id %u cannot be deleted. Invalid q state: %s\n",
			descq->conf.name,
			descq->conf.qidx,
			q_state_list[descq->q_state].name);

		pr_err("queue %s, id %u cannot be deleted. Invalid q state: %s",
			descq->conf.name,
			descq->conf.qidx,
			q_state_list[descq->q_state].name);
		return -EINVAL;
	}

#ifdef DEBUGFS
	if (pair_descq)
		/** if pair_descq is not NULL, it means the queue
		 * is in ENABLED state
		 */
		dbgfs_queue_exit(&descq->conf, pair_descq);
	else
		dbgfs_queue_exit(&descq->conf, NULL);
#endif
#ifndef __QDMA_VF__
	rv = qdma_dev_decrement_active_queue(xdev->conf.pdev->bus->number,
			xdev->func_id, descq->conf.q_type);
	if (rv < 0) {
		pr_err("Failed to decrement the active %s queue",
				q_type_list[descq->conf.q_type].name);
		return rv;
	}

	if (descq->conf.st && (descq->conf.q_type == Q_C2H)) {
		rv = qdma_dev_decrement_active_queue(
				xdev->conf.pdev->bus->number,
				xdev->func_id, QDMA_DEV_Q_TYPE_CMPT);
		if (rv < 0) {
			pr_err("Failed to decrement the active CMPT queue");
			qdma_dev_increment_active_queue(
					xdev->conf.pdev->bus->number,
					xdev->func_id, descq->conf.q_type);
			return rv;
		}
	}
#else
	rv = qdma_dev_notify_qdel(descq, descq->conf.q_type);
	if (rv < 0) {
		pr_err("Failed to decrement active %s queue count",
				q_type_list[descq->conf.q_type].name);
		return rv;
	}
	if (descq->conf.st && (descq->conf.q_type == Q_C2H)) {
		rv = qdma_dev_notify_qdel(descq, QDMA_DEV_Q_TYPE_CMPT);
		if (rv < 0) {
			pr_err("Failed to decrement active CMPT queue count");
			qdma_dev_notify_qadd(descq, descq->conf.q_type);
			return rv;
		}
	}
#endif

	lock_descq(descq);
	descq->q_state = Q_STATE_DISABLED;
	unlock_descq(descq);

#ifndef __QDMA_VF__
	if (xdev->conf.qdma_drv_mode == LEGACY_INTR_MODE)
		intr_legacy_clear(descq);
#endif
	snprintf(buf, buflen, "queue %s, id %u deleted.\n",
		descq->conf.name, descq->conf.qidx);

	pr_debug("queue %s, id %u deleted.\n",
				descq->conf.name, descq->conf.qidx);

	return 0;
}

/*****************************************************************************/
/**
 * qdma_queue_config() - configure the queue with qconf parameters
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	qid:		queue id
 * @param[in]	qconf:		queue configuration parameters
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_config(unsigned long dev_hndl, unsigned long qid,
			struct qdma_queue_conf *qconf, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_dev *qdev;
	struct qdma_descq *descq = NULL;
	int rv = 0;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	qdev = xdev_2_qdev(xdev);
	/** make sure that qdev is not NULL, else return error */
	if  (!qdev) {
		pr_err("dev %s, qdev null.\n",
			dev_name(&xdev->conf.pdev->dev));
		snprintf(buf, buflen, "Q not already added. Add Q first\n");
		return -EINVAL;
	}

	/** get the descq for the given qid */
	descq = qdma_device_get_descq_by_id(xdev, qid, NULL, 0, 0);
	if (!descq) {
		pr_err("Invalid queue ID! qid=%lu, max=%u\n", qid, qdev->qmax);
		snprintf(buf, buflen,
				 "Invalid queue ID qid=%lu, max=%u, base=%u\n",
				 qid, qdev->qmax, qdev->qbase);
		return -EINVAL;
	}

	lock_descq(descq);
	/* if descq is not in enabled state, return error */
	if (descq->q_state != Q_STATE_ENABLED) {
		pr_err("queue_%lu Invalid state! Q not in enabled state\n",
		       qid);
		snprintf(buf, buflen,
				"Error. Required Q state=%s, Current Q state=%s\n",
				q_state_list[Q_STATE_ENABLED].name,
				q_state_list[descq->q_state].name);
		unlock_descq(descq);
		return -EINVAL;
	}
	unlock_descq(descq);

	rv = qdma_validate_qconfig(xdev, qconf, buf, buflen);
	if (rv != 0)
		return rv;

	/** configure descriptor queue */
	qdma_descq_config(descq, qconf, 1);

	snprintf(buf, buflen,
		"Queue %s id %u is configured with the qconf passed ",
		descq->conf.name,
		descq->conf.qidx);

	return 0;
}

/*****************************************************************************/
/**
 * qdma_queue_list() - display all configured queues in a string buffer
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	success: if optional message buffer used then strlen of buf,
 *	otherwise 0
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_list(unsigned long dev_hndl, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_dev *qdev;
	struct qdma_descq *descq = NULL;
	char *cur = buf;
	char * const end = buf + buflen;
	uint32_t h2c_qcnt = 0, c2h_qcnt = 0, cmpt_qcnt = 0;
	int i;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	qdev = xdev_2_qdev(xdev);
	/** make sure that qdev is not NULL, else return error */
	if  (!qdev) {
		pr_err("dev %s, qdev null.\n", dev_name(&xdev->conf.pdev->dev));
		snprintf(buf, buflen, "Q not already added. Add Q first\n");
		return -EINVAL;
	}
#ifndef __QDMA_VF__
	h2c_qcnt = qdma_get_device_active_queue_count(
			xdev->conf.pdev->bus->number,
			xdev->func_id,
			QDMA_DEV_Q_TYPE_H2C);

	c2h_qcnt = qdma_get_device_active_queue_count(
			xdev->conf.pdev->bus->number,
			xdev->func_id,
			QDMA_DEV_Q_TYPE_C2H);

	cmpt_qcnt = qdma_get_device_active_queue_count(
			xdev->conf.pdev->bus->number,
			xdev->func_id,
			QDMA_DEV_Q_TYPE_CMPT);
#else
	qdma_dev_get_active_qcnt(xdev, &h2c_qcnt, &c2h_qcnt, &cmpt_qcnt);
#endif
	cur += snprintf(cur, end - cur, "H2C Q: %u, C2H Q: %u, CMPT Q %u.\n",
				h2c_qcnt, c2h_qcnt, cmpt_qcnt);
	if (cur >= end)
		goto handle_truncation;

	/** traverse through the h2c and c2h queue list
	 *  and dump the descriptors
	 */
	if (h2c_qcnt) {
		descq = qdev->h2c_descq;
		for (i = 0; i < h2c_qcnt; i++, descq++) {
			lock_descq(descq);
			if (descq->q_state != Q_STATE_DISABLED)
				cur +=
				qdma_descq_dump(descq, cur, end - cur, 0);
			unlock_descq(descq);

			if (cur >= end)
				goto handle_truncation;
		}
	}

	if (c2h_qcnt) {
		descq = qdev->c2h_descq;
		for (i = 0; i < c2h_qcnt; i++, descq++) {
			lock_descq(descq);
			if (descq->q_state != Q_STATE_DISABLED)
				cur +=
				qdma_descq_dump(descq, cur, end - cur, 0);
			unlock_descq(descq);

			if (cur >= end)
				goto handle_truncation;
		}
	}

	if (cmpt_qcnt) {
		descq = qdev->cmpt_descq;
		for (i = 0; i < cmpt_qcnt; i++, descq++) {
			lock_descq(descq);
			if (descq->q_state != Q_STATE_DISABLED)
				cur +=
				qdma_descq_dump(descq, cur, end - cur, 0);
			unlock_descq(descq);

			if (cur >= end)
				goto handle_truncation;
		}
	}

	return cur - buf;

handle_truncation:
	*buf = '\0';
	return buf - cur;
}

#define Q_PRESENT_H2C_MASK (1 << Q_H2C)
#define Q_PRESENT_C2H_MASK (1 << Q_C2H)
#define Q_PRESENT_CMPT_MASK (1 << Q_CMPT)
#define Q_MODE_SHIFT 3
#define Q_MODE_MASK  (1 << Q_MODE_SHIFT)

static int is_usable_queue(struct xlnx_dma_dev *xdev, int qidx,
		int q_type, int st)
{
	struct qdma_dev *qdev = xdev_2_qdev(xdev);
	int refmask = 0x0;
	int cmptq_chkmask = 0x0;
	int h2cq_chkmask = 0x0;
	int c2hq_chkmask = 0x0;
	int reqmask = 0;
	struct qdma_descq *h2c_descq = qdev->h2c_descq + qidx;
	struct qdma_descq *c2h_descq = qdev->c2h_descq + qidx;
	struct qdma_descq *cmpt_descq = qdev->cmpt_descq + qidx;

	lock_descq(h2c_descq);
	if (h2c_descq->q_state != Q_STATE_DISABLED)  {
		refmask |= (1 << Q_H2C);
		if (h2c_descq->conf.st)
			refmask |= (1 << Q_MODE_SHIFT);
	}
	unlock_descq(h2c_descq);

	lock_descq(c2h_descq);
	if (c2h_descq->q_state != Q_STATE_DISABLED)  {
		refmask |= (1 << Q_C2H);
		if (c2h_descq->conf.st)
			refmask |= (1 << Q_MODE_SHIFT);
	}
	unlock_descq(c2h_descq);

	lock_descq(cmpt_descq);
	if (cmpt_descq->q_state != Q_STATE_DISABLED)
		refmask |= (1 << Q_CMPT);
	unlock_descq(cmpt_descq);

	reqmask = (1 << q_type);
	if (st)
		reqmask |= Q_MODE_MASK;
	if (q_type == Q_CMPT) {
		cmptq_chkmask |= (Q_PRESENT_CMPT_MASK | Q_MODE_MASK);
		if (refmask & cmptq_chkmask)
			goto q_reject;
	} else if (q_type == Q_H2C) {
		c2hq_chkmask = (Q_PRESENT_C2H_MASK | Q_MODE_MASK);
		if (st && (refmask & Q_PRESENT_CMPT_MASK))
			goto q_reject; /* CMPT q given to MM */
		if (st && (refmask & Q_PRESENT_C2H_MASK)
				&& (refmask & c2hq_chkmask)
						== Q_PRESENT_C2H_MASK)
			goto q_reject; /* MM mode c2h q present*/
		if (!st && (refmask & Q_PRESENT_C2H_MASK)
				&& (refmask & c2hq_chkmask) == c2hq_chkmask)
			goto q_reject; /* ST mode c2h q present*/
		if (refmask & Q_PRESENT_H2C_MASK)
			goto q_reject; /* h2c q already present */
	} else {
		h2cq_chkmask |= (Q_PRESENT_H2C_MASK | Q_MODE_MASK);
		if (st && (refmask & Q_PRESENT_CMPT_MASK))
			goto q_reject; /* CMPT q given to MM */
		if (st && (refmask & Q_PRESENT_H2C_MASK)
				&& (refmask & h2cq_chkmask)
						== Q_PRESENT_H2C_MASK)
			goto q_reject; /* MM mode h2c q present*/
		if (!st && (refmask & Q_PRESENT_H2C_MASK)
				&& (refmask & h2cq_chkmask) == h2cq_chkmask)
			goto q_reject; /* ST mode h2c q present*/
		if (refmask & Q_PRESENT_C2H_MASK)
			goto q_reject; /* c2h q already present */
	}
	return 0;
q_reject:
	pr_err("Q addition feasibility check failed");
	return -1;
}

/*****************************************************************************/
/**
 * qdma_queue_add() - add a queue
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	qconf:		queue configuration parameters
 * @param[in]	qhndl:	list of unsigned long values that are the opaque qhndl
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_add(unsigned long dev_hndl, struct qdma_queue_conf *qconf,
			unsigned long *qhndl, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	unsigned int qcnt;
	struct qdma_descq *descq;
	struct qdma_dev *qdev;
#ifdef DEBUGFS
	struct qdma_descq *pairq;
#endif
#ifdef __QDMA_VF__
	int h2c_qcnt = 0, c2h_qcnt = 0, cmpt_qcnt = 0;
#endif
	int rv = 0;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/** If qconf is NULL, return error*/
	if (!qconf) {
		pr_err("Invalid qconf %p", qconf);
		snprintf(buf, buflen,
			"%s, add, qconf NULL.\n",
			xdev->conf.name);
		return -EINVAL;
	}

	/** If qhndl is NULL, return error*/
	if (!qhndl) {
		pr_warn("qhndl NULL.\n");
		snprintf(buf, buflen,
			"%s, add, qhndl NULL.\n",
			xdev->conf.name);
		return -EINVAL;
	}

	if (qconf->q_type > Q_CMPT) {
		pr_err("Invalid queue type passed");
		snprintf(buf, buflen, "Invalid queue type passed");
		return -EINVAL;
	}

	if (qconf->st && (qconf->q_type == Q_CMPT)) {
		pr_err("Can not create independent completion ring for ST mode. It is supported along with C2H direction");
		snprintf(buf, buflen,
				"Can not create independent completion ring for ST mode. It is supported along with C2H direction");
		return -EINVAL;
	}

	if ((qconf->q_type == Q_CMPT) && !xdev->dev_cap.mm_cmpt_en) {
		pr_err("MM Completions not enabled");
		snprintf(buf, buflen, "MM Completions not enabled");
		return -EINVAL;
	}

	qdev = xdev_2_qdev(xdev);

	/** reset qhandle to an invalid value
	 * can't use 0 or NULL here because queue idx 0 has same value
	 */
	*qhndl = QDMA_QUEUE_IDX_INVALID;

	/** check if the requested mode is enabled?
	 *  the request modes are read from the HW
	 *  before serving any request, first check if the
	 *  HW has the capability or not, else return error
	 */
	if ((qconf->st && !xdev->dev_cap.st_en) ||
	    (!qconf->st && !xdev->dev_cap.mm_en)) {
		pr_warn("%s, %s mode not enabled.\n",
			xdev->conf.name, qconf->st ? "ST" : "MM");
		snprintf(buf, buflen,
				"qdma%05x %s mode not enabled.\n",
				xdev->conf.bdf, qconf->st ? "ST" : "MM");
		return -EINVAL;
	}

	rv = qdma_validate_qconfig(xdev, qconf, buf, buflen);
	if (rv < 0)
		return rv;

	spin_lock(&qdev->lock);
	/** if incase the qidx is not QDMA_QUEUE_IDX_INVALID
	 *  then, make sure that the qidx range falls between
	 *  0 - qdev->qmax, else return error
	 */
	if ((qconf->qidx != QDMA_QUEUE_IDX_INVALID) &&
	    (qconf->qidx >= qdev->qmax)) {
		spin_unlock(&qdev->lock);
		snprintf(buf, buflen,
			"qdma%05x invalid idx %u >= %u.\n",
			xdev->conf.bdf, qconf->qidx, qdev->qmax);
		pr_err("Invalid queue index, qid = %d, qmax = %d",
					qconf->qidx, qdev->qmax);
		return -EINVAL;
	}

	/** check if any free qidx available
	 *  if qcnt is >= qdev->qmax, return error as
	 *  no free queues found and descq is full
	 */
#ifndef __QDMA_VF__
	qcnt = qdma_get_device_active_queue_count(xdev->conf.pdev->bus->number,
			xdev->func_id, qconf->q_type);
#else
	qdma_dev_get_active_qcnt(xdev, &h2c_qcnt, &c2h_qcnt, &cmpt_qcnt);
	if (qconf->q_type == Q_H2C)
		qcnt = h2c_qcnt;
	else if (qconf->q_type == Q_C2H)
		qcnt = c2h_qcnt;
	else
		qcnt = cmpt_qcnt;
#endif
	if (qcnt >= qdev->qmax) {
		spin_unlock(&qdev->lock);
		pr_warn("No free queues %u/%u.\n", qcnt, qdev->qmax);
		snprintf(buf, buflen,
			"qdma%05x No free queues %u/%u.\n",
			xdev->conf.bdf, qcnt, qdev->qmax);
		return -EIO;
	}


	spin_unlock(&qdev->lock);

	if (qconf->q_type == Q_C2H)
		descq = qdev->c2h_descq;
	else if (qconf->q_type == Q_H2C)
		descq = qdev->h2c_descq;
	else
		descq = qdev->cmpt_descq;

	/** need to allocate a free qidx if it has an invalid id
	 *  ie. qidx is not specified in the add request
	 */
	if (qconf->qidx == QDMA_QUEUE_IDX_INVALID) {
		int i;

		/** loop through the qlist till qmax and find an empty descq*/
		for (i = 0; i < qdev->qmax; i++, descq++) {
			if (is_usable_queue(xdev, i, qconf->q_type,
					qconf->st) < 0)
				continue;

			descq += i;
			lock_descq(descq);

			/** set the descq as enabled*/
			descq->q_state = Q_STATE_ENABLED;
			/** assign the qidx */
			qconf->qidx = i;
			unlock_descq(descq);

			break;
		}

		/** we are reached here means no free descq found
		 *  decrement the queue count and return error
		 */
		if (i == qdev->qmax) {
			pr_warn("no free %s qp found, %u.\n",
				qconf->st ? "ST" : "MM", qdev->qmax);
			rv = -EPERM;
			snprintf(buf, buflen,
				"qdma%05x no %s QP, %u.\n",
				xdev->conf.bdf,
				qconf->st ? "ST" : "MM", qdev->qmax);
			return rv;
		}
	} else {
		if (is_usable_queue(xdev, qconf->qidx, qconf->q_type,
				qconf->st) < 0) {
			pr_err("Queue compatibility check failed against existing queues\n");
			snprintf(buf, buflen,
				 "Queue compatibility check failed against existing queues\n");
			return -EPERM;
		}

		descq += qconf->qidx;

		lock_descq(descq);

		/** set the descq as enabled*/
		descq->q_state = Q_STATE_ENABLED;

		unlock_descq(descq);
	}

	/** prepare the queue resources*/
	rv = qdma_device_prep_q_resource(xdev);
	if (rv < 0) {
#ifdef __QDMA_VF__
		lock_descq(descq);
		descq->q_state = Q_STATE_DISABLED;
		unlock_descq(descq);
#endif
		return rv;
	}
#ifndef __QDMA_VF__
	if (xdev->conf.qdma_drv_mode == LEGACY_INTR_MODE) {
		rv = intr_legacy_setup(descq);
		if (rv > 0) {
			/** support only 1 queue in legacy interrupt mode */
			intr_legacy_clear(descq);
			descq->q_state = Q_STATE_DISABLED;
			pr_debug("qdma%05x - Q%u - No free queues %u/%u.\n",
				xdev->conf.bdf, descq->conf.qidx,
				rv, 1);
			rv = -EINVAL;
			snprintf(buf, buflen,
				"qdma%05x No free queues %u/%u.\n",
				xdev->conf.bdf, qcnt, 1);
			return rv;
		} else if (rv < 0) {
			rv = -EINVAL;
			descq->q_state = Q_STATE_DISABLED;
			pr_debug("qdma%05x Legacy interrupt setup failed.\n",
					xdev->conf.bdf);
			snprintf(buf, buflen,
				"qdma%05x Legacy interrupt setup failed.\n",
				xdev->conf.bdf);
			return rv;
		}
	}
#endif
	/** fill in config. info */
	qdma_descq_config(descq, qconf, 0);
#ifndef __QDMA_VF__
	rv = qdma_dev_increment_active_queue(
			xdev->conf.pdev->bus->number,
			xdev->func_id,
			qconf->q_type);
	if (rv < 0) {
		pr_err("Failed to increment active %s queue count",
				q_type_list[qconf->q_type].name);
		return rv;
	}
	if (qconf->st && (qconf->q_type == Q_C2H)) {
		rv = qdma_dev_increment_active_queue(
				xdev->conf.pdev->bus->number,
				xdev->func_id,
				QDMA_DEV_Q_TYPE_CMPT);
		if (rv < 0) {
			pr_err("Failed to increment CMPT queue count");
			qdma_dev_decrement_active_queue(
						xdev->conf.pdev->bus->number,
						xdev->func_id,
						qconf->q_type);
			return rv;
		}
	}
#else
	rv = qdma_dev_notify_qadd(descq, qconf->q_type);
	if (rv < 0) {
		pr_err("Failed to increment active %s queue count",
				q_type_list[qconf->q_type].name);
		return rv;
	}
	if (qconf->st && (qconf->q_type == Q_C2H)) {
		rv = qdma_dev_notify_qadd(descq, QDMA_DEV_Q_TYPE_CMPT);
		if (rv < 0) {
			pr_err("Failed to increment active CMPT queue count");
			qdma_dev_notify_qdel(descq, qconf->q_type);
			return rv;
		}
	}
#endif

	/** copy back the name in config*/
	memcpy(qconf->name, descq->conf.name, QDMA_QUEUE_NAME_MAXLEN);
	*qhndl = (unsigned long)descq->conf.qidx;
	if (qconf->q_type == Q_C2H)
		*qhndl += qdev->qmax;
	if (qconf->q_type == Q_CMPT)
		*qhndl += (2 * qdev->qmax);
	descq->q_hndl = *qhndl;


#ifdef DEBUGFS
	if (qconf->q_type != Q_CMPT) {
		if (qconf->q_type == Q_H2C)
			pairq = qdev->c2h_descq + qconf->qidx;
		else
			pairq = qdev->h2c_descq + qconf->qidx;

		rv = dbgfs_queue_init(&descq->conf, pairq,
				xdev->dbgfs_queues_root);
		if (rv < 0) {
			pr_err("failed to create queue debug files for the queueu %d\n",
					descq->conf.qidx);
		}
	}
#endif

	pr_debug("added %s, %s, qidx %u.\n",
			descq->conf.name,
			q_type_list[qconf->q_type].name,
			qconf->qidx);

	snprintf(buf, buflen, "%s %s added.\n",
		descq->conf.name,
		q_type_list[qconf->q_type].name);

	return 0;
}

/*****************************************************************************/
/**
 * qdma_queue_start() - start a queue (i.e, online, ready for dma)
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	id:		queue index
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_start(unsigned long dev_hndl, unsigned long id,
		     char *buf, int buflen)
{
	struct qdma_descq *descq;
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
	/** make sure that descq is not NULL, else return error*/
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		snprintf(buf, buflen,
			"Invalid qid(%ld)\n", id);
		return -EINVAL;
	}

	lock_descq(descq);
	/** if the descq is not enabled,
	 *  it is in invalid state, return error
	 */
	if (descq->q_state != Q_STATE_ENABLED) {
		pr_err("%s invalid state, q_status%d.\n",
			descq->conf.name, descq->q_state);
		snprintf(buf, buflen,
			"%s invalid state, q_state %d.\n",
			descq->conf.name, descq->q_state);
		unlock_descq(descq);
		return -EINVAL;
	}
	unlock_descq(descq);

	/** complete the queue configuration*/
	rv = qdma_descq_config_complete(descq);
	if (rv < 0) {
		pr_err("%s 0x%x setup failed.\n",
			descq->conf.name, descq->qidx_hw);
		snprintf(buf, buflen,
			"%s config failed.\n", descq->conf.name);
		return -EIO;
	}

	/** allocate the queue resources*/
	rv = qdma_descq_alloc_resource(descq);
	if (rv < 0) {
		pr_err("%s alloc resource failed.\n", descq->conf.name);
		snprintf(buf, buflen,
			"%s alloc resource failed.\n",
			descq->conf.name);
		return rv;
	}

	/** program the hw contexts*/
	rv = qdma_descq_prog_hw(descq);
	if (rv < 0) {
		pr_err("%s 0x%x setup failed.\n",
			descq->conf.name, descq->qidx_hw);
		snprintf(buf, buflen,
			"%s prog. context failed.\n",
			descq->conf.name);
		goto clear_context;
	}

	/** Interrupt mode */
	if (descq->xdev->num_vecs) {
		unsigned long flags;
		struct intr_info_t *dev_intr_info_list =
			&descq->xdev->dev_intr_info_list[descq->intr_id];

		spin_lock_irqsave(&dev_intr_info_list->vec_q_list, flags);
		list_add_tail(&descq->intr_list,
				&dev_intr_info_list->intr_list);
		dev_intr_info_list->intr_list_cnt++;
		spin_unlock_irqrestore(&dev_intr_info_list->vec_q_list, flags);
	}

#ifdef CONFIG_HIGH_RES_TIMERS
	hrtimer_init(&descq->pidx_upd_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	descq->pidx_upd_timer.function = &qdma_descq_pidx_upd_timer_cb;
	INIT_WORK(&descq->pidx_upd_work,
			  qdma_descq_pidx_update_handler);
#else
	INIT_DELAYED_WORK(&descq->pidx_upd_work,
			  qdma_descq_pidx_update_handler);
#endif
	qdma_thread_add_work(descq);

	snprintf(buf, buflen, "queue %s, idx %u started\n",
			descq->conf.name, descq->conf.qidx);

	/** set the descq to online state*/
	lock_descq(descq);
	descq->q_state = Q_STATE_ONLINE;
	unlock_descq(descq);

	return 0;

clear_context:
	qdma_descq_context_clear(descq->xdev, descq->qidx_hw,
					descq->conf.st, descq->conf.q_type, 1);
	qdma_descq_free_resource(descq);

	return rv;
}

int qdma_get_queue_state(unsigned long dev_hndl, unsigned long id,
		struct qdma_q_state *q_state, char *buf, int buflen)
{
	struct qdma_descq *descq;
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
	/** make sure that descq is not NULL, else return error */
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		snprintf(buf, buflen,
			"Invalid qid(%ld)\n", id);
		return -EINVAL;
	}

	if (!q_state) {
		pr_err("Invalid q_state:%p", q_state);
		return -EINVAL;
	}
	lock_descq(descq);
	/** mode */
	q_state->st = descq->conf.st;
	/** type */
	q_state->q_type = descq->conf.q_type;
	/** qidx */
	q_state->qidx = descq->conf.qidx;
	/** q state */
	q_state->qstate = descq->q_state;

	snprintf(buf, buflen,
		"queue state for %s id %u : %s\n",
		descq->conf.name,
		descq->conf.qidx,
		q_state_list[descq->q_state].name);

	unlock_descq(descq);

	return 0;
}



/*****************************************************************************/
/**
 * qdma_queue_stop() - stop a queue (i.e., offline, NOT ready for dma)
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	id:		queue index
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_stop(unsigned long dev_hndl, unsigned long id, char *buf,
			int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	struct qdma_sgt_req_cb *cb, *tmp;
	struct qdma_request *req;
	unsigned int pend_list_empty = 0;

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	descq = qdma_device_get_descq_by_id(xdev, id, buf, buflen, 1);
	/** make sure that descq is not NULL, else return error */
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		return -EINVAL;
	}

	lock_descq(descq);
		/** if the descq not online donot proceed */
	if (descq->q_state != Q_STATE_ONLINE) {
		unlock_descq(descq);
		pr_err("%s invalid state, q_state %d.\n",
		descq->conf.name, descq->q_state);
		snprintf(buf, buflen,
			"queue %s, idx %u stop failed.\n",
			 descq->conf.name, descq->conf.qidx);
		return -EINVAL;
	}
	pend_list_empty = descq->pend_list_empty;

	descq->q_stop_wait = 1;
	unlock_descq(descq);
	if (!pend_list_empty) {
		qdma_waitq_wait_event_timeout(descq->pend_list_wq,
			descq->pend_list_empty,
			msecs_to_jiffies(QDMA_Q_PEND_LIST_COMPLETION_TIMEOUT));
	}
	lock_descq(descq);
	/** free the descq by updating the state */
	descq->q_state = Q_STATE_ENABLED;
	descq->q_stop_wait = 0;
#ifdef CONFIG_HIGH_RES_TIMERS
	hrtimer_try_to_cancel(&descq->pidx_upd_timer);
#else
	cancel_delayed_work(&descq->pidx_upd_work);
#endif
	list_for_each_entry_safe(cb, tmp, &descq->pend_list, list) {
		req = (struct qdma_request *)cb;
		cb->done = 1;
		cb->status = -ENXIO;
		if (req->fp_done) {
			list_del(&cb->list);
			req->fp_done(req, 0, -ENXIO);
		} else
			qdma_waitq_wakeup(&cb->wq);
	}
	list_for_each_entry_safe(cb, tmp, &descq->work_list, list) {
		req = (struct qdma_request *)cb;
		cb->done = 1;
		cb->status = -ENXIO;
		if (req->fp_done) {
			list_del(&cb->list);
			req->fp_done(req, 0, -ENXIO);
		} else
			qdma_waitq_wakeup(&cb->wq);
	}
	unlock_descq(descq);

	/** remove the work thread associated with the current queue */
	qdma_thread_remove_work(descq);

	/** clear the queue context */
	qdma_descq_context_clear(descq->xdev, descq->qidx_hw,
					descq->conf.st, descq->conf.q_type, 0);

	/** if the device is in direct/indirect interrupt mode,
	 *  delete the interrupt list for the queue
	 */
	if ((descq->xdev->conf.qdma_drv_mode != POLL_MODE) &&
		(descq->xdev->conf.qdma_drv_mode != LEGACY_INTR_MODE)) {
		unsigned long flags;
		struct intr_info_t *dev_intr_info_list =
			&descq->xdev->dev_intr_info_list[descq->intr_id];

		spin_lock_irqsave(&dev_intr_info_list->vec_q_list, flags);
		list_del(&descq->intr_list);
		dev_intr_info_list->intr_list_cnt--;
		spin_unlock_irqrestore(&dev_intr_info_list->vec_q_list, flags);
	}

	/** free the queue resources */
	qdma_descq_free_resource(descq);
	/** free the descq by updating the state */
	descq->total_cmpl_descs = 0;

	/** fill the return buffer indicating that queue is stopped */
	snprintf(buf, buflen, "queue %s, idx %u stopped.\n",
			descq->conf.name, descq->conf.qidx);

	return 0;
}

/*****************************************************************************/
/**
 * qdma_intr_ring_dump() - display the interrupt ring info of a vector
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	vector_idx:	vector number
 * @param[in]	start_idx:	interrupt ring start idx
 * @param[in]	end_idx:	interrupt ring end idx
 * @param[in]	buflen:		length of the input buffer
 * @param[out]	buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_intr_ring_dump(unsigned long dev_hndl, unsigned int vector_idx,
	int start_idx, int end_idx, char *buf, int buflen)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	union qdma_intr_ring *ring_entry;
	struct intr_coal_conf *coal_entry;
	char *cur = buf;
	char * const end = buf + buflen;
	int counter = 0;
	u32 data[2];

	/** make sure that input buffer is not empty, else return error */
	if (!buf || !buflen) {
		pr_err("invalid argument: buf=%p, buflen=%d", buf, buflen);
		return -EINVAL;
	}

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		snprintf(buf, buflen, "dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		snprintf(buf, buflen, "Invalid dev_hndl passed");
		return -EINVAL;
	}

	/** if, interrupt aggregation is not enabled,
	 *  interrupt ring is not created, return error
	 */
	if ((xdev->conf.qdma_drv_mode != INDIRECT_INTR_MODE) &&
			(xdev->conf.qdma_drv_mode != AUTO_MODE)) {
		pr_err("Interrupt aggregation not enabled in %s mode",
				mode_name_list[xdev->conf.qdma_drv_mode].name);
		snprintf(buf, buflen,
				"Interrupt aggregation not enabled in %s mode",
				mode_name_list[xdev->conf.qdma_drv_mode].name);
		return -EINVAL;
	}

	/** make sure that vector index is with in the
	 *  start and end vector limit, else return error
	 */
	if ((vector_idx < xdev->dvec_start_idx) ||
		(vector_idx >=
		(xdev->dvec_start_idx + QDMA_NUM_DATA_VEC_FOR_INTR_CXT))) {
		pr_err("Vector idx %d is invalid. Shall be in range: %d -  %d.\n",
			vector_idx,
			xdev->dvec_start_idx,
			(xdev->dvec_start_idx +
			QDMA_NUM_DATA_VEC_FOR_INTR_CXT - 1));
		snprintf(buf, buflen,
			"Vector idx %d is invalid. Shall be in range: %d -  %d.\n",
			vector_idx,
			xdev->dvec_start_idx,
			(xdev->dvec_start_idx +
			QDMA_NUM_DATA_VEC_FOR_INTR_CXT - 1));
		return -EINVAL;
	}

	/** get the intr entry based on vector index */
	coal_entry = xdev->intr_coal_list + (vector_idx - xdev->dvec_start_idx);

	/** make sure that intr ring entry indexes
	 *  given are with in the range
	 */
	if (start_idx > coal_entry->intr_rng_num_entries) {
		pr_err("start_idx %d is invalid. Shall be less than: %d\n",
					start_idx,
					coal_entry->intr_rng_num_entries);
		snprintf(buf, buflen,
			"start_idx %d is invalid. Shall be less than: %d\n",
			start_idx,
			coal_entry->intr_rng_num_entries);
		return -EINVAL;
	}

	if (end_idx == -1 || end_idx >= coal_entry->intr_rng_num_entries)
		end_idx = coal_entry->intr_rng_num_entries - 1;

	if (start_idx == -1)
		start_idx = 0;

	if (start_idx > end_idx) {
		pr_err("start_idx can't be greater than end_idx\n");
		snprintf(buf, buflen,
			"start_idx can't be greater than end_idx\n");
		return -EINVAL;
	}

	/** read the ring entries based on the range given and
	 * update the input buffer with details
	 */
	for (counter = start_idx; counter <= end_idx; counter++) {
		ring_entry = coal_entry->intr_ring_base + counter;
		memcpy(data, ring_entry, sizeof(u32) * 2);
		cur += snprintf(cur, end - cur,
				"intr_ring_entry = %d: 0x%08x 0x%08x\n",
				counter, data[1], data[0]);
		if (cur >= end)
			goto handle_truncation;
	}

	return 0;
handle_truncation:
	*buf = '\0';
	return 0;
}

 /*****************************************************************************/
 /**
  * qdma_software_version_info  Provides the qdma software version
  *
  * @param[out]   software_version:   libqdma software version
  *
  * @return  0: success
  * @return  <0: error
  *****************************************************************************/
int qdma_software_version_info(char *software_version)
{
	if (!software_version) {
		pr_err("Invalid input software_version:%p", software_version);
		return -EINVAL;
	}

	sprintf(software_version, "%s", LIBQDMA_VERSION_STR);

	return 0;
}


#ifdef __QDMA_VF__
 /*****************************************************************************/
 /**
  * qdma_vf_qconf    call for VF to request qmax number of Qs
  *
  * @param[in]   dev_hndl:   dev_hndl returned from qdma_device_open()
  * @param[in]   qmax:   number of qs requested by vf
  *
  * @return  0: success
  * @return  <0: error
  *****************************************************************************/
int qdma_vf_qconf(unsigned long dev_hndl, int qmax)
{
	int err = 0;
	int qbase = -1;
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	if (!xdev) {
		pr_err("Invalid dev handle\n");
		return -EINVAL;
	}

	err = device_set_qconf(xdev, &qmax, &qbase);
	if (err < 0) {
		pr_err("Setting qconf failed\n");
		return err;
	}
	qdma_device_cleanup(xdev);
	xdev->conf.qsets_base = qbase;
	xdev->conf.qsets_max = qmax;
	err  = qdma_device_init(xdev);
	if (err < 0) {
		pr_warn("qdma_init failed %d.\n", err);
		qdma_device_cleanup(xdev);
	}

	return err;
}
#endif

/*****************************************************************************/
/**
 * sgl_unmap() - unmap the sg list from host pages
 *
 * @param[in]	pdev:	pointer to struct pci_dev
 * @param[in]	sg:		qdma sg request list
 * @param[in]	sgcnt:	number of sg lists
 * @param[in]	dir:	direction of the dma transfer
 *			DMA_BIDIRECTIONAL = 0,	DMA_TO_DEVICE = 1,
 *			DMA_FROM_DEVICE = 2, DMA_NONE = 3,
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
void sgl_unmap(struct pci_dev *pdev, struct qdma_sw_sg *sg, unsigned int sgcnt,
		 enum dma_data_direction dir)
{
	int i;

	/** unmap the sg list and set the dma_addr to 0 all sg entries */
	for (i = 0; i < sgcnt; i++, sg++) {
		if (!sg->pg)
			break;
		if (sg->dma_addr) {
			pci_unmap_page(pdev, sg->dma_addr - sg->offset,
							PAGE_SIZE, dir);
			sg->dma_addr = 0UL;
		}
	}
}

/*****************************************************************************/
/**
 * sgl_map() - map sg list to host pages
 *
 * @param[in]	pdev:	pointer to struct pci_dev
 * @param[in]	sg:		qdma sg request list
 * @param[in]	sgcnt:	number of sg lists
 * @param[in]	dir:	direction of the dma transfer
 *			DMA_BIDIRECTIONAL = 0,	DMA_TO_DEVICE = 1,
 *			DMA_FROM_DEVICE = 2, DMA_NONE = 3,
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int sgl_map(struct pci_dev *pdev, struct qdma_sw_sg *sgl, unsigned int sgcnt,
		enum dma_data_direction dir)
{
	int i;
	struct qdma_sw_sg *sg = sgl;

	/** Map the sg list onto a dma pages where
	 *  each page has max of PAGE_SIZE i.e 4K
	 */
	for (i = 0; i < sgcnt; i++, sg++) {
		/* !! TODO  page size !! */
		sg->dma_addr = pci_map_page(pdev, sg->pg, 0, PAGE_SIZE, dir);
		if (unlikely(pci_dma_mapping_error(pdev, sg->dma_addr))) {
			pr_err("map sgl failed, sg %d, %u.\n", i, sg->len);
			if (i)
				sgl_unmap(pdev, sgl, i, dir);
			return -EIO;
		}
		sg->dma_addr += sg->offset;
	}

	return 0;
}

/*****************************************************************************/
/**
 * qdma_request_submit() - submit a scatter-gather list of data for dma
 * operation (for both read and write)
 * This is a callback function called from upper layer(character device)
 * to handle the read/write request on the queues
 *
 * @param[in]	dev_hndl:	hndl retured from qdma_device_open()
 * @param[in]	id:			queue index
 * @param[in]	req:		qdma request
 *
 * @return	# of bytes transferred
 * @return	<0: error
 *****************************************************************************/
ssize_t qdma_request_submit(unsigned long dev_hndl, unsigned long id,
			struct qdma_request *req)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	struct qdma_sgt_req_cb *cb;
	enum dma_data_direction dir;
	int wait = 0;
	int rv = 0;

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		return -EINVAL;
	}

	if (!req) {
		pr_err("req is NULL");
		return -EINVAL;
	}

	wait = req->fp_done ? 0 : 1;

	descq = qdma_device_get_descq_by_id(xdev, id, NULL, 0, 1);
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		return -EINVAL;
	}

	if (descq->conf.q_type == Q_CMPT) {
		pr_err("Error : Transfer initiated on completion queue\n");
		return -EINVAL;
	}

	cb = qdma_req_cb_get(req);

	pr_debug("%s %s-%s, data len %u, sg cnt %u.\n",
		descq->conf.name, descq->conf.st ? "ST" : "MM",
		(descq->conf.q_type == Q_C2H) ? "C2H" : "H2C",
		req->count, req->sgcnt);

	/** Identify the direction of the transfer */
	dir = (descq->conf.q_type == Q_C2H) ?  DMA_FROM_DEVICE : DMA_TO_DEVICE;

	/** If write request is given on the C2H direction
	 *  OR, a read request given on non C2H direction
	 *  then, its an invalid request, return error in this case
	 */
	if ((req->write && (descq->conf.q_type != Q_H2C)) ||
	    (!req->write && (descq->conf.q_type != Q_C2H))) {
		pr_err("%s: bad direction, %c.\n",
			descq->conf.name, req->write ? 'W' : 'R');
		return -EINVAL;
	}

	/** Reset the local cb request with 0's */
	memset(cb, 0, QDMA_REQ_OPAQUE_SIZE);
	/** Initialize the wait queue */
	qdma_waitq_init(&cb->wq);

	pr_debug("%s: data len %u, ep 0x%llx, sgl 0x%p, sgl cnt %u, tm %u ms.\n",
		descq->conf.name, req->count, req->ep_addr, req->sgl,
		req->sgcnt, req->timeout_ms);

	/** If the request is streaming mode C2H, invoke the
	 *  handler to perform the read operation
	 */
	if (descq->conf.st && (descq->conf.q_type == Q_C2H))
		return qdma_request_submit_st_c2h(xdev, descq, req);

	if (!req->dma_mapped) {
		rv = sgl_map(xdev->conf.pdev,  req->sgl, req->sgcnt, dir);
		if (rv < 0) {
			pr_err("%s map sgl %u failed, %u.\n",
				descq->conf.name, req->sgcnt, req->count);
			goto unmap_sgl;
		}
		cb->unmap_needed = 1;
	}

	lock_descq(descq);
	/**  if the descq is already in online state*/
	if (descq->q_state != Q_STATE_ONLINE) {
		unlock_descq(descq);
		pr_err("%s descq %s NOT online.\n",
			xdev->conf.name, descq->conf.name);
		rv = -EINVAL;
		goto unmap_sgl;
	}
	list_add_tail(&cb->list, &descq->work_list);
	descq->pend_req_desc += ((req->count + PAGE_SIZE - 1) >> PAGE_SHIFT);
	unlock_descq(descq);

	pr_debug("%s: cb 0x%p submitted.\n", descq->conf.name, cb);

	qdma_descq_proc_sgt_request(descq);

	if (!wait)
		return 0;

	rv = qdma_request_wait_for_cmpl(xdev, descq, req);
	if (rv < 0)
		goto unmap_sgl;

	return cb->offset;

unmap_sgl:
	if (!req->dma_mapped)
		sgl_unmap(xdev->conf.pdev,  req->sgl, req->sgcnt, dir);

	return rv;
}

/*****************************************************************************/
/**
 * qdma_batch_request_submit() - submit a batch of scatter-gather list of data
 *  for dma operation (for both read and write)
 * This is a callback function called from upper layer(character device)
 * to handle the read/write request on the queues
 *
 * @param[in]	dev_hndl:	hndl retured from qdma_device_open()
 * @param[in]	id:			queue index
 * @param[in]	count:		Number of qdma requests
 * @param[in]	reqv:		qdma request vector
 *
 * @return	# of bytes transferred
 * @return	<0: error
 *****************************************************************************/
ssize_t qdma_batch_request_submit(unsigned long dev_hndl, unsigned long id,
			  unsigned long count, struct qdma_request **reqv)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;
	struct qdma_sgt_req_cb *cb;
	enum dma_data_direction dir;
	int rv = 0;
	unsigned long i;
	struct qdma_request *req;
	int st_c2h = 0;

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		return -EINVAL;
	}

	if (!reqv) {
		pr_err("reqv is NULL");
		return -EINVAL;
	}

	descq = qdma_device_get_descq_by_id(xdev, id, NULL, 0, 0);
	if (!descq) {
		pr_err("Invalid qid(%ld)", id);
		return -EINVAL;
	}

	if (descq->conf.q_type == Q_CMPT) {
		pr_err("Error : Transfer initiated on completion queue\n");
		return -EINVAL;
	}

	st_c2h = (descq->conf.st && (descq->conf.q_type == Q_C2H)) ? 1 : 0;

	/** Identify the direction of the transfer */
	dir = (descq->conf.q_type == Q_C2H) ?  DMA_FROM_DEVICE : DMA_TO_DEVICE;

	req = reqv[0];
	/** If write request is given on the C2H direction
	 *  OR, a read request given on non C2H direction
	 *  then, its an invalid request, return error in this case
	 */
	if ((req->write && (descq->conf.q_type != Q_H2C))
			|| (!req->write && (descq->conf.q_type != Q_C2H))) {
		pr_err("%s: bad direction, %c.\n", descq->conf.name,
				req->write ? 'W' : 'R');
		return -EINVAL;
	}
	if (!req->fp_done) {
		pr_err("%s: missing fp_done.\n", descq->conf.name);
		return -EINVAL;
	}

	if (st_c2h) {
		for (i = 0; i < count; i++) {
			req = reqv[i];
			cb = qdma_req_cb_get(req);
			/** Reset the local cb request with 0's */
			memset(cb, 0, QDMA_REQ_OPAQUE_SIZE);

			rv = qdma_request_submit_st_c2h(xdev, descq, req);
			if ((rv < 0) || (rv == req->count))
				req->fp_done(req, rv, rv);
		}

		return 0;

	} else {
		struct pci_dev *pdev = xdev->conf.pdev;

		for (i = 0; i < count; i++) {
			req = reqv[i];
			cb = qdma_req_cb_get(req);
			/** Reset the local cb request with 0's */
			memset(cb, 0, QDMA_REQ_OPAQUE_SIZE);

			if (!req->dma_mapped) {
				rv = sgl_map(pdev, req->sgl, req->sgcnt, dir);
				if (unlikely(rv < 0)) {
					pr_info("%s map sgl %u failed, %u.\n",
						descq->conf.name,
						req->sgcnt,
						req->count);
					req->fp_done(req, 0, rv);
				}
				cb->unmap_needed = 1;
			}
		}
	}

	lock_descq(descq);
	/**  if the descq is already in online state*/
	if (unlikely(descq->q_state != Q_STATE_ONLINE)) {
		unlock_descq(descq);
		pr_err("%s descq %s NOT online.\n", xdev->conf.name,
				descq->conf.name);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		req = reqv[i];
		cb = qdma_req_cb_get(req);

		list_add_tail(&cb->list, &descq->work_list);
	}
	unlock_descq(descq);

	qdma_descq_proc_sgt_request(descq);

	return 0;
}

/*****************************************************************************/
/**
 * libqdma_init()       initialize the QDMA core library
 *
 * @param[in] num_threads - number of threads to be created each for request
 *  processing and writeback processing
 *
 * @return	0:	success
 * @return	<0:	error
 *****************************************************************************/
int libqdma_init(unsigned int num_threads, void *debugfs_root)
{

	int ret;
	/** Make sure that the size of qdma scatter gather request size
	 *  is less than the QDMA_REQ_OPAQUE_SIZE
	 *  If not, return error
	 */
	if (sizeof(struct qdma_sgt_req_cb) > QDMA_REQ_OPAQUE_SIZE) {
		pr_err("dma req. opaque data size too big %lu > %d.\n",
			sizeof(struct qdma_sgt_req_cb), QDMA_REQ_OPAQUE_SIZE);
		return -1;
	}
	if (sizeof(struct qdma_flq) > QDMA_FLQ_SIZE) {
		pr_err("qdma flq data size too big %lu > %d",
		       sizeof(struct qdma_flq), QDMA_FLQ_SIZE);
		return -1;
	}

	/** Create the qdma threads */
	ret = qdma_threads_create(num_threads);
	if(ret < 0){
		pr_err("qdma_threads_create failed for num_thread=%d",
			       num_threads);
		return ret;
	}
#ifdef DEBUGFS

	if (debugfs_root) {
		qdma_debugfs_root = debugfs_root;
		qdma_debufs_cleanup = false;
		return 0;
	}

	ret =  qdma_debugfs_init(&qdma_debugfs_root);
	if (ret < 0) {
		pr_err("qdma_debugfs_init failed for num_thread=%d",
				num_threads);
		return ret;
	}
#endif
	return 0;
}

/*****************************************************************************/
/**
 * libqdma_exit()	cleanup the QDMA core library before exiting
 *
 * @return	none
 *****************************************************************************/
void libqdma_exit(void)
{
#ifdef DEBUGFS
	if (qdma_debufs_cleanup)
		qdma_debugfs_exit(qdma_debugfs_root);
#endif
	/** Destroy the qdma threads */
	qdma_threads_destroy();
}

#ifdef __LIBQDMA_MOD__
/** for module support only */
#include "version.h"

static char version[] =
	DRV_MODULE_DESC " " DRV_MODULE_NAME " v" DRV_MODULE_VERSION "\n";

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION(DRV_MODULE_DESC);
MODULE_VERSION(DRV_MODULE_VERSION);
MODULE_LICENSE("Dual BSD/GPL");

/*****************************************************************************/
/**
 * libqdma_mod_init()	libqdma module initialization routine
 *
 * Returns: none
 *****************************************************************************/
static int __init libqdma_mod_init(void)
{
	pr_info("%s", version);

	/** invoke libqdma_init to initialize the libqdma library */
	return libqdma_init();
}

/*****************************************************************************/
/**
 * libqdma_mod_exit()       cleanup the QDMA core library before exiting
 *
 * Returns: none
 *****************************************************************************/
static void __exit libqdma_mod_exit(void)
{
	/** invoke libqdma_exit to cleanup the libqdma library resources */
	libqdma_exit();
}

module_init(libqdma_mod_init);
module_exit(libqdma_mod_exit);
#endif /* ifdef __LIBQDMA_MOD__ */
