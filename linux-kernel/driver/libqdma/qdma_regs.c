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

#include "qdma_regs.h"

#include <linux/version.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/stddef.h>
#include <linux/string.h>

#include "xdev.h"
#include "qdma_device.h"
#include "qdma_descq.h"
#include "qdma_st_c2h.h"
#include "qdma_access.h"
#include "qdma_reg.h"
#include "qdma_resource_mgmt.h"

int qdma_set_ring_sizes(struct xlnx_dma_dev *xdev, u8 index,
		u8 count, u32 *glbl_rng_sz)
{
	int i = 0;

	if (!xdev || !glbl_rng_sz)
		return -EINVAL;

	/* Adding 1 for the wrap around descriptor and status descriptor */
	for (i = index; i < (index + count); i++)
		*(glbl_rng_sz + i) += 1;

	if (qdma_set_global_ring_sizes(xdev, index, count, glbl_rng_sz))
		return -EINVAL;

	return 0;
}

int qdma_get_ring_sizes(struct xlnx_dma_dev *xdev, u8 index,
		u8 count, u32 *glbl_rng_sz)
{
	int i = 0;

	if (!xdev || !glbl_rng_sz)
		return -EINVAL;

	if (qdma_get_global_ring_sizes(xdev, index, count, glbl_rng_sz))
		return -EINVAL;

	/* Subtracting 1 for the wrap around descriptor and status descriptor */
	for (i = index; i < (index + count); i++)
		*(glbl_rng_sz + i) -= 1;

	return 0;
}

/*****************************************************************************/
/**
 * qdma_device_read_config_register() - read dma config. register
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	reg_addr:	register address
 *
 * @return	value of the config register
 *****************************************************************************/
unsigned int qdma_device_read_config_register(unsigned long dev_hndl,
					unsigned int reg_addr)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	if (!xdev)
		return -EINVAL;

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0)
		return -EINVAL;

	return readl(xdev->regs + reg_addr);
}

/*****************************************************************************/
/**
 * qdma_device_write_config_register() - write dma config. register
 *
 * @param[in]	dev_hndl:	dev_hndl returned from qdma_device_open()
 * @param[in]	reg_addr:	register address
 * @param[in]	value:		register value to be writen
 *
 *****************************************************************************/
void qdma_device_write_config_register(unsigned long dev_hndl,
				unsigned int reg_addr, unsigned int val)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	if (!xdev)
		return;

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0)
		return;

	pr_debug("%s reg 0x%x, w 0x%08x.\n", xdev->conf.name, reg_addr, val);
	writel(val, xdev->regs + reg_addr);
}


#ifdef __QDMA_VF__
#include "qdma_mbox.h"

int qdma_csr_read(struct xlnx_dma_dev *xdev, struct global_csr_conf *csr)
{
	struct mbox_msg *m = qdma_mbox_msg_alloc();
	int rv, i;
	struct qdma_csr_info csr_info;

	if (!m)
		return -ENOMEM;

	memset(&csr_info, 0, sizeof(csr_info));

	qdma_mbox_compose_csr_read(xdev->func_id, m->raw);
	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0)
		goto free_msg;

	rv = qdma_mbox_vf_csr_get(m->raw, &csr_info);
	if (!rv) {
		for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
			csr->ring_sz[i] = (uint32_t)csr_info.ringsz[i];
			csr->c2h_buf_sz[i] = (uint32_t)csr_info.bufsz[i];
			csr->c2h_timer_cnt[i] = (uint32_t)csr_info.timer_cnt[i];
			csr->c2h_cnt_th[i] = (uint32_t)csr_info.cnt_thres[i];
		}
		csr->wb_intvl = csr_info.wb_intvl;
	} else
		pr_err("csr info read failed");


free_msg:
	qdma_mbox_msg_free(m);
	return rv;
}

#ifdef QDMA_CSR_REG_UPDATE
int qdma_global_csr_set(unsigned long dev_hndl, u8 index, u8 count,
		struct global_csr_conf *csr)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0)
		return -EINVAL;

	pr_info("VF %d setting csr NOT allowed.\n", xdev->func_id);
	return -EINVAL;
}
#endif
#else /* ifdef __QDMA_VF__ */

void qdma_sort_c2h_cntr_th_values(struct xlnx_dma_dev *xdev)
{
	uint8_t i, idx = 0, j = 0;
	uint8_t c2h_cntr_val = xdev->csr_info.c2h_cnt_th[0];
	uint8_t least_max = 0;
	int ref_idx = -1;

get_next_idx:
	for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
		if ((ref_idx >= 0) && (ref_idx == i))
			continue;
		if (xdev->csr_info.c2h_cnt_th[i] < least_max)
			continue;
		c2h_cntr_val = xdev->csr_info.c2h_cnt_th[i];
		idx = i;
		break;
	}
	for ( ; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++) {
		if ((ref_idx >= 0) && (ref_idx == i))
			continue;
		if (xdev->csr_info.c2h_cnt_th[i] < least_max)
			continue;
		if (c2h_cntr_val >= xdev->csr_info.c2h_cnt_th[i]) {
			c2h_cntr_val = xdev->csr_info.c2h_cnt_th[i];
			idx = i;
		}
	}
	xdev->sorted_c2h_cntr_idx[j] = idx;
	ref_idx = idx;
	j++;
	idx = j;
	least_max = c2h_cntr_val;
	if (j < QDMA_GLOBAL_CSR_ARRAY_SZ)
		goto get_next_idx;
	for (i = 0; i < QDMA_GLOBAL_CSR_ARRAY_SZ; i++)
		pr_debug("%u - %u", xdev->sorted_c2h_cntr_idx[i], xdev->csr_info.c2h_cnt_th[xdev->sorted_c2h_cntr_idx[i]]);
}

int qdma_csr_read(struct xlnx_dma_dev *xdev, struct global_csr_conf *csr)
{
	qdma_get_global_writeback_interval(xdev, &csr->wb_intvl);
	qdma_get_global_ring_sizes(xdev, 0,
			QDMA_GLOBAL_CSR_ARRAY_SZ, csr->ring_sz);
	qdma_get_global_buffer_sizes(xdev, 0,
			QDMA_GLOBAL_CSR_ARRAY_SZ, csr->c2h_buf_sz);
	qdma_get_global_timer_count(xdev, 0,
			QDMA_GLOBAL_CSR_ARRAY_SZ, csr->c2h_timer_cnt);
	qdma_get_global_counter_threshold(xdev, 0, QDMA_GLOBAL_CSR_ARRAY_SZ,
			csr->c2h_cnt_th);
	qdma_sort_c2h_cntr_th_values(xdev);

	return 0;
}

#ifdef QDMA_CSR_REG_UPDATE
int qdma_global_csr_set(unsigned long dev_hndl, u8 index, u8 count,
		struct global_csr_conf *csr)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0)
		return -EINVAL;

	/** If qdma_get_active_queue_count() > 0,
	 *  cannot modify global CSR.
	 */
	if (qdma_get_active_queue_count(xdev->conf.pdev->bus->number)) {
		pr_err("xdev %s, FMAP prog done, cannot modify global CSR\n",
				xdev->mod_name);
		return -EINVAL;
	}
	if (qdma_set_global_writeback_interval(xdev, csr->wb_intvl))
		return -EINVAL;
	if (qdma_set_ring_sizes(xdev, index, count, csr->ring_sz))
		return -EINVAL;
	if (qdma_set_global_timer_count(xdev, index, count, csr->c2h_timer_cnt))
		return -EINVAL;
	if (qdma_set_global_counter_threshold(xdev, index, count,
			csr->c2h_cnt_th))
		return -EINVAL;
	if (qdma_set_global_buffer_sizes(xdev, index, count, csr->c2h_buf_sz))
		return -EINVAL;

	qdma_csr_read(xdev, &xdev->csr_info);
	return 0;
}
#endif
#endif

int qdma_global_csr_get(unsigned long dev_hndl, u8 index, u8 count,
		struct global_csr_conf *csr)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int i = 0;

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0)
		return -EINVAL;
	/** If qdma_get_active_queue_count() > 0,
	 *  read the stored xdev csr values.
	 */
	if (qdma_get_active_queue_count(xdev->conf.pdev->bus->number))
		memcpy(csr, &xdev->csr_info, sizeof(struct global_csr_conf));
	else
		qdma_csr_read(xdev, csr);

	/** Subtracting 1 for the wrap around descriptor and status descriptor
	 *  This same logic is present in qdma_get/set_ring_sizes() API
	 *  In case of any changes to this number, make sure to update in
	 *  all these places
	 */
	for (i = index; i < (index + count); i++)
		csr->ring_sz[i]--;

	return 0;
}

int qdma_device_flr_quirk_set(struct pci_dev *pdev, unsigned long dev_hndl)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv;

	if (!xdev->dev_cap.flr_present) {
		pr_info("FLR not present, therefore skipping FLR reset\n");
		return 0;
	}

	if (!dev_hndl || xdev_check_hndl(__func__, pdev, dev_hndl) < 0)
		return -EINVAL;

#ifndef __QDMA_VF__
	rv = qdma_initiate_flr(xdev, 0);
#else
	rv = qdma_initiate_flr(xdev, 1);
#endif
	if (rv)
		return -EINVAL;

	return 0;
}

int qdma_device_flr_quirk_check(struct pci_dev *pdev, unsigned long dev_hndl)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	int rv;
	uint8_t flr_done = 0;

	if (!xdev->dev_cap.flr_present) {
		pr_info("FLR not present, therefore skipping FLR reset status\n");
		return 0;
	}

	if (!dev_hndl || xdev_check_hndl(__func__, pdev, dev_hndl) < 0)
		return -EINVAL;

#ifndef __QDMA_VF__
	rv = qdma_is_flr_done(xdev, 0, &flr_done);
#else
	rv = qdma_is_flr_done(xdev, 1, &flr_done);
#endif
	if (rv)
		return -EINVAL;

	if (!flr_done)
		pr_info("%s, flr status stuck\n", xdev->conf.name);

	return 0;
}

int qdma_device_flr_quirk(struct pci_dev *pdev, unsigned long dev_hndl)
{
	int rv;

	rv = qdma_device_flr_quirk_set(pdev, dev_hndl);
	if (rv  < 0)
		return rv;

	return qdma_device_flr_quirk_check(pdev, dev_hndl);
}

int qdma_device_version_info(unsigned long dev_hndl,
		struct qdma_version_info *version_info)
{
	int rv = 0;
	struct qdma_hw_version_info info;
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;

	if (!xdev || !version_info)
		return -EINVAL;

	memset(&info, 0, sizeof(info));
#ifndef __QDMA_VF__
	rv = qdma_get_version(xdev, QDMA_DEV_PF, &info);
#else
	rv = qdma_get_version(xdev, QDMA_DEV_VF, &info);
#endif
	if (rv < 0)
		return rv;

	strncpy(version_info->everest_ip_str,
			qdma_get_everest_ip(info.everest_ip),
			sizeof(version_info->everest_ip_str) - 1);
	strncpy(version_info->rtl_version_str,
			qdma_get_rtl_version(info.rtl_version),
			sizeof(version_info->rtl_version_str) - 1);
	strncpy(version_info->vivado_release_str,
			qdma_get_vivado_release_id(info.vivado_release),
			sizeof(version_info->vivado_release_str) - 1);
	return 0;
}

#ifndef __QDMA_VF__
void qdma_device_attributes_get(struct xlnx_dma_dev *xdev)
{
	struct qdma_dev_attributes dev_info;

	memset(&dev_info, 0, sizeof(dev_info));

	qdma_get_device_attributes(xdev, &dev_info);

	xdev->dev_cap.num_pfs = dev_info.num_pfs;
	xdev->dev_cap.num_qs = dev_info.num_qs;
	xdev->dev_cap.flr_present = dev_info.flr_present;
	xdev->dev_cap.st_en = dev_info.st_en;
	xdev->dev_cap.mm_en = dev_info.mm_en;
	xdev->dev_cap.mm_cmpt_en = dev_info.mm_cmpt_en;
	xdev->dev_cap.mailbox_en = dev_info.mailbox_en;
	xdev->dev_cap.mm_channel_max = dev_info.mm_channel_max;

	pr_info("%s: num_pfs:%d, num_qs:%d, flr_present:%d, st_en:%d, mm_en:%d, mm_cmpt_en:%d, mailbox_en:%d, mm_channel_max:%d",
		xdev->conf.name,
		xdev->dev_cap.num_pfs,
		xdev->dev_cap.num_qs,
		xdev->dev_cap.flr_present,
		xdev->dev_cap.st_en,
		xdev->dev_cap.mm_en,
		xdev->dev_cap.mm_cmpt_en,
		xdev->dev_cap.mailbox_en,
		xdev->dev_cap.mm_channel_max);
}
#endif

int qdma_queue_cmpl_ctrl(unsigned long dev_hndl, unsigned long id,
			struct qdma_cmpl_ctrl *cctrl, bool set)
{
	int rv = 0;
	struct qdma_descq *descq =  qdma_device_get_descq_by_id(
					(struct xlnx_dma_dev *)dev_hndl,
					id, NULL, 0, 1);

	if (!descq)
		return QDMA_ERR_INVALID_QIDX;

	if (set) {
		lock_descq(descq);

		descq->cmpt_cidx_info.trig_mode =
				descq->conf.cmpl_trig_mode =
						cctrl->trigger_mode;
		descq->cmpt_cidx_info.timer_idx =
				descq->conf.cmpl_timer_idx = cctrl->timer_idx;
		descq->cmpt_cidx_info.counter_idx =
				descq->conf.cmpl_cnt_th_idx = cctrl->cnt_th_idx;
		descq->cmpt_cidx_info.irq_en =
				descq->conf.cmpl_en_intr = cctrl->cmpl_en_intr;
		descq->cmpt_cidx_info.wrb_en =
				descq->conf.cmpl_stat_en = cctrl->en_stat_desc;

		descq->cmpt_cidx_info.wrb_cidx = descq->cidx_cmpt;
		rv = queue_cmpt_cidx_update(descq->xdev,
				descq->conf.qidx, &descq->cmpt_cidx_info);
		if (rv < 0) {
			pr_err("%s: Failed to update cmpt cidx\n",
					descq->conf.name);
			return rv;
		}

		unlock_descq(descq);

	} else {
		/* read the setting */
		rv = queue_cmpt_cidx_read(descq->xdev,
				descq->conf.qidx, &descq->cmpt_cidx_info);
		if (rv < 0)
			return rv;

		cctrl->trigger_mode = descq->conf.cmpl_trig_mode =
				descq->cmpt_cidx_info.trig_mode;
		cctrl->timer_idx = descq->conf.cmpl_timer_idx =
				descq->cmpt_cidx_info.timer_idx;
		cctrl->cnt_th_idx = descq->conf.cmpl_cnt_th_idx =
				descq->cmpt_cidx_info.counter_idx;
		cctrl->en_stat_desc = descq->conf.cmpl_stat_en =
				descq->cmpt_cidx_info.wrb_en;
		cctrl->cmpl_en_intr = descq->conf.cmpl_en_intr =
				descq->cmpt_cidx_info.irq_en;
	}

	return 0;
}

#ifndef __QDMA_VF__
int hw_init_qctxt_memory(struct xlnx_dma_dev *xdev, unsigned int qbase,
		unsigned int qmax)
{
	u32 data[QDMA_REG_IND_CTXT_REG_COUNT];
	unsigned int i = qbase;

	memset(data, 0, sizeof(u32) * QDMA_REG_IND_CTXT_REG_COUNT);

	for (; i < qmax; i++) {
		enum ind_ctxt_cmd_sel sel = QDMA_CTXT_SEL_SW_C2H;
		int rv;

		for ( ; sel <= QDMA_CTXT_SEL_PFTCH; sel++) {
			/** if the st mode(h2c/c2h) not enabled in the design,
			 *  then skip the PFTCH and CMPT context setup
			 */
			if ((xdev->dev_cap.st_en == 0) &&
			    (sel == QDMA_CTXT_SEL_PFTCH ||
				sel == QDMA_CTXT_SEL_CMPT)) {
				pr_debug("ST context is skipped: sel = %d",
					 sel);
				continue;
			}

			switch (sel) {
			case QDMA_CTXT_SEL_SW_C2H:
				rv = qdma_sw_context_clear(xdev, 1, i);
				break;
			case QDMA_CTXT_SEL_SW_H2C:
				rv = qdma_sw_context_clear(xdev, 0, i);
				break;
			case QDMA_CTXT_SEL_HW_C2H:
				rv = qdma_hw_context_clear(xdev, 1, i);
				break;
			case QDMA_CTXT_SEL_HW_H2C:
				rv = qdma_hw_context_clear(xdev, 0, i);
				break;
			case QDMA_CTXT_SEL_CR_C2H:
				rv = qdma_credit_context_clear(xdev, 1, i);
				break;
			case QDMA_CTXT_SEL_CR_H2C:
				rv = qdma_credit_context_clear(xdev, 0, i);
				break;
			case QDMA_CTXT_SEL_CMPT:
				rv = qdma_cmpt_context_clear(xdev, i);
				break;
			case QDMA_CTXT_SEL_PFTCH:
				rv = qdma_pfetch_context_clear(xdev, i);
				break;
			case QDMA_CTXT_SEL_INT_COAL:
			case QDMA_CTXT_SEL_PASID_RAM_LOW:
			case QDMA_CTXT_SEL_PASID_RAM_HIGH:
			case QDMA_CTXT_SEL_TIMER:
			case QDMA_CTXT_SEL_FMAP:
				break;
			}
			if (rv < 0)
				return rv;
		}
	}

	return 0;
}
#endif
