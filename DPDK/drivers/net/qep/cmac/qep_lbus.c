/*
 * Copyright(c) 2020 Xilinx, Inc. All rights reserved.
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

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/slab.h>
#define PRIu64 "llu"
#define lbus_err pr_err
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#define lbus_err printf
#endif

#include "qep_cmac.h"

#define QEP_LBUS_BUF_LEN 2048

#define QEP_LBUS_BASE 0x00030000

#define QEP_LBUS_DROP_LSB 0x00000020
#define QEP_AXIS_DROP_LSB 0x00000028
#define QEP_AXIS_OUT_LSB 0x00000030
#define QEP_LBUS_STAT_PM    0x00000038
#define QEP_LBUS_RX_ERR  0x00000010
#define QEP_LBUS_RX_STATUS 0x00000014
#define QEP_LBUS_RX_CTL 0x0000003c
#define QEP_LBUS_TX_ERR  0x00000040
#define QEP_LBUS_TX_STATUS 0x00000044
#define QEP_LBUS_TX_CTL 0x00000048

struct qep_lbus_err {
	uint32_t rsv1:13;
	uint32_t invalid_bus_err:1;
	uint32_t drop_bus_err:1;
	uint32_t drop_fifo_full:1;
	uint32_t rsv2:12;
	uint32_t data_fifo_empty:1;
	uint32_t data_fifo_full:1;
	uint32_t ctl_fifo_empty:1;
	uint32_t ctl_fifo_full:1;
};

struct qep_lbus_status {
	uint32_t rsv1:14;
	uint32_t cmac_err:1;
	uint32_t backpressure:1;
	uint32_t rsv2:10;
	uint32_t axis_intf_idle:1;
	uint32_t lbus_intf_idle:1;
	uint32_t data_fifo_full:1;
	uint32_t ctl_fifo_in_reset:1;
	uint32_t data_fifo_in_reset:1;
	uint32_t module_in_reset:1;
};

struct qep_lbus_rx_ctl {
	uint32_t rsv1:30;
	uint32_t pass_trunc:1;
	uint32_t pass_err:1;
};

struct qep_lbus_tx_ctl {
	uint32_t rsv1:31;
	uint32_t padd_en:1;
};

struct qep_lbus_rx_stats {
	uint64_t lbus_drop;
	uint64_t axis_drop;
	uint64_t axis_out;
};

struct qep_lbus_rx {
	struct qep_lbus_err err;
	struct qep_lbus_status status;
	struct qep_lbus_rx_ctl ctl;
	struct qep_lbus_rx_stats stats;
};

struct qep_lbus_tx {
	struct qep_lbus_err err;
	struct qep_lbus_status status;
	struct qep_lbus_tx_ctl ctl;
};


static int qep_lbus_pm_tick(uint64_t base)
{
	uint64_t offset;

	offset = base + QEP_LBUS_BASE + QEP_LBUS_STAT_PM;
	xcmac_out32(offset, 0x1);

	return 0;
}

static uint64_t qep_lbus_read64(uint64_t offset)
{
	uint64_t data = 0;

	data = xcmac_in32(offset + 4);
	data = data << 31;
	data |=  xcmac_in32(offset);

	return data;
}

static int qep_lbus_get_rx_stats(uint64_t base, struct qep_lbus_rx_stats *stats)
{
	uint64_t offset;

	if (!stats) {
		lbus_err("%s: Invalid input param", __func__);
		return -EINVAL;
	}

	qep_lbus_pm_tick(base);

	offset = base + QEP_LBUS_BASE;
	stats->lbus_drop = qep_lbus_read64(offset + QEP_LBUS_DROP_LSB);
	stats->axis_drop = qep_lbus_read64(offset + QEP_AXIS_DROP_LSB);
	stats->axis_out = qep_lbus_read64(offset + QEP_AXIS_OUT_LSB);

	return 0;
}

static int qep_lbus_get_rx(uint64_t base, struct qep_lbus_rx *lbus_rx)
{
	uint64_t offset;
	uint32_t *val;
	int ret;

	if (!lbus_rx) {
		lbus_err("%s: Invalid input param", __func__);
		return -EINVAL;
	}

	ret = qep_lbus_get_rx_stats(base, &lbus_rx->stats);
	if (ret) {
		lbus_err("%s: qep_lbus_get_rx_stats failed", __func__);
		return ret;
	}

	offset = base + QEP_LBUS_BASE;

	val = (uint32_t *)&lbus_rx->err;
	*val = xcmac_in32(offset + QEP_LBUS_RX_ERR);

	val = (uint32_t *)&lbus_rx->status;
	*val = xcmac_in32(offset + QEP_LBUS_RX_STATUS);

	val = (uint32_t *)&lbus_rx->ctl;
	*val = xcmac_in32(offset + QEP_LBUS_RX_CTL);

	return 0;
}

static int qep_lbus_get_tx(uint64_t base, struct qep_lbus_tx *lbus_tx)
{
	uint64_t offset;
	uint32_t *val;

	if (!lbus_tx) {
		lbus_err("%s: Invalid input param", __func__);
		return -EINVAL;
	}

	offset = base + QEP_LBUS_BASE;

	val = (uint32_t *)&lbus_tx->err;
	*val = xcmac_in32(offset + QEP_LBUS_TX_ERR);

	val = (uint32_t *)&lbus_tx->status;
	*val = xcmac_in32(offset + QEP_LBUS_TX_STATUS);

	val = (uint32_t *)&lbus_tx->ctl;
	*val = xcmac_in32(offset + QEP_LBUS_TX_CTL);

	return 0;
}

static void qep_lbus_snprintf_err(struct qep_lbus_err *err,
		enum xcmac_core_type core, char *buf, int len)
{
	if (!buf || !err || len <= 0) {
		lbus_err("%s: Invalid input param", __func__);
		return;
	}

	snprintf(buf + strlen(buf), len - strlen(buf),
		"Error : Ctl_FIFO_Full=%u Ctl_FIFO_Empty=%u Data_FIFO_Full=%u Data_FIFO_Empty=%u\n",
		err->ctl_fifo_full, err->ctl_fifo_empty, err->data_fifo_full,
		err->data_fifo_full);

	if (core == XCMAC_CORE_RX) {
		snprintf(buf + strlen(buf), len - strlen(buf),
			"Packet Drop Indication : FIFO_full=%u lbus_err=%u Invalid_bus=%u\n",
			err->drop_fifo_full, err->drop_bus_err,
			err->invalid_bus_err);
	} else {
		snprintf(buf + strlen(buf), len - strlen(buf),
			"Error : Lbus_overflow =%u lbus_underflow=%u\n",
			err->drop_fifo_full, err->drop_bus_err);
	}
}

static void qep_lbus_snprintf_status(struct qep_lbus_status *status,
		enum xcmac_core_type core, char *buf, int len)
{
	if (!buf || !status || len <= 0) {
		lbus_err("%s: Invalid input param", __func__);
		return;
	}

	snprintf(buf + strlen(buf), len - strlen(buf),
		"Status : Module_In_Reset= %u Data_FIFO_in_reset=%u Control_FIFO_in_Reset=%u Data_FIFO_Full=%u axis_idle=%u lbus_idle=%u\n",
		status->module_in_reset, status->data_fifo_in_reset,
		status->ctl_fifo_in_reset, status->data_fifo_full,
		status->axis_intf_idle, status->lbus_intf_idle);
	if (core == XCMAC_CORE_RX) {
		snprintf(buf + strlen(buf), len - strlen(buf),
			"Packet Drop Cause : Backpressure=%u Cmac Err=%u\n",
			status->backpressure, status->cmac_err);
	}
}

static int qep_lbus_snprint_rx(struct qep_lbus_rx *lbus_rx, char *buf, int len)
{

	if (!buf || !lbus_rx || len <= 0) {
		lbus_err("%s: Invalid input param", __func__);
		return -EINVAL;
	}

	snprintf(buf + strlen(buf), len - strlen(buf),
		"RX(LBUS_to_AXIS) Info\n");
	snprintf(buf + strlen(buf), len - strlen(buf),
		"Config: Pass_Err=%u Pass_Truncate=%u\n",
		lbus_rx->ctl.pass_err, lbus_rx->ctl.pass_trunc);

	qep_lbus_snprintf_err(&lbus_rx->err, XCMAC_CORE_RX, buf + strlen(buf),
		len - strlen(buf));

	qep_lbus_snprintf_status(&lbus_rx->status, XCMAC_CORE_RX,
		buf + strlen(buf), len - strlen(buf));

	snprintf(buf + strlen(buf), len - strlen(buf),
		"Statistics :  axis_out=%" PRIu64 " axis_drop=%" PRIu64
		" lbus_drop=%" PRIu64 "\n",
		lbus_rx->stats.axis_out, lbus_rx->stats.axis_drop,
		lbus_rx->stats.lbus_drop);

	return 0;
}

static int qep_lbus_snprint_tx(struct qep_lbus_tx *lbus_tx, char *buf, int len)
{
	if (!buf || !lbus_tx || len <= 0) {
		lbus_err("%s: Invalid input param", __func__);
		return -EINVAL;
	}

	snprintf(buf + strlen(buf), len - strlen(buf),
		"TX(AXIS_to_LBUS) Info\n");
	snprintf(buf + strlen(buf), len - strlen(buf),
		"Config : Padd_Enable=%u\n", lbus_tx->ctl.padd_en);

	qep_lbus_snprintf_err(&lbus_tx->err, XCMAC_CORE_TX, buf + strlen(buf),
			len - strlen(buf));

	qep_lbus_snprintf_status(&lbus_tx->status, XCMAC_CORE_TX,
			buf + strlen(buf), len - strlen(buf));

	return 0;
}


int qep_lbus_get_buf_len(void)
{
	return QEP_LBUS_BUF_LEN;
}

int qep_lbus_snprintf(void *addr, char *buf, int len)
{
	struct qep_lbus_rx lbus_rx;
	struct qep_lbus_tx lbus_tx;
	int ret;
	uint64_t base;

	if (!addr || !buf || len <  qep_lbus_get_buf_len()) {
		lbus_err("%s: Invalid input param", __func__);
		return -EINVAL;
	}

	base = (uint64_t)addr;
	memset(&lbus_rx, 0, sizeof(struct qep_lbus_rx));
	memset(&lbus_tx, 0, sizeof(struct qep_lbus_tx));

	ret = qep_lbus_get_rx(base, &lbus_rx);
	if (ret) {
		lbus_err("%s: qep_lbus_get_rx failed", __func__);
		return ret;
	}

	ret = qep_lbus_get_tx(base, &lbus_tx);
	if (ret) {
		lbus_err("%s: qep_lbus_get_tx failed", __func__);
		return ret;
	}

	ret = qep_lbus_snprint_rx(&lbus_rx, buf, len);
	if (ret) {
		lbus_err("%s: qep_lbus_snprint_rx failed", __func__);
		return ret;
	}

	ret = qep_lbus_snprint_tx(&lbus_tx, buf + strlen(buf),
		len - strlen(buf));
	if (ret) {
		lbus_err("%s: qep_lbus_snprint_tx failed", __func__);
		return ret;
	}

	return 0;
}
