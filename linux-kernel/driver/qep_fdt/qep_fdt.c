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
#ifdef __KERNEL__
#define qep_fdt_err pr_err
#define qep_fdt_info pr_info
#define qep_fdt_warn pr_warn
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#define PRIu64 "llu"
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <assert.h>
#define qep_fdt_err printf
#define qep_fdt_info printf
#define qep_fdt_warn printf
#endif

#include "qep_fdt.h"

static const char *qep_netpf_ip_names[QEP_NETPF_MAX_BLOCKS] = {
	"qep_qdma_00",
	"qep_stmn_00",
	"qep_rss_00",
	"qep_sec_block_rx_00",
	"qep_checsum_00",
	"qep_platform_info",
	"qep_mailbox.q",
	"qap_top_00",
	"base_0",
	"cmac_ctrl_0",
	"cmac_usplus_0",
	"lbus_axis_conv_0",
	"qdma_user_h2c_0",
	"qdma_user_c2h_0",
	"axi_gpio_qsfp0",
	"iic_0",
	"cmac_drp_0",
	"cmac_drp_1",
	"pci_drp_0",
	"pcie_rqrc_mon_0"
};

static const char *qep_mgmtpf_ip_names[QEP_MGMTPF_MAX_BLOCKS] = {
	"qep_sec_block_tx_01",
	"qep_sec_block_rx_01",
	"qap_top_01",
	"qep_qsfp_iic_00",
};

static const char *qep_board_names[QEP_BOARD_TYPE_MAX] = {
	"1525",
	"u250",
	"u200",
	"u50"
};

static const char *qep_ip_prop_names[QEP_FDT_PROP_MAX] = {
	"pcie_physical_function",
	"pcie_base_address_register",
	"enabled",
	"offset",
	"range",

	"version",
	"qeueue_max",
	"qeueue_base",
	"msixVectorCount",
	"C2HBypass",

	"H2CBypass",
	"master_pf",
	"poll_mode",
	"rsfec_en",
	"1588_ptp",

	"qsfp_port",
	"qsfp_bandwidth",
	"mgmt_pcie_physical_function",
	"network_pcie_physical_function",
	"design_name"
};

static const char *qep_design_name[QEP_DESIGN_MAX] = {
	"qep",
	"qap",
	"qap_stmn"
};

static struct qep_platform_info default_pinfo = {
	.mgmt_pf = 0,
	.net_pf = 2,
	.qdma_bar = 2,
	.user_bar = 0,
	.stmn_queue_base = 256,
	.stmn_queue_max = 256,
	.pci_msix_vec_count = 32,
	.c2h_bypass = 1,
	.h2c_bypass = 1,
	.master_pf = 0,
	.poll_mode = 0,
	.rsfec_en = 1,
	.ptp_1588 = 0,
	.bandwith = QEP_100G,
	.qsfp_port = 0,
	.board = QEP_BOARD_U250,
	.design = QEP_DESIGN_QEP,
	.mgmt_ip = { {1, 2, 0x200000, 0x0, "2.1" },
			{1, 2, 0x203000, 0x0, "2.1" },
			{0, 2, 0x0, 0xfffff, "0.0" },
			{0, 2, 0x41000, 0x0, "0.0" }
	},
	.net_ip = {
		{1, 0, 0x0, 0xfffff, "4.0" }, /*QEP_NETPF_QDMA*/
		{1, 2, 0xd0000, 0xdffff, "2.1" }, /*QEP_NETPF_STMN*/
		{1, 2, 0x300000, 0x3fffff, "2.1" }, /*QEP_NETPF_RSS */
		{1, 2, 0x200000, 0x2fffff, "2.1" }, /*QEP_NETPF_SEC_RX*/
		{1, 2, 0x0, 0x0, "2.1" }, /*QEP_NETPF_CHECKSUM*/
		{1, 2, 0x110000, 0x0, "2.1" }, /*QEP_NETPF_PLATFORM_INFO*/
		{1, 2, 0x01F00000, 0x01F000ff, "0.0" }, /*QEP_NETPF_MAILBOX*/
		{1, 2, 0x0, 0xfffff, "0.0" }, /*QEP_NETPF_QAP_TOP*/
		{1, 2, 0x0, 0xffff, "0.0" }, /*QEP_NETPF_QAP_BASE*/
		{1, 2, 0x10000, 0x1ffff, "0.0" }, /*QEP_NETPF_CMAC_CTL*/
		{1, 2, 0x20000, 0x2ffff, "2.4" }, /*QEP_NETPF_CMAC_IP*/
		{1, 2, 0x30000, 0x3ffff, "0.0" }, /*QEP_NETPF_LBUS_CONV*/
		{1, 2, 0x40000, 0x4ffff, "0.0" }, /*QEP_NETPF_QDMA_USER_H2C*/
		{1, 2, 0x50000, 0x5ffff, "0.0" }, /*QEP_NETPF_QDMA_USER_C2H*/
		{0, 2, 0x70000, 0x7ffff, "2.0" }, /*QEP_NETPF_GPIO_QSFP*/
		{0, 2, 0x80000, 0x8ffff, "2.0" }, /*QEP_NETPF_IIC_QSFP*/
		{1, 2, 0xa0000, 0xa3fff, "0.0" }, /*QEP_NETPF_CMAC_DRP0*/
		{1, 2, 0xa4000, 0xa7fff, "0.0" }, /*QEP_NETPF_CMAC_DRP1*/
		{0, 2, 0xb0000, 0xbffff, "0.0" }, /*QEP_NETPF_PCIE_DRP0*/
		{0, 2, 0xc0000, 0xcffff, "0.0" }, /*QEO_NETPF_PCIE_RQRC_MON*/
	},
};


static void _qep_fdt_free(void *mem)
{
#ifdef __KERNEL__
	kfree(mem);
#else
	free(mem);
#endif
}

static void *_qep_fdt_alloc(int size)
{
#ifdef __KERNEL__
	return kzalloc(size, GFP_KERNEL);
#else
	return malloc(size);
#endif
}

const char *qep_fdt_get_name(enum qep_fdt_name_type type, unsigned int idx)
{
	const char *name = NULL;

	switch (type) {
	case QEP_NAME_NET_IP:
		if (idx < QEP_NETPF_MAX_BLOCKS)
			name = qep_netpf_ip_names[idx];
		break;
	case QEP_NAME_MGMT_IP:
		if (idx < QEP_MGMTPF_MAX_BLOCKS)
			name = qep_mgmtpf_ip_names[idx];
		break;
	case QEP_NAME_IP_PROP:
		if (idx < QEP_FDT_PROP_MAX)
			name = qep_ip_prop_names[idx];
		break;
	case QEP_NAME_BOARD:
		if (idx < QEP_BOARD_TYPE_MAX)
			name = qep_board_names[idx];
		break;
	case QEP_NAME_DESIGN:
		if (idx < QEP_DESIGN_MAX)
			name = qep_design_name[idx];
		break;
	default:
		break;
	}

	return name;
}
static int _find_idx(const char *name, const char **arr,
		int arrlen)
{
	int i = 0;

	for (i = 0 ; i < arrlen; i++) {
		if (strcmp(name, arr[i]) == 0)
			return i;
	}

	return -1;
}

int qep_fdt_find_index(enum qep_fdt_name_type type, const char *name)
{
	int idx = -1;

	switch (type) {
	case QEP_NAME_NET_IP:
		idx = _find_idx(name, qep_netpf_ip_names, QEP_NETPF_MAX_BLOCKS);
		break;
	case QEP_NAME_MGMT_IP:
		idx = _find_idx(name, qep_mgmtpf_ip_names,
				QEP_MGMTPF_MAX_BLOCKS);
		break;
	case QEP_NAME_IP_PROP:
		idx = _find_idx(name, qep_ip_prop_names, QEP_FDT_PROP_MAX);
		break;
	case QEP_NAME_BOARD:
		idx = _find_idx(name, qep_board_names, QEP_BOARD_TYPE_MAX);
		break;
	case QEP_NAME_DESIGN:
		idx = _find_idx(name, qep_design_name, QEP_DESIGN_MAX);
		break;
	default:
		break;
	}
	return  idx;
}

#ifdef __KERNEL__
int qep_fdt_read_metadata(const char *fname, char **buf_ref, unsigned long *len)
{
	int err = 0;
	char fw_name[256];
	char *fw_buf = NULL;
	const struct firmware *fw;

	if (!buf_ref || !len) {
		qep_fdt_err(" %s: invalid args\n", __func__);
		return -EINVAL;
	}

	snprintf(fw_name, sizeof(fw_name), "xilinx/%s", fname);

	err = request_firmware(&fw, fw_name, NULL);
	if (err) {
		pr_info("%s: request_firmware failed : %d\n", __func__, err);
		return err;
	}

	fw_buf = kzalloc(fw->size, GFP_KERNEL);
	if (fw_buf != NULL)
		memcpy(fw_buf, fw->data, fw->size);
	else {
		err = -ENOMEM;
		goto func_exit;
	}

	*buf_ref = fw_buf;
	*len = fw->size;

func_exit:
	release_firmware(fw);
	return err;
}
#else
int qep_fdt_read_metadata(const char *fname, char **buf_ref, unsigned long *len)
{
	int ret;
	FILE *fp;
	long int length;
	char *buf = NULL;

	if (!buf_ref || !len) {
		qep_fdt_info(" %s: invalid args\n", __func__);
		return -EINVAL;
	}

	fp = fopen(fname, "r");
	if (!fp)
		return -ENOENT;

	ret = fseek(fp, 0L, SEEK_END);
	if (ret) {
		fclose(fp);
		return -EPERM;
	}

	length = ftell(fp);
	if (length <= 0) {
		fclose(fp);
		return -EPERM;
	}

	ret = fseek(fp, 0L, SEEK_SET);
	if (ret) {
		fclose(fp);
		return -EPERM;
	}

	buf =  malloc(sizeof(char) * (length + 1));
	if (!buf) {
		fclose(fp);
		return -ENOMEM;
	}

	ret = fread(buf, 1, length, fp);
	if (ret != length) {
		fclose(fp);
		return -EFAULT;
	}

	*buf_ref = buf;
	*len = length;

	fclose(fp);
	return 0;
}
#endif


static long int _parse_int(const char *data, int base, const char *info)
{
	long int num;
#ifdef __KERNEL__
	int ret;
#endif
	char *end = NULL;

	if (!data)
		return -EINVAL;
#ifdef __KERNEL__
	ret = kstrtoul(data, base, &num);
	if (ret) {
		qep_fdt_err("%s: %s is not numeric string:%s End:%s\n",
			__func__, info, data, end);
		return -EINVAL;
	}
#else
	num = strtoul(data, &end, base);
	if (strlen(end)) {
		qep_fdt_err("%s: %s is not numeric string:%s End:%s\n",
			__func__, info, data, end);
		return -EINVAL;
	}
#endif
	return num;
}
int qep_fdt_validate_schema(char *blob)
{
	int node;
	const char *data;
	long major, minor;

	if (!blob) {
		qep_fdt_err("%s: fdt blob is null\n", __func__);
		return -EINVAL;
	}

	/* Since this node will remain same, no need to add to enums and names*/
	node = fdt_subnode_offset(blob, 0, "schema_version");

	data = fdt_getprop(blob, node, "major", NULL);
	major = _parse_int(data, 16, "schema_version major");
	if (major == -EINVAL) {
		qep_fdt_err("%s: schema_version major is invalid\n", __func__);
		return -EINVAL;
	}

	data = fdt_getprop(blob, node, "minor", NULL);
	minor = _parse_int(data, 16,  "schema_version minor");
	if (minor == -EINVAL) {
		qep_fdt_err("%s: schema_version major is invalid\n", __func__);
		return -EINVAL;
	}

	if ((major != QEP_JSON_SCHEMA_VERSION_MAJOR) &&
		(minor != QEP_JSON_SCHEMA_VERSION_MINOR)) {
		qep_fdt_err("%s: schema_version mismatch found: %d %d Supported: %ld %ld\n",
				__func__, QEP_JSON_SCHEMA_VERSION_MAJOR,
				QEP_JSON_SCHEMA_VERSION_MINOR, major, minor);
		return -EINVAL;
	}

	return 0;
}


long qep_fdt_get_prop_long(char *blob, int node, enum qep_fdt_name_type type,
		unsigned int idx, int base)
{
	const char *data, *fdt_name = NULL;
	long val;

	if (!blob) {
		qep_fdt_err("%s: fdt blob is null\n", __func__);
		return -EINVAL;
	}

	fdt_name = qep_fdt_get_name(type, idx);
	if (!fdt_name) {
		qep_fdt_err("%s: Prop type:%d idx:%d name not found\n",
				__func__, type, idx);
		return -EINVAL;
	}

	/* Property may not exit for the node */
	data = fdt_getprop(blob, node, fdt_name, NULL);
	if (!data)
		return -EINVAL;


	val = _parse_int(data, base,  fdt_name);
	if (val == -EINVAL) {
		qep_fdt_err("%s: %s prop value invalid type:%d idx:%d\n",
				__func__, fdt_name, type, idx);
		return -EINVAL;
	}

	return val;
}

const char *qep_fdt_get_prop_str(char *blob, int node,
		enum qep_fdt_name_type type,
		unsigned int idx)
{
	const char *data, *fdt_name = NULL;

	if (!blob) {
		qep_fdt_err("%s: fdt blob is null\n", __func__);
		return NULL;
	}

	fdt_name = qep_fdt_get_name(type, idx);
	if (!fdt_name) {
		qep_fdt_err("%s: Prop type:%d idx:%d name not found\n",
				__func__, type, idx);
		return NULL;
	}

	data = fdt_getprop(blob, node, fdt_name, NULL);
	if (!data) {
		qep_fdt_err("%s: %s value is NULL, type:%d idx:%d\n",
				__func__, fdt_name, type, idx);
		return NULL;
	}

	return data;
}

long qep_fdt_get_prop_qdma(char *blob, int parent,
		struct qep_platform_info *pinfo)
{
	int node;
	long val;

	if (!pinfo || !blob) {
		qep_fdt_err("%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	node = fdt_subnode_offset(blob, parent, "qep_qdma_00");
	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
				QEP_FDT_PROP_BAR, QEP_BASE_16);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->qdma_bar = val;

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_MSIX_VEC, QEP_BASE_10);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->pci_msix_vec_count = val;


	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_C2H_BYPASS, QEP_BASE_10);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->c2h_bypass = val;


	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_H2C_BYPASS, QEP_BASE_10);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->h2c_bypass = val;

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_MASTER_PF, QEP_BASE_10);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->master_pf = val;

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_POLL_MODE, QEP_BASE_10);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->poll_mode = val;


	return 0;
}

long qep_fdt_get_prop_stmn(char *blob, int parent,
		struct qep_platform_info *pinfo)
{
	int node;
	long val;

	if (!pinfo || !blob) {
		qep_fdt_err("%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	node = fdt_subnode_offset(blob, parent, "qep_stmn_00");

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_BAR, QEP_BASE_16);
	if (val == -EINVAL)
		return -EINVAL;

	pinfo->user_bar = val;

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_QMAX, QEP_BASE_10);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->stmn_queue_max = val;

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_QBASE, QEP_BASE_10);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->stmn_queue_base = val;

	return 0;
}

long qep_fdt_get_prop_cmac(char *blob, int parent,
		struct qep_platform_info *pinfo)
{
	int node;
	long val;

	if (!pinfo || !blob) {
		qep_fdt_err("%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	parent = fdt_subnode_offset(blob, parent, "qap_ip_blocks");
	node = fdt_subnode_offset(blob, parent, "cmac_usplus_0");

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_RSFEC_EN, QEP_BASE_16);
	if (val == -EINVAL)
		return -EINVAL;

	pinfo->rsfec_en = val;

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_PTP_1588, QEP_BASE_10);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->ptp_1588 = val;

	return 0;
}

int qep_fdt_get_platform_prop(char *blob, struct qep_platform_info *pinfo)
{
	long val;
	int node, parent;
	const char *str;

	if (!pinfo || !blob) {
		qep_fdt_err("%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	node = fdt_subnode_offset(blob, 0, "platform_info");

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_MGMT_PF, QEP_BASE_16);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->mgmt_pf = val;


	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_NET_PF, QEP_BASE_16);
	if (val == -EINVAL)
		return -EINVAL;
	pinfo->net_pf = val;

	str = qep_fdt_get_prop_str(blob, node, QEP_NAME_IP_PROP,
				QEP_FDT_PROP_DESIGN);
	if (!str)
		return -EINVAL;
	pinfo->design = qep_fdt_find_index(QEP_NAME_DESIGN, str);


	parent = fdt_subnode_offset(blob, 0, "addressable_endpoints");

	val = qep_fdt_get_prop_qdma(blob, parent, pinfo);
	if (val == -EINVAL)
		return -EINVAL;

	val = qep_fdt_get_prop_stmn(blob, parent, pinfo);
	if (val == -EINVAL)
		return -EINVAL;

	val = qep_fdt_get_prop_cmac(blob, parent, pinfo);
	if (val == -EINVAL)
		return -EINVAL;

	//TODO GET Board and QSFP INFO

	return 0;
}

static struct qep_ip_info *_get_ip_info_struct(char *blob, int node,
		struct qep_platform_info *pinfo)
{
	int len, idx;
	uint32_t pf;
	long val;
	const char *name = NULL;
	struct qep_ip_info *ip_info;

	if (!pinfo || !blob || node < 0) {
		qep_fdt_err("%s: Invalid args\n", __func__);
		return NULL;
	}

	pf = pinfo->net_pf;

	name = fdt_get_name(blob, node, &len);
	if (!name) {
		qep_fdt_err("%s: %s: name not found\n", __func__, name);
		return NULL;
	}

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_PF, QEP_BASE_16);
	if (val != -EINVAL)
		pf = val;

	if ((pf != pinfo->net_pf) && (pf != pinfo->mgmt_pf)) {
		qep_fdt_err("%s: Invalid pf of ip block %s\n", __func__, name);
		return NULL;
	}

	if (pf == pinfo->net_pf)
		idx = qep_fdt_find_index(QEP_NAME_NET_IP, name);
	else
		idx = qep_fdt_find_index(QEP_NAME_MGMT_IP, name);
	if (idx < 0) {
		qep_fdt_err("%s: %s: pf:%d unknown ip\n", __func__, name, pf);
		return NULL;
	}

	if (pf == pinfo->net_pf)
		ip_info = &pinfo->net_ip[idx];
	else
		ip_info = &pinfo->mgmt_ip[idx];

	return ip_info;
}



int qep_fdt_parse_ip_prop(char *blob, int node, struct qep_platform_info *pinfo)
{
	int len;
	long val;
	const char *data;
	const char *name;
	struct qep_ip_info *ip_info;
	uint32_t bar;

	if (!pinfo || !blob || node < 0) {
		qep_fdt_err("%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	name = fdt_get_name(blob, node, &len);
	if (!name)
		return -EINVAL;

	/*qap_ip_blocks is wrapper block no need to parse*/
	if (strcmp("qap_ip_blocks", name) == 0)
		return 0;

	ip_info = _get_ip_info_struct(blob, node, pinfo);
	if (!ip_info)
		return -EINVAL;

	/*bar is not necessary for all block of netpf and user bar*/
	bar = pinfo->user_bar;
	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_BAR, QEP_BASE_16);
	if (val != -EINVAL)
		bar = val;
	ip_info->bar = bar;

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_OFFSET, QEP_BASE_16);
	if (val != -EINVAL)
		ip_info->offset = val;

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_RANGE, QEP_BASE_16);
	if (val != -EINVAL)
		ip_info->range = val;

	data = fdt_getprop(blob, node, "version", NULL);
	if (data)
		memcpy(ip_info->version, data, strlen(data));

	val = qep_fdt_get_prop_long(blob, node, QEP_NAME_IP_PROP,
			QEP_FDT_PROP_EN, QEP_BASE_10);
	if (val != -EINVAL)
		ip_info->enable = val;

	return 0;
}

void qep_fdt_parse_ip_node(char *blob, int parent,
		struct qep_platform_info *pinfo)
{
	int node;
	int depth;
	int ret;

	if (!pinfo || !blob) {
		qep_fdt_err("%s: Invalid args\n", __func__);
		return;
	}

	fdt_for_each_subnode(node, blob, parent) {
		depth = fdt_node_depth(blob, node);

		if (depth > 0)
			qep_fdt_parse_ip_node(blob, node, pinfo);

		ret = qep_fdt_parse_ip_prop(blob, node, pinfo);
		if (ret) {
			qep_fdt_err("%s: %s: partially parsed\n", __func__,
					fdt_get_name(blob, node, NULL));
		}
	}
}

static void _add_qap_top_offset(struct qep_platform_info *pinfo)
{
	int i;
	int qap_top = pinfo->net_ip[QEP_NETPF_QAP_TOP].offset;

	for (i = QEP_NETPF_QAP_BASE; i < QEP_NETPF_MAX_BLOCKS; i++)
		pinfo->net_ip[i].offset += qap_top;

}

static void _update_qap_bar_info(struct qep_platform_info *pinfo)
{
	int i;

	for (i = QEP_NETPF_QAP_BASE; i < QEP_NETPF_MAX_BLOCKS; i++)
		pinfo->net_ip[i].bar = pinfo->user_bar;

}

int qep_get_platform_info(char *pltfm_identifier, int ident_len,
		struct qep_platform_info *pinfo)
{
	char *blob = NULL;
	unsigned long len;
	int node;
	int ret;
	enum qep_design_type type;

	if (!pinfo || !pltfm_identifier || ident_len < 0) {
		qep_fdt_err("%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	//TODO identifier to path mapping logic and firmware file location

	type = pinfo->design;

	ret  = qep_fdt_read_metadata(pltfm_identifier, &blob, &len);
	if (ret) {
		qep_fdt_warn("%s: qep_fdt_read_metadata failed Err:%d\n",
				__func__, ret);
		goto return_default;
	}

	ret = qep_fdt_validate_schema(blob);
	if (ret) {
		qep_fdt_err("%s: qep_fdt_validate_schema failed, Err:%d\n",
				__func__, ret);
		goto return_default;
	}

	ret = qep_fdt_get_platform_prop(blob, pinfo);
	if (ret) {
		qep_fdt_err("%s: qep_fdt_get_platform_prop failed, Err:%d\n",
				__func__, ret);
		goto return_default;
	}

	if (pinfo->design != type) {
		qep_fdt_err("%s: QEP Design Type Mismatch, type:%d pinfo:%d\n",
				__func__, type, pinfo->design);
		goto return_default;
	}

	node = fdt_subnode_offset(blob, 0, "addressable_endpoints");
	qep_fdt_parse_ip_node(blob, node, pinfo);

	_add_qap_top_offset(pinfo);

	_qep_fdt_free(blob);

	return 0;

return_default:
	if (blob)
		_qep_fdt_free(blob);

	qep_fdt_info("%s: Loading default platform configuration\n", __func__);
	ret = qep_get_platform_default(type, pinfo);
	if (ret)
		qep_fdt_err("%s: qep_get_platform_default failed Err:%d\n",
			__func__, ret);
	return ret;
}

int qep_get_platform_default(enum qep_design_type type,
		struct qep_platform_info *pinfo)
{
	memcpy(pinfo, &default_pinfo, sizeof(struct qep_platform_info));

	if (type > QEP_DESIGN_MAX) {
		qep_fdt_err("%s: Unknown design type: %d\n", __func__, type);
		return -EINVAL;
	}

	if (type == QEP_DESIGN_QAP) {
		pinfo->c2h_bypass = 0;
		pinfo->h2c_bypass = 0;
		pinfo->net_ip[QEP_NETPF_STMN].enable = 0;
	}

	if (type == QEP_DESIGN_QAP || type == QEP_DESIGN_QAP_STMN) {
		pinfo->qdma_bar = 0;
		pinfo->user_bar = 1;
		pinfo->stmn_queue_base = 256;
		pinfo->pci_msix_vec_count = 8;
		pinfo->net_ip[QEP_NETPF_QAP_TOP].enable = 0;
		pinfo->net_ip[QEP_NETPF_QAP_TOP].offset = 0;
		pinfo->net_ip[QEP_NETPF_RSS].enable = 0;
		pinfo->net_ip[QEP_NETPF_CHECKSUM].enable = 0;
		pinfo->net_ip[QEP_NETPF_PLATFORM_INFO].enable = 0;
		pinfo->net_ip[QEP_NETPF_MAILBOX].enable = 0;
		pinfo->net_ip[QEP_NETPF_SEC_RX].enable = 0;
		_update_qap_bar_info(pinfo);
	}

	_add_qap_top_offset(pinfo);

	return 0;
}

void qep_print_platform_info(struct qep_platform_info *pinfo, char *buf,
		int len)
{
	int i;

	if (!pinfo)
		return;
	snprintf(buf + strlen(buf), len - strlen(buf),
		"design:%s\n",
		qep_fdt_get_name(QEP_NAME_DESIGN, pinfo->design));
	snprintf(buf + strlen(buf), len - strlen(buf),
		"mgmtpf:%d\n", pinfo->mgmt_pf);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"net_pf:%d\n", pinfo->net_pf);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"qdma_bar:%d\n", pinfo->qdma_bar);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"user_bar:%d\n", pinfo->user_bar);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"c2h_bypass:%d\n", pinfo->c2h_bypass);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"h2c_bypass:%d\n", pinfo->h2c_bypass);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"master_pf:%d\n", pinfo->master_pf);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"poll mode:%d\n", pinfo->poll_mode);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"stmn_queue_base:%d\n", pinfo->stmn_queue_base);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"stmn_queue_max:%d\n", pinfo->stmn_queue_max);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"pci_msix_vec_count:%d\n", pinfo->pci_msix_vec_count);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"qsfp_port:%d\n", pinfo->qsfp_port);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"bandwith:%d\n", pinfo->bandwith);
	snprintf(buf + strlen(buf), len - strlen(buf),
		"rsfec:%d\n", pinfo->rsfec_en);
	snprintf(buf + strlen(buf), len - strlen(buf),
			"ptp_1588:%d\n", pinfo->ptp_1588);

	for (i = 0; i < QEP_NETPF_MAX_BLOCKS; i++) {
		snprintf(buf + strlen(buf), len - strlen(buf),
			"%-2d: %-20s: { %d, %d, 0x%x, 0x%x, \"%s\" },\n",
			i, qep_netpf_ip_names[i],
			pinfo->net_ip[i].enable,
			pinfo->net_ip[i].bar,
			pinfo->net_ip[i].offset,
			pinfo->net_ip[i].range,
			pinfo->net_ip[i].version);
	}

	for (i = 0; i < QEP_MGMTPF_MAX_BLOCKS; i++) {
		snprintf(buf + strlen(buf), len - strlen(buf),
			"%-2d: %-20s: { %d, %d, 0x%x, 0x%x, \"%s\" },\n",
			i, qep_mgmtpf_ip_names[i],
			pinfo->mgmt_ip[i].enable,
			pinfo->mgmt_ip[i].bar,
			pinfo->mgmt_ip[i].offset,
			pinfo->mgmt_ip[i].range,
			pinfo->mgmt_ip[i].version);
	}

}

void qep_print_platform_info1(struct qep_platform_info *pinfo)
{
	char *buf;

	buf = _qep_fdt_alloc(QEP_PINFO_PRINT_BUF_LEN);
	if (!buf) {
		qep_fdt_err("%s: mem aloc failed", __func__);
		return;
	}

	qep_print_platform_info(pinfo, buf, QEP_PINFO_PRINT_BUF_LEN);

	qep_fdt_info(" %s", buf);

	_qep_fdt_free(buf);

}

void qep_fdt_traverse_node(char *blob, int parent)
{
	int node = 0;

	fdt_for_each_subnode(node, blob, parent) {
		int property;
		const char *name;
		int len;
		int depth;

		depth = fdt_node_depth(blob, node);
		qep_fdt_info("node: %d %s %d\n",
				node, fdt_get_name(blob, node, NULL), depth);

		if (depth > 0)
			qep_fdt_traverse_node(blob, node);

		fdt_for_each_property_offset(property, blob, node) {
			fdt_getprop_by_offset(blob, property, &name, &len);
			qep_fdt_info(" %s ", name);
		}
		qep_fdt_info("\n");
	}
}

static int _cmp_ip_info(struct qep_ip_info *p1, struct qep_ip_info *p2)
{
	if (p1->enable != p2->enable) {
		qep_fdt_err("enable mismatch\n");
		return -EINVAL;
	}
	if (p1->bar != p2->bar) {
		qep_fdt_err("bar mismatch\n");
		return -EINVAL;

	}
	if (p1->offset != p2->offset) {
		qep_fdt_err("offset mismatch\n");
		return -EINVAL;

	}
	if (p1->range != p2->range) {
		qep_fdt_err("range mismatch\n");
		return -EINVAL;

	}
	if (strcmp(p1->version, p2->version) != 0) {
		qep_fdt_err("version mismatch\n");
		return -EINVAL;

	}
	return 0;
}
int qep_compare_platform_info(struct qep_platform_info *p1,
		struct qep_platform_info *p2)
{
	int i = 0;
	int ret;

	if (p1->mgmt_pf != p2->mgmt_pf) {
		qep_fdt_err("mgmt_pf mismatch\n");
		return -EINVAL;
	}
	if (p1->net_pf != p2->net_pf) {
		qep_fdt_err("net_pf mismatch\n");
		return -EINVAL;
	}
	if (p1->qdma_bar != p2->qdma_bar) {
		qep_fdt_err("qdma_bar mismatch\n");
		return -EINVAL;
	}
	if (p1->user_bar != p2->user_bar) {
		qep_fdt_err("user_bar mismatch\n");
		return -EINVAL;
	}
	if (p1->c2h_bypass != p2->c2h_bypass) {
		qep_fdt_err("c2h_bypass mismatch\n");
		return -EINVAL;
	}
	if (p1->h2c_bypass != p2->h2c_bypass) {
		qep_fdt_err("h2c_bypass mismatch\n");
		return -EINVAL;
	}
	if (p1->master_pf != p2->master_pf) {
		qep_fdt_err("master_pf mismatch\n");
		return -EINVAL;
	}
	if (p1->poll_mode != p2->poll_mode) {
		qep_fdt_err("master_pf mismatch\n");
		return -EINVAL;
	}
	if (p1->stmn_queue_base != p2->stmn_queue_base) {
		qep_fdt_err("stmn_queue_base mismatch\n");
		return -EINVAL;
	}
	if (p1->stmn_queue_max != p2->stmn_queue_max) {
		qep_fdt_err("stmn_queue_max mismatch\n");
		return -EINVAL;
	}
	if (p1->pci_msix_vec_count != p2->pci_msix_vec_count) {
		qep_fdt_err("pci_msix_vec_count mismatch\n");
		return -EINVAL;
	}
	if (p1->qsfp_port != p2->qsfp_port) {
		qep_fdt_err("qsfp_port mismatch\n");
		return -EINVAL;
	}

	/*if (p1->bandwith != p2->bandwith) {
	 *	qep_fdt_err("bandwith mismatch\n");
	 *	return -EINVAL;
	 *}
	 */
	if (p1->rsfec_en != p2->rsfec_en) {
		qep_fdt_err("rsfec_en mismatch\n");
		return -EINVAL;
	}
	if (p1->ptp_1588 != p2->ptp_1588) {
		qep_fdt_err("ptp_1588 mismatch\n");
		return -EINVAL;
	}

	for (i = 0; i < QEP_NETPF_MAX_BLOCKS; i++) {
		ret = _cmp_ip_info(&p1->net_ip[i], &p2->net_ip[i]);
		if (ret) {
			qep_fdt_err(" %sIP block mismatch\n",
				qep_fdt_get_name(QEP_NAME_NET_IP, i));
			return -EINVAL;
		}
	}

	for (i = 0; i < QEP_MGMTPF_MAX_BLOCKS; i++) {
		ret = _cmp_ip_info(&p1->mgmt_ip[i], &p2->mgmt_ip[i]);
		if (ret) {
			qep_fdt_err(" %sIP block mismatch\n",
				qep_fdt_get_name(QEP_NAME_MGMT_IP, i));
			return -EINVAL;
		}
	}
	return 0;
}

