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

#ifndef QEP_FDT_H_
#define QEP_FDT_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <inttypes.h>
#endif
#include <stdbool.h>
#include "libfdt/libfdt.h"

#define QEP_JSON_SCHEMA_VERSION_MAJOR 0
#define QEP_JSON_SCHEMA_VERSION_MINOR 1

#define QEP_VER_STR_LEN_MAX 32
#define QEP_PINFO_PRINT_BUF_LEN 1536

#define QEP_BASE_10  10
#define QEP_BASE_16  16

enum qep_qsfp_port_bandwidth {
	QEP_10G,
	QEP_10_25G,
	QEP_100G,
	QEP_MAX_BANDWIDTH
};

enum qep_design_type {
	QEP_DESIGN_QEP,
	QEP_DESIGN_QAP,
	QEP_DESIGN_QAP_STMN,
	QEP_DESIGN_MAX
};
/* Index for board names NETPF IP blocks */
enum qep_board_type {
	QEP_BOARD_1525,
	QEP_BOARD_U50,
	QEP_BOARD_U200,
	QEP_BOARD_U250,
	QEP_BOARD_TYPE_MAX
};

/* Index for name string of NETPF IP blocks */
enum qep_netpf_blocks {
	QEP_NETPF_QDMA,
	QEP_NETPF_STMN,
	QEP_NETPF_RSS,
	QEP_NETPF_SEC_RX,
	QEP_NETPF_CHECKSUM,
	QEP_NETPF_PLATFORM_INFO,
	QEP_NETPF_MAILBOX,
	QEP_NETPF_QAP_TOP, /*Below IP offset are relative to top*/
	QEP_NETPF_QAP_BASE,
	QEP_NETPF_CMAC_CTL,
	QEP_NETPF_CMAC_IP,
	QEP_NETPF_LBUS_CONV,
	QEP_NETPF_QDMA_USER_H2C,
	QEP_NETPF_QDMA_USER_C2H,
	QEP_NETPF_GPIO_QSFP,
	QEP_NETPF_IIC_QSFP,
	QEP_NETPF_CMAC_DRP0,
	QEP_NETPF_CMAC_DRP1,
	QEP_NETPF_PCIE_DRP0,
	QEO_NETPF_PCIE_RQRC_MON,
	QEP_NETPF_MAX_BLOCKS,
};

/* Index for name string of MGMTPF IP blocks */
enum qep_mgmtpf_blocks {
	QEP_MGMTPF_SEC_BLOCK_TX,
	QEP_MGMTPF_SEC_BLOCK_RX,
	QEP_MGMTPF_CMAC_TOP,
	QEP_MGMTPF_QSFP,
	QEP_MGMTPF_MAX_BLOCKS
};

/* Index for name string of the properties of the FDT Nodes */
enum qep_fdt_prop {
	QEP_FDT_PROP_PF,
	QEP_FDT_PROP_BAR,
	QEP_FDT_PROP_EN,
	QEP_FDT_PROP_OFFSET,
	QEP_FDT_PROP_RANGE,
	QEP_FDT_PROP_VERSION,
	QEP_FDT_PROP_QMAX,
	QEP_FDT_PROP_QBASE,
	QEP_FDT_PROP_MSIX_VEC,
	QEP_FDT_PROP_C2H_BYPASS,
	QEP_FDT_PROP_H2C_BYPASS,
	QEP_FDT_PROP_MASTER_PF,
	QEP_FDT_PROP_POLL_MODE,
	QEP_FDT_PROP_RSFEC_EN,
	QEP_FDT_PROP_PTP_1588,
	QEP_FDT_PROP_QSFP_PORT,
	QEP_FDT_PROP_QSFP_BANDWITH,
	QEP_FDT_PROP_MGMT_PF,
	QEP_FDT_PROP_NET_PF,
	QEP_FDT_PROP_DESIGN,
	QEP_FDT_PROP_MAX
};

enum qep_fdt_name_type {
	QEP_NAME_NET_IP,
	QEP_NAME_MGMT_IP,
	QEP_NAME_IP_PROP,
	QEP_NAME_BOARD,
	QEP_NAME_DESIGN,
	QEP_NAME_TYPE_MAX
};

/* Structure to store generic info of a IP block*/
struct qep_ip_info {
	bool enable;
	uint8_t bar;
	uint32_t offset;
	uint32_t range;
	char version[QEP_VER_STR_LEN_MAX];
};

/* Structure to hold the platform info extracted from JSON */
struct qep_platform_info {
	uint8_t mgmt_pf;
	uint8_t net_pf;
	uint8_t qdma_bar;
	uint8_t user_bar;
	uint16_t stmn_queue_base;
	uint16_t stmn_queue_max;
	uint8_t pci_msix_vec_count;
	bool c2h_bypass;
	bool h2c_bypass;
	bool master_pf;
	bool poll_mode;
	bool rsfec_en;
	bool ptp_1588;
	uint8_t qsfp_port;
	enum qep_qsfp_port_bandwidth bandwith;
	enum qep_board_type board;
	enum qep_design_type design;
	struct qep_ip_info mgmt_ip[QEP_MGMTPF_MAX_BLOCKS];
	struct qep_ip_info net_ip[QEP_NETPF_MAX_BLOCKS];
};
/*****************************************************************************/
/*  This function provides platform info. It reads the firmware(json/dtb),
 *  parses into @info. Firmware name is identified using @pltform_identier
 */
int qep_get_platform_info(char *pltfm_identifier, int ident_len,
		struct qep_platform_info *pinfo);

/* This function provides default  platform info in case of no firmware*/
int qep_get_platform_default(enum qep_design_type type,
		struct qep_platform_info *pinfo);
/* Fill the Buffer with human readable platform info*/
void qep_print_platform_info(struct qep_platform_info *pinfo, char *buf,
		int len);
int qep_compare_platform_info(struct qep_platform_info *p1,
		struct qep_platform_info *p2);
void qep_print_platform_info1(struct qep_platform_info *pinfo);
/*****************************************************************************/
/*Utility Functions*/

const char *qep_fdt_get_name(enum qep_fdt_name_type type, unsigned int idx);
int qep_fdt_find_index(enum qep_fdt_name_type type, const char *name);

static inline uint32_t qep_fdt_get_netip_offset(struct qep_platform_info *pinfo,
		enum qep_netpf_blocks idx)
{
	return idx < QEP_NETPF_MAX_BLOCKS ? pinfo->net_ip[idx].offset : 0;
}

static inline bool qep_fdt_get_netip_enable(struct qep_platform_info *pinfo,
		enum qep_netpf_blocks idx)
{
	return idx < QEP_NETPF_MAX_BLOCKS && pinfo->net_ip[idx].enable ? 1 : 0;
}
/******************************************************************************/
/* Internal functions */
void qep_fdt_traverse_node(char *blob, int parent);
int qep_fdt_read_metadata(const char *fname, char **buf_ref,
		unsigned long *len);
int qep_fdt_validate_schema(char *blob);
void qep_fdt_parse_ip_node(char *blob, int parent,
		struct qep_platform_info *pinfo);
int qep_fdt_parse_ip_prop(char *blob, int node,
		struct qep_platform_info *pinfo);
int qep_fdt_get_platform_prop(char *blob, struct qep_platform_info *pinfo);
long qep_fdt_get_prop_stmn(char *blob, int parent,
		struct qep_platform_info *pinfo);
long qep_fdt_get_prop_qdma(char *blob, int parent,
		struct qep_platform_info *pinfo);
long qep_fdt_get_prop_cmac(char *blob, int parent,
		struct qep_platform_info *pinfo);
long qep_fdt_get_prop_long(char *blob, int node, enum qep_fdt_name_type type,
		unsigned int idx, int base);
const char *qep_fdt_get_prop_str(char *blob, int node,
		enum qep_fdt_name_type type,
		unsigned int idx);
/******************************************************************************/
#endif /* QEP_FDT_H_ */
