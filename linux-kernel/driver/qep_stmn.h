/*
 * Copyright (c) 2019 Xilinx, Inc.
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

#ifndef QEP_STMN_H
#define QEP_STMN_H

#include <linux/types.h>
#include "qdma_ul_ext.h"

/**
 * @union - c2h_metadata
 * @brief	STMN Completion entry metadata
 */
union c2h_metadata {
	__be64 user:40;
	struct {
		u32 hw_l4_chksum_bad:1;
		u32 hw_l4_chksum_valid:1;
		u32 hw_l3_chksum_bad:1;
		u32 hw_l3_chksum_valid:1;
		u32 l4_hdr_dw:4;
		u32 l4_offset:6;
		u32 l3_hdr_dw:4;
		u32 l3_offset:6;
		u32 is_ipfrag:1;
		u32 is_udp:1;
		u32 is_tcp:1;
		u32 is_ipv6:1;
		u32 is_ipv4:1;
		u32 is_vlan:1;
		u32 reserved:10;
	};
};

/**
 * @struct - smtn_cmpt_entry
 * @brief	STMN Completion entry
 */
struct stmn_cmpt_entry {
	__be32 format:1;
	__be32 color:1;
	__be32 err:1;
	__be32 desc_used:1;
	__be32 len:16;
	__be32 rsvd:3;
	__be32 usr_err:1;
	__be64 metadata:40;
};

/**
 * @union - h2c_metadata
 * @brief	h2c descriptor metadata
 */
union h2c_metadata {
	__be32 user;
	struct {
		u32 l3_hdr_off:7;
		u32 l3_cksum_gen:1;
		u32 l4_hdr_off:7;
		u32 l4_cksum_gen:1;
		u32 l3_len:2;
		u32 l4_len:4;
		u32 udp:1;
		u32 rsvd:9;
	};
};

/**
 * @struct - qdma_stmn_h2c_desc
 * @brief	h2c descriptor format
 */
struct qdma_stmn_h2c_desc {
	__be32 metadata;
	__be32 len:16;		/**< total packet length */
	__be32 flags:16;	/**< descriptor flags */
	__be64 src_addr;	/**< source address */
};


int qep_stmn_bypass_desc_fill(void *q_hndl, enum qdma_q_mode q_mode,
			       enum qdma_q_dir q_dir, struct qdma_request *req);

int qep_stmn_parse_cmpl_entry(void *cmpl_entry,
			       struct qdma_ul_cmpt_info *cmpl);

#endif /* QEP_STMN_H */
