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

#ifndef QDMA_UL_EXT_STMN_H
#define QDMA_UL_EXT_STMN_H
#include "qdma_ul_ext.h"
#include <linux/types.h>


int qdma_stmn_bypass_desc_fill(void *q_hndl, enum qdma_q_mode q_mode,
			       enum qdma_q_dir q_dir, struct qdma_request *req);

int qdma_stmn_parse_cmpl_entry(void *cmpl_entry,
			       struct qdma_ul_cmpt_info *cmpl);

#endif /* QDMA_UL_EXT_STMN_H */
