/*-
 * BSD LICENSE
 *
 * Copyright(c) 2019 Xilinx, Inc. All rights reserved.
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

#include "qdma_access.h"
#include "rte_pmd_qep.h"
#include "rte_pmd_qdma.h"
#include "qdma.h"


static int qdma_desc_type(enum rte_pmd_qep_desc_type type,
		   enum rte_pmd_qdma_xdebug_desc_type *qtype)
{
	int ret = 0;

	switch (type) {
	case RTE_PMD_QEP_DESC_C2H:
		*qtype = RTE_PMD_QDMA_XDEBUG_DESC_C2H;
		break;
	case RTE_PMD_QEP_DESC_H2C:
		*qtype = RTE_PMD_QDMA_XDEBUG_DESC_H2C;
		break;
	case RTE_PMD_QEP_DESC_CMPT:
		*qtype = RTE_PMD_QDMA_XDEBUG_DESC_CMPT;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int rte_pmd_qep_dbg_regdump(uint8_t port_id)
{
	return rte_pmd_qdma_xdebug(port_id,
				   RTE_PMD_QDMA_XDEBUG_QDMA_GLOBAL_CSR,
				   NULL);
}

int rte_pmd_qep_dbg_qinfo(uint8_t port_id, uint16_t queue)
{
	return rte_pmd_qdma_xdebug(port_id, RTE_PMD_QDMA_XDEBUG_QUEUE_CONTEXT,
				   &queue) ?: rte_pmd_qdma_xdebug(port_id,
				RTE_PMD_QDMA_XDEBUG_QUEUE_STRUCT, &queue);
}

int rte_pmd_qep_dbg_qdesc(uint8_t port_id, uint16_t queue, int start, int end,
			  enum rte_pmd_qep_desc_type type)
{
	struct rte_pmd_qdma_xdebug_desc_param param;
	int ret = -EINVAL;

	param.queue = queue;
	param.start = start;
	param.end = end;
	ret = qdma_desc_type(type, &param.type);
	if (ret)
		return -EINVAL;

	return rte_pmd_qdma_xdebug(port_id, RTE_PMD_QDMA_XDEBUG_QUEUE_DESC_DUMP,
				   &param);
}

int rte_pmd_qep_dbg_stmninfo(uint8_t port_id)
{
	return rte_pmd_qdma_xdebug(port_id, RTE_PMD_QDMA_XDEBUG_STMN, NULL);

}

