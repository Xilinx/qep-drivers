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

#ifndef __RTE_PMD_QEP_EXPORT_H__
#define __RTE_PMD_QEP_EXPORT_H__

/*Enums*/
enum rte_pmd_qep_desc_type {
	RTE_PMD_QEP_DESC_C2H,
	RTE_PMD_QEP_DESC_H2C,
	RTE_PMD_QEP_DESC_CMPT,
	RTE_PMD_QEP_DESC_MAX,
};

int rte_pmd_qep_dbg_regdump(uint8_t port_id);
int rte_pmd_qep_dbg_qinfo(uint8_t port_id, uint16_t queue);
int rte_pmd_qep_dbg_qdesc(uint8_t port_id, uint16_t queue, int start, int end,
			  enum rte_pmd_qep_desc_type type);
int rte_pmd_qep_dbg_stmninfo(uint8_t port_id);

#endif /* ifndef __RTE_PMD_QEP_EXPORT_H__ */
