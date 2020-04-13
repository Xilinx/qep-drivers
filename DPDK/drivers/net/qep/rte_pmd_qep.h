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

/*Functions*/
/******************************************************************************/
/**
 * Function Name:	rte_pmd_qep_get_bar_details
 * Description:		Returns the BAR indices of the QDMA BARs
 *
 * @param	portid : Port ID
 * @param	config_bar_idx : Config BAR index
 * @param	user_bar_idx   : User BAR index
 * @param	bypass_bar_idx : Bypass BAR index
 *
 * @return	'0' on success and '< 0' on failure.
 *
 * @note	None.
 ******************************************************************************/
int rte_pmd_qep_get_bar_details(int portid, int32_t *config_bar_idx,
			int32_t *user_bar_idx, int32_t *bypass_bar_idx);

/******************************************************************************/
/**
 * Function Name:	rte_pmd_qep_get_queue_base
 * Description:		Returns queue base for given port
 *
 * @param	portid : Port ID.
 * @param	queue_base : queue base.
 *
 * @return	'0' on success and '< 0' on failure.
 *
 * @note    Application can call this API only after successful
 *          call to rte_eh_dev_configure() API.
 ******************************************************************************/
int rte_pmd_qep_get_queue_base(int portid, uint32_t *queue_base);

/******************************************************************************/
/**
 * Function Name:	rte_pmd_qep_dbg_regdump
 * Description:		Dumps the QDMA configuration registers
 *			for the given port.
 *
 * @param	portid : Port ID
 *
 * @return	'0' on success and "< 0" on failure.
 *
 * @note	None.
 ******************************************************************************/
int rte_pmd_qep_dbg_regdump(uint8_t portid);

/******************************************************************************/
/**
 * Function Name:	rte_pmd_qep_dbg_qinfo
 * Description:		Dumps the queue contexts and queue specific SW
 *			structures for the given queue ID.
 *
 * @param	portid : Port ID
 * @param	queue  : Queue ID relative to the Port
 *
 * @return	'0' on success and "< 0" on failure.
 *
 * @note	None.
 ******************************************************************************/
int rte_pmd_qep_dbg_qinfo(uint8_t portid, uint16_t queue);

/******************************************************************************/
/**
 * Function Name:	rte_pmd_qep_dbg_qdesc
 * Description:		Dumps the Queue descriptors.
 *
 * @param	portid : Port ID
 * @param	queue  : Queue ID relative to the Port
 * @param	start  : start index of the descriptor to dump
 * @param	end    : end index of the descriptor to dump
 * @param	type   : Descriptor type
 *
 * @return	'0' on success and "< 0" on failure.
 *
 * @note	None.
 ******************************************************************************/
int rte_pmd_qep_dbg_qdesc(uint8_t portid, uint16_t queue, int start, int end,
			  enum rte_pmd_qep_desc_type type);

/******************************************************************************/
/**
 * Function Name:	rte_pmd_qep_dbg_stmninfo
 * Description:		Dumps the STM-N status and statistics
 *
 * @param	portid : Port ID
 *
 * @return	'0' on success and "< 0" on failure.
 *
 * @note	None.
 ******************************************************************************/
int rte_pmd_qep_dbg_stmninfo(uint8_t portid);

/******************************************************************************/
/**
 * Function Name:	rte_pmd_qep_dbg_lbusinfo
 * Description:		Dumps the STM-N status and statistics
 *
 * @param	portid : Port ID
 *
 * @return	'0' on success and "< 0" on failure.
 *
 * @note	None.
 ******************************************************************************/
int rte_pmd_qep_dbg_lbusinfo(uint8_t portid);

/******************************************************************************/
/**
 * Function Name:	rte_pmd_qep_addr_read
 * Description:		Returns the value at the given BAR offset
 *
 * @param	portid : Port ID.
 * @param	bar : PCIe BAR number.
 * @param	addr : BAR offset to read.
 * @param	val : Pointer holding the read value.
 *
 * @return	'0' on success and '< 0' on failure.
 ******************************************************************************/
int rte_pmd_qep_addr_read(int portid, uint32_t bar,
			uint32_t addr, uint32_t *val);

/******************************************************************************/
/**
 * Function Name:	rte_pmd_qep_addr_write
 * Description:		Writes the value at the given BAR offset
 *
 * @param	portid : Port ID.
 * @param	bar : PCIe BAR number.
 * @param	addr : BAR offset to write.
 * @param	val : Value to be written.
 *
 * @return	'0' on success and '< 0' on failure.
 ******************************************************************************/
int rte_pmd_qep_addr_write(int portid, uint32_t bar,
			uint32_t addr, uint32_t val);

#endif /* ifndef __RTE_PMD_QEP_EXPORT_H__ */
