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
 * @note   None.
 ******************************************************************************/
int rte_pmd_qep_get_bar_details(int portid, int32_t *config_bar_idx,
			int32_t *user_bar_idx, int32_t *bypass_bar_idx)
{
	return rte_pmd_qdma_get_bar_details(portid, config_bar_idx,
				user_bar_idx, bypass_bar_idx);
}

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
int rte_pmd_qep_get_queue_base(int portid, uint32_t *queue_base)
{
	return rte_pmd_qdma_get_queue_base(portid, queue_base);
}

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
int rte_pmd_qep_dbg_regdump(uint8_t portid)
{
	return rte_pmd_qdma_dbg_regdump(portid);
}

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
int rte_pmd_qep_dbg_qinfo(uint8_t portid, uint16_t queue)
{
	return rte_pmd_qdma_dbg_qinfo(portid, queue);
}

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
			  enum rte_pmd_qep_desc_type type)
{
	enum rte_pmd_qdma_xdebug_desc_type desc_type;
	int ret;

	ret = qdma_desc_type(type, &desc_type);
	if (ret)
		return -EINVAL;

	return rte_pmd_qdma_dbg_qdesc(portid, queue, start, end, desc_type);
}

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
int rte_pmd_qep_dbg_stmninfo(uint8_t portid)
{
	struct rte_eth_dev *dev;
	struct qdma_pci_dev *dma_priv;
	char msg[STMN_MSG_BUF_LEN_MAX];

	RTE_ETH_VALID_PORTID_OR_ERR_RET(portid, -ENODEV);
	dev = &rte_eth_devices[portid];
	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	memset(msg, 0, STMN_MSG_BUF_LEN_MAX);
	stmn_print_debug(dma_priv, msg, STMN_MSG_BUF_LEN_MAX);
	rte_log(RTE_LOG_INFO, RTE_LOGTYPE_USER1, "%s", msg);

	return 0;
}

/******************************************************************************/
/**
 * Function Name:	rte_pmd_qep_dbg_lbusinfo
 * Description:		Dumps the LBUS status and statistics
 *
 * @param	portid : Port ID
 *
 * @return	'0' on success and "< 0" on failure.
 *
 * @note	None.
 ******************************************************************************/
int rte_pmd_qep_dbg_lbusinfo(uint8_t portid)
{
	struct rte_eth_dev *dev;
	struct qdma_pci_dev *dma_priv;
	char msg[qep_lbus_get_buf_len()];

	RTE_ETH_VALID_PORTID_OR_ERR_RET(portid, -ENODEV);
	dev = &rte_eth_devices[portid];
	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	memset(msg, 0, qep_lbus_get_buf_len());
	qep_lbus_snprintf(dma_priv->bar_addr[dma_priv->user_bar_idx], msg,
		qep_lbus_get_buf_len());
	rte_log(RTE_LOG_INFO, RTE_LOGTYPE_USER1, "%s", msg);

	return 0;
}

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
			uint32_t addr, uint32_t *val)
{
	struct rte_eth_dev *dev = &rte_eth_devices[portid];
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	uint64_t baseaddr;

	if (portid < 0 || portid >= rte_eth_dev_count_avail()) {
		PMD_DRV_LOG(ERR, "Wrong port id %d\n", portid);
		return -ENOTSUP;
	}

	if (val == NULL) {
		PMD_DRV_LOG(ERR, "Caught NULL pointer for val\n");
		return -EINVAL;
	}

	if (!is_dev_qep_supported(dev)) {
		PMD_DRV_LOG(ERR, "Device is not supported\n");
		return -ENOTSUP;
	}

	if (bar >= (QDMA_NUM_BARS - 1)) {
		PMD_DRV_LOG(ERR, "Invalid PCI BAR number:%d\n", bar);
		return -ENOTSUP;
	}

	baseaddr = (uint64_t)qdma_dev->bar_addr[bar];
	if (!baseaddr) {
		PMD_DRV_LOG(ERR, "PCI BAR number %d not mapped\n", bar);
		return -ENOTSUP;
	}

	*val = *((volatile uint32_t *)(baseaddr + addr));
	return 0;
}

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
			uint32_t addr, uint32_t val)
{
	struct rte_eth_dev *dev = &rte_eth_devices[portid];
	struct qdma_pci_dev *qdma_dev = dev->data->dev_private;
	uint64_t baseaddr;

	if (portid < 0 || portid >= rte_eth_dev_count_avail()) {
		PMD_DRV_LOG(ERR, "Wrong port id %d\n", portid);
		return -ENOTSUP;
	}

	if (!is_dev_qep_supported(dev)) {
		PMD_DRV_LOG(ERR, "Device is not supported\n");
		return -ENOTSUP;
	}

	if (bar >= (QDMA_NUM_BARS - 1)) {
		PMD_DRV_LOG(ERR, "Invalid PCI BAR number:%d\n", bar);
		return -ENOTSUP;
	}

	baseaddr = (uint64_t)qdma_dev->bar_addr[bar];
	if (!baseaddr) {
		PMD_DRV_LOG(ERR, "PCI BAR number %d not mapped\n", bar);
		return -ENOTSUP;
	}

	*((volatile uint32_t *)(baseaddr + addr)) = val;
	return 0;
}
