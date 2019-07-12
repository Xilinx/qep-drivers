/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2018-2019 Xilinx, Inc. All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <rte_ethdev.h>
#include "qdma.h"

void qep_set_rx_mode(struct rte_eth_dev *dev, uint32_t mode)
{
	struct qdma_pci_dev *dma_priv;
	uint32_t reg_val;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;
	reg_val = qdma_read_reg(
			(uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
				DMA_C2H_BASE_ADDR + DMA_CLCR_BASE_OFFSET);

	reg_val = reg_val & (~DMA_CLCR_QCTRL_MASK);
	/*0 -  Sending all traffic to single queue
	 *1 - Round robin
	 *2 - 2 for TCAM input
	 */
	reg_val |= (mode << DMA_CLCR_QCTRL_BIT);
	/* Setting q num to default zero*/
	reg_val &= (~DMA_CLCR_QNUM_MASK);

	/* Setting num queues for Round Robin Mode*/
	if (mode == QEP_RX_MODE_RR)
		reg_val |= ((dev->data->nb_rx_queues - 1) & DMA_CLCR_QNUM_MASK);

	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
	       DMA_C2H_BASE_ADDR + DMA_CLCR2_OFFSET, dma_priv->queue_base);
	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
		       DMA_C2H_BASE_ADDR + DMA_CLCR_BASE_OFFSET, reg_val);
}

