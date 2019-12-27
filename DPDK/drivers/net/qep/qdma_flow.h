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

#ifndef __QDMA_FLOW_H__
#define __QDMA_FLOW_H__

extern const struct rte_flow_ops qep_flow_ops;

#define DMA_C2H_BASE_ADDR                0x00050000
#define DMA_CLCR_BASE_OFFSET             0x00000010
#define DMA_CLCR2_OFFSET		(0x00000014)
#define DMA_CLCR_QCTRL_MASK              (0x3000)
#define DMA_CLCR_QCTRL_BIT               12
#define DMA_CLCR_QNUM_MASK               0x000007FF
#define QEP_RX_MODE_RR 1
#define QEP_RX_MODE_TCAM 2
#define RX_START_QUEUE_RR 0

#define QEP_RSS_FLOW_TYPE ( \
		ETH_RSS_IPV4 | \
		ETH_RSS_NONFRAG_IPV4_TCP | \
		ETH_RSS_NONFRAG_IPV4_UDP | \
		ETH_RSS_IPV6 | \
		ETH_RSS_NONFRAG_IPV6_TCP | \
		ETH_RSS_NONFRAG_IPV6_UDP)
#define QEP_RSS_TBL_SIZE (256)
#define QEP_RSS_HKEY_SIZE (40)
#define QEP_RSS_TBL_ENT_MASK 0x7ff
struct qep_rss_info {
	uint32_t rss_tbl[QEP_RSS_TBL_SIZE];
};

struct rte_flow {
	uint32_t rss;
	struct qep_rss_info rssinfo;
	TAILQ_ENTRY(rte_flow) entries;	/* flow list entries */
};
TAILQ_HEAD(qep_flow_list, rte_flow);

void qep_set_rx_mode(struct rte_eth_dev *dev, uint32_t mode);


#endif /* ifndef __QDMA_FLOW_H__ */
