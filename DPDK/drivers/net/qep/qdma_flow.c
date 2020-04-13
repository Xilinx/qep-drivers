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
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_flow.h>
#include <rte_flow_driver.h>
#include <rte_tailq.h>

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
	       DMA_C2H_BASE_ADDR + DMA_CLCR2_OFFSET, 0);
	qdma_write_reg((uint64_t)(dma_priv->bar_addr[dma_priv->user_bar_idx]) +
		       DMA_C2H_BASE_ADDR + DMA_CLCR_BASE_OFFSET, reg_val);

}

void qep_flow_init(struct qdma_pci_dev *dma_priv)
{
	TAILQ_INIT(&dma_priv->flow_list);
}

void qep_flow_uninit(struct qdma_pci_dev *dma_priv)
{
	struct rte_flow *flow;

	while ((flow = TAILQ_FIRST(&dma_priv->flow_list)) != NULL) {
		TAILQ_REMOVE(&dma_priv->flow_list, flow, entries);
		rte_free(flow);
	}
}

static int qep_flow_parse_rss(struct rte_eth_dev *dev,
			      const struct rte_flow_action actions[],
			      struct rte_flow *filter,
			      struct rte_flow_error *error)
{
	const struct rte_flow_action_rss *action_rss;
	struct qdma_pci_dev *dma_priv;
	struct rte_flow *flow_ptr = NULL;
	unsigned int nb_queues;
	uint32_t nrxq = 0;
	uint32_t i;

	action_rss = actions->conf;
	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;
	nb_queues = action_rss->queue_num;

	TAILQ_FOREACH(flow_ptr, &dma_priv->flow_list, entries) {
		if (flow_ptr->rss == 1) {
			rte_flow_error_set(error, -EINVAL,
					   RTE_FLOW_ERROR_TYPE_ACTION,
					   actions, "RSS rule already exist");
			return -rte_errno;
		}
	}

	if (action_rss->queue_num == 0) {
		rte_flow_error_set(error, -EINVAL,
				RTE_FLOW_ERROR_TYPE_ACTION, actions,
				"Number of queue 0");
		return -rte_errno;
	}
	if (action_rss->level) {
		rte_flow_error_set(error, -EINVAL,
				RTE_FLOW_ERROR_TYPE_ACTION, actions,
				"level should be 0");
		return -rte_errno;
	}
	nrxq = dev->data->nb_rx_queues;
	switch (action_rss->func) {
	case RTE_ETH_HASH_FUNCTION_DEFAULT:
	case RTE_ETH_HASH_FUNCTION_TOEPLITZ:
		break;
	default:
		rte_flow_error_set(error, -EINVAL,
					RTE_FLOW_ERROR_TYPE_ACTION, actions,
					"Hash Function not supported");
		return -rte_errno;
	}

	for (i = 0; i < action_rss->queue_num; ++i) {
		if (action_rss->queue[i] >= nrxq) {
			rte_flow_error_set(error, -EINVAL,
					RTE_FLOW_ERROR_TYPE_ACTION, actions,
					"Queue out of range");
			return -rte_errno;
		}
	}

	if ((action_rss->types &  QEP_RSS_FLOW_TYPE) != action_rss->types) {
		PMD_DRV_LOG(INFO, "%s expected rss type should be subset of"
		" %llx, received %lx\n", __func__,
		QEP_RSS_FLOW_TYPE, action_rss->types);
		rte_flow_error_set(error, -EINVAL,
					RTE_FLOW_ERROR_TYPE_ACTION, actions,
					"Hash Function not supported");
		return -rte_errno;
	}

	if (action_rss->key_len)
		PMD_DRV_LOG(INFO, "Ignored key received");

	for (i = 0; i < RTE_DIM(filter->rssinfo.rss_tbl); ++i)
		filter->rssinfo.rss_tbl[i] = action_rss->queue[i % nb_queues];

	filter->rss = 1;

	return 0;
}

static int
qep_flow_validate_action(struct rte_eth_dev *dev,
				const struct rte_flow_action actions[],
				struct rte_flow_error *error,
				struct rte_flow *filter)
{
	int rc;

	if (!actions) {
		rte_flow_error_set(error, EINVAL,
				   RTE_FLOW_ERROR_TYPE_ACTION_NUM,
				   NULL, "NULL action.");
		return -rte_errno;
	}

	for (; actions->type != RTE_FLOW_ACTION_TYPE_END; actions++) {
		switch (actions->type) {
		case RTE_FLOW_ACTION_TYPE_RSS:
			rc = qep_flow_parse_rss(dev, actions, filter, error);
			if (rc != 0)
				return rc;
			break;
		default:
			rte_flow_error_set(error, ENOTSUP,
					   RTE_FLOW_ERROR_TYPE_ACTION, actions,
					   "Action is not supported");
			return -rte_errno;
		}
	}

	return 0;
}

static int
qep_flow_validate_pattern(const struct rte_flow_item pattern[],
				 struct rte_flow_error *error,
				 struct rte_flow *filter __rte_unused)
{

	if (!pattern) {
		rte_flow_error_set(error, EINVAL,
				   RTE_FLOW_ERROR_TYPE_ITEM_NUM,
				   NULL, "NULL pattern.");
		return -rte_errno;
	}

	if (pattern->type != RTE_FLOW_ITEM_TYPE_END) {
		rte_flow_error_set(error, ENOTSUP,
				   RTE_FLOW_ERROR_TYPE_ITEM, pattern,
				   "Pattern Item not supported with RSS");
		return -rte_errno;
	}

	return 0;
}

/* Validate attributes */
static int
qep_flow_validate_attr(const struct rte_flow_attr *attr,
		       struct rte_flow_error *error)
{

	if (!attr) {
		rte_flow_error_set(error, EINVAL,
				   RTE_FLOW_ERROR_TYPE_ATTR,
				   NULL, "NULL attribute.");
		return -rte_errno;
	}

	/* Must be input direction */
	if (!attr->ingress) {
		rte_flow_error_set(error, EINVAL,
				   RTE_FLOW_ERROR_TYPE_ATTR_INGRESS,
				   attr, "Only support ingress.");
		return -rte_errno;
	}

	/* Not supported */
	if (attr->egress) {
		rte_flow_error_set(error, EINVAL,
				   RTE_FLOW_ERROR_TYPE_ATTR_EGRESS,
				   attr, "No support for egress.");
		return -rte_errno;
	}

	/* Not supported */
	if (attr->priority) {
		rte_flow_error_set(error, EINVAL,
				   RTE_FLOW_ERROR_TYPE_ATTR_PRIORITY,
				   attr, "No support for priority.");
		return -rte_errno;
	}

	/* Not supported */
	if (attr->group) {
		rte_flow_error_set(error, EINVAL,
				   RTE_FLOW_ERROR_TYPE_ATTR_GROUP,
				   attr, "No support for group.");
		return -rte_errno;
	}

	return 0;
}

static int
qep_validate_and_parse_flow(struct rte_eth_dev *dev,
			    const struct rte_flow_item pattern[],
			    const struct rte_flow_action actions[],
			    const struct rte_flow_attr *attr,
			    struct rte_flow_error *error,
			    struct rte_flow *filter)
{
	int rc = 0;

	rc = qep_flow_validate_attr(attr, error);
	if (rc != 0)
		return rc;
	rc = qep_flow_validate_pattern(pattern, error, filter);
	if (rc != 0)
		return rc;
	rc = qep_flow_validate_action(dev, actions, error, filter);
	if (rc != 0)
		return rc;

	return rc;
}

/**
 * DPDK callback to check whether a flow rule can be created on a given dev.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param attr
 *   Flow rule attributes.
 * @param pattern
 *   Pattern specification (list terminated by the END pattern item).
 * @param actions
 *   Associated actions (list terminated by the END action).
 * @param error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */

static int
qep_flow_validate(struct rte_eth_dev *dev,
		  const struct rte_flow_attr *attr,
		  const struct rte_flow_item pattern[],
		  const struct rte_flow_action actions[],
		  struct rte_flow_error *error)
{
	int ret = 0;
	struct rte_flow filter;

	memset(&filter, 0, sizeof(struct rte_flow));

	ret = qep_validate_and_parse_flow(dev, pattern, actions, attr,
					  error, &filter);

	return ret;
}

/**
 * DPDK callback to create flow on a given dev.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param attr
 *   Flow rule attributes.
 * @param pattern
 *   Pattern specification (list terminated by the END pattern item).
 * @param actions
 *   Associated actions (list terminated by the END action).
 * @param error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */

static struct rte_flow *
qep_flow_create(struct rte_eth_dev *dev,
		const struct rte_flow_attr *attr,
		const struct rte_flow_item pattern[],
		const struct rte_flow_action actions[],
		struct rte_flow_error *error)
{
	int ret = 0;
	struct rte_flow *flow = NULL;
	struct qdma_pci_dev *dma_priv;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;
	flow = rte_zmalloc("qep_rte_flow", sizeof(*flow), 0);
	if (flow == NULL) {
		rte_flow_error_set(error, ENOMEM,
				   RTE_FLOW_ERROR_TYPE_UNSPECIFIED, NULL,
				   "Failed to allocate memory");
		goto fail_no_mem;
	}

	ret = qep_validate_and_parse_flow(dev, pattern, actions, attr,
					  error, flow);

	if (ret != 0)
		goto fail;

	qep_set_reta(dev, flow->rssinfo.rss_tbl);
	TAILQ_INSERT_TAIL(&dma_priv->flow_list, flow, entries);

	return flow;
fail:
	rte_free(flow);
fail_no_mem:
	return NULL;
}

/**
 * DPDK callback to destroy a flow rule on a given dev.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param flow
 *    Device private flow rule handle to destroy.
 * @param error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */

static int
qep_flow_destroy(struct rte_eth_dev *dev,
		 struct rte_flow *flow,
		 struct rte_flow_error *error)
{
	struct rte_flow *flow_ptr = NULL;
	struct qdma_pci_dev *dma_priv;
	int rc = EINVAL;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	TAILQ_FOREACH(flow_ptr, &dma_priv->flow_list, entries) {
		if (flow_ptr == flow)
			rc = 0;
	}
	if (rc != 0) {
		rte_flow_error_set(error, rc,
				   RTE_FLOW_ERROR_TYPE_HANDLE, NULL,
				   "Failed to find flow rule to destroy");
		goto fail_bad_value;
	}
	qep_init_reta(dev, 0);
	TAILQ_REMOVE(&dma_priv->flow_list, flow, entries);
	rte_free(flow);

	return 0;
fail_bad_value:

	return -EINVAL;
}

/**
 * DPDK callback to destroy all flow rule on a given dev.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 */

static int
qep_flow_flush(struct rte_eth_dev *dev,
	       struct rte_flow_error *error __rte_unused)
{
	struct qdma_pci_dev *dma_priv;
	struct rte_flow *flow = NULL;

	dma_priv = (struct qdma_pci_dev *)dev->data->dev_private;

	while ((flow = TAILQ_FIRST(&dma_priv->flow_list)) != NULL) {
		qep_init_reta(dev, 0);
		TAILQ_REMOVE(&dma_priv->flow_list, flow, entries);
		rte_free(flow);
	}

	return 0;
}

const struct rte_flow_ops qep_flow_ops = {
	.validate = qep_flow_validate,
	.create = qep_flow_create,
	.destroy = qep_flow_destroy,
	.flush = qep_flow_flush,
};


