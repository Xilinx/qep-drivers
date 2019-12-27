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
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#include <linux/debugfs.h>
#include <linux/pci.h>
#include "qep.h"
#include "stmn.h"

#define QDMA_DEV_NAME_SZ (32)
#define CMAC_MSG_BUF_LEN_MAX (8192)

struct dentry *qep_debugfs_root;

static int qep_cmac_get_hw_status(struct xcmac *mac, char *buf, u16 len)
{
	int status, i = 0;
	struct xcmac_rx_lane_am_status lane;
	struct xcmac_rx_fault_status fault;
	struct xcmac_rx_packet_status packet_status;
	struct xcmac_rx_vldemux_status vldemux_status;
	struct xcmac_tx_status tx_status;
	struct xcmac_bip7_config bip7_config;

	if (!mac || !buf)
		return -EINVAL;

	memset(&lane, 0, sizeof(lane));
	memset(&fault, 0, sizeof(fault));
	memset(&packet_status, 0, sizeof(packet_status));
	memset(&vldemux_status, 0, sizeof(vldemux_status));
	memset(&tx_status, 0, sizeof(tx_status));
	memset(&bip7_config, 0, sizeof(bip7_config));

	/* Get PCS lane status */
	status = xcmac_get_rx_lane_status(mac, &lane);
	if (status != 0)
		pr_err("Error in retrieving Rx lane status\n");

	/* Get Ethernet Packet status */
	status = xcmac_get_rx_packet_status(mac, &packet_status);
	if (status != 0)
		pr_err("Error in retrieving Rx packet status\n");

	/* Get Fault status */
	status = xcmac_get_rx_fault_status(mac, &fault);
	if (status != 0)
		pr_err("Error in retrieving Rx fault status\n");

	/* Get Tx status */
	status = xcmac_get_tx_status(mac, &tx_status);
	if (status != 0)
		pr_err("Error in retrieving Tx status\n");

	/* Get Virtual lane demux status */
	status = xcmac_get_rx_vldemux_status(mac, &vldemux_status);
	if (status != 0)
		pr_err("Error in retrieving Rx VlDemux status\n");

	status = xcmac_get_bip7_config(mac, &bip7_config);
	if (status != 0)
		pr_err("Error in retrieving Rx BIP7 Value\n");

	snprintf(buf + strlen(buf), len - strlen(buf),
		 "PCS Status=0x%x Aligned=0x%x Misaligned=0x%x Aligned error=0x%x\n",
			lane.pcs_status, lane.aligned,
				lane.mis_aligned, lane.aligned_error);

	snprintf(buf + strlen(buf), len - strlen(buf),
		 "HiBer=0x%x Bad Preamble=0x%x Bad Sfd=0x%x Got Signal OS=0x%x\n",
			packet_status.hi_ber,
			packet_status.bad_preamble, packet_status.bad_sfd,
			packet_status.got_signal_os);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"Remote Fault=0x%x Local Fault=0x%x Internal Local Fault=0x%x Received Local Fault=0x%x\n",
		fault.remote_fault,
		fault.local_fault, fault.internal_local_fault,
		fault.received_local_fault);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"Local Fault=0x%x PTP FIFO Read Error=0x%x PTP FIFO Write Error=0x%x\n",
		tx_status.local_fault, tx_status.ptp_fifo_read_error,
		tx_status.ptp_fifo_write_error);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"Rx BIP Override Value=0x%x Valid=0x%x\n",
		bip7_config.bip7_value, bip7_config.bip7_valid);


	snprintf(buf + strlen(buf), len - strlen(buf), "Lane:         ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf), " %2d ", i);

	snprintf(buf + strlen(buf), len - strlen(buf), "\nBlock Lock    ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf), "0x%x ",
			lane.block_lock[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nSynced Err    ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			"0x%x ", lane.synced_error[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nLM Err        ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			"0x%x ", lane.lane_marker_error[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nLM len Err    ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			"0x%x ",
			lane.lane_marker_len_error[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nLM Repeat Err ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			"0x%x ",
			lane.lane_marker_repeat_error[i]);


	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nVlDemuxed     ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf),
			len - strlen(buf), "0x%x ",
			vldemux_status.vldemuxed[i]);

	snprintf(buf + strlen(buf), len - strlen(buf),
		"\nVlNumber      ");
	for (i = 0; i < XCMAC_PCS_LANE_COUNT; i++)
		snprintf(buf + strlen(buf), len - strlen(buf),
			 "0x%x ", vldemux_status.vlnumber[i]);

	snprintf(buf + strlen(buf), len - strlen(buf), "\n");
		return status;
}
static int config_open(struct inode *inodep, struct file *filp)
{
	filp->private_data = inodep->i_private;
	return 0;
}

static ssize_t config_read(struct file *filp, char __user *buffer, size_t count,
			   loff_t *ppos)
{
	int ret = 0;
	char temp[512] = { 0 };
	struct qep_priv *xpriv;
	struct global_csr_conf l_global_csr_conf;

	if (!filp) {
		pr_err("invalid debugfs file ptr");
		return -EINVAL;
	}

	xpriv = (struct qep_priv *)(filp->private_data);
	if (!xpriv) {
		pr_err("invalid debugfs dev ptr");
		return -EINVAL;
	}

	if (!xpriv->netdev) {
		pr_err("invalid debugfs netdev ptr");
		return -EINVAL;
	}

	if (!buffer) {
		pr_err("invalid debugfs buffer ptr");
		return -EINVAL;
	}

	if (*ppos != 0)
		return 0;


	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &l_global_csr_conf);
	if (ret != 0)
		qep_warn(drv,
			 "%s: qdma_global_csr_get() failed with status %d\n",
			 __func__, ret);

	sprintf(temp,
		"rs_fec_en			= %d\n"
		"num_rx_queues			= %d\n"
		"num_tx_queues			= %d\n"
		"num_msix			= %d\n"
		"rx_desc_rng_sz_idx		= %d\n"
		"rx_desc_rng_sz			= %d\n"
		"tx_desc_rng_sz_idx		= %d\n"
		"tx_desc_rng_sz			= %d\n"
		"rx_buf_sz_idx			= %d\n"
		"rx_buf_sz			= %d\n"
		"cmpl_rng_sz_idx                = %d\n"
		"cmpl_rng_sz			= %d\n",
		xpriv->rs_fec_en, xpriv->netdev->real_num_rx_queues,
		xpriv->netdev->real_num_rx_queues,
		xpriv->qdma_dev_conf.msix_qvec_max, xpriv->rx_desc_rng_sz_idx,
		l_global_csr_conf.ring_sz[xpriv->rx_desc_rng_sz_idx],
		xpriv->tx_desc_rng_sz_idx,
		l_global_csr_conf.ring_sz[xpriv->tx_desc_rng_sz_idx],
		xpriv->rx_buf_sz_idx,
		l_global_csr_conf.c2h_buf_sz[xpriv->rx_buf_sz_idx],
		xpriv->cmpl_rng_sz_idx,
		l_global_csr_conf.ring_sz[xpriv->cmpl_rng_sz_idx]);

	if (*ppos >= strlen(temp))
		return 0;

	if (*ppos + count > strlen(temp))
		count = strlen(temp) - *ppos;

	if (copy_to_user(buffer, temp, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static int stmn_open(struct inode *inodep, struct file *filp)
{
	filp->private_data = inodep->i_private;
	return 0;
}

static ssize_t stmn_read(struct file *filp, char __user *buffer, size_t count,
			 loff_t *ppos)
{
	int ret;
	char *msg;
	u32 len;
	struct qep_priv *dev_hndl;

	if (!filp) {
		pr_err("invalid debugfs file ptr");
		return -EINVAL;
	}

	dev_hndl = (struct qep_priv *)(filp->private_data);
	if (!dev_hndl) {
		pr_err("invalid debugfs dev ptr");
		return -EINVAL;
	}

	if (!dev_hndl->netdev) {
		pr_err("invalid debugfs netdev ptr");
		return -EINVAL;
	}

	if (!buffer) {
		pr_err("invalid debugfs buffer ptr");
		return -EINVAL;
	}

	if (*ppos != 0)
		return 0;
	len = STMN_MSG_BUF_LEN_MAX +  (STMN_MSG_RAM_BUF_LEN *
			dev_hndl->netdev->real_num_tx_queues) +
			(STMN_MSG_RAM_BUF_LEN *
			dev_hndl->netdev->real_num_rx_queues);
	msg = kcalloc(len, sizeof(unsigned char), GFP_KERNEL);
	if (!msg) {
		pr_err("debugfs OOM");
		return 0;
	}

	ret = stmn_print_msg(dev_hndl, msg, len);
	if (ret < STMN_SUCCESS)
		pr_warn("%s : stmn_print_msg failed", __func__);

	stmn_print_ram_status_msg(dev_hndl, msg, len,
				  dev_hndl->netdev->real_num_tx_queues,
				  dev_hndl->netdev->real_num_rx_queues,
				  0);

	if (*ppos >= strlen(msg))
		return 0;

	if (*ppos + count > strlen(msg))
		count = strlen(msg) - *ppos;

	if (copy_to_user(buffer, msg, count))
		return -EFAULT;

	*ppos += count;
	kfree(msg);
	return count;
}

static ssize_t cmac_read(struct file *filp, char __user *buffer, size_t count,
			 loff_t *ppos)
{
	char *msg;
	u32 len = CMAC_MSG_BUF_LEN_MAX;
	u32 i;
	struct qep_priv *dev_hndl;

	if (!filp) {
		pr_err("invalid debugfs file ptr");
		return -EINVAL;
	}

	dev_hndl = (struct qep_priv *)(filp->private_data);
	if (!dev_hndl) {
		pr_err("invalid debugfs dev ptr");
		return -EINVAL;
	}

	if (!dev_hndl->netdev) {
		pr_err("invalid debugfs netdev ptr");
		return -EINVAL;
	}

	if (*ppos != 0)
		return 0;

	msg = kcalloc(len, sizeof(unsigned char), GFP_KERNEL);
	if (!msg)
		return -EINVAL;


	if (dev_hndl->dev_state == QEP_DEV_STATE_UP) {
		qep_get_cmac_stats(dev_hndl->netdev, len - strlen(msg),
			   msg + strlen(msg));

		for (i = 0; i < dev_hndl->netdev->real_num_tx_queues; i++)
			snprintf(msg + strlen(msg), len - strlen(msg),
				"txq %d %llu\n", i,
				dev_hndl->tx_q[i].stats.tx_packets);

		for (i = 0; i < dev_hndl->netdev->real_num_rx_queues; i++)
			snprintf(msg + strlen(msg), len - strlen(msg),
				"rxq_%d %llu\n", i,
				dev_hndl->rx_q[i].stats.rx_packets);

		qep_cmac_get_hw_status(&dev_hndl->cmac_instance,
				msg + strlen(msg),
				len - strlen(msg));
	}

	if (*ppos >= strlen(msg))
		return 0;

	if (*ppos + count > strlen(msg))
		count = strlen(msg) - *ppos;

	if (copy_to_user(buffer, msg, count))
		return -EFAULT;

	*ppos += count;
	kfree(msg);
	return count;
}

static const struct file_operations config_fops = {
	.owner = THIS_MODULE,
	.open = config_open,
	.read = config_read,
};

static const struct file_operations stmn_fops = {
	.owner = THIS_MODULE,
	.open = stmn_open,
	.read = stmn_read,
};

static const struct file_operations cmac_fops = {
	.owner = THIS_MODULE,
	.open = stmn_open,
	.read = cmac_read,
};

int qep_debugfs_dev_init(struct qep_priv *xpriv)
{
	struct dentry *temp;
	char dname[QDMA_DEV_NAME_SZ] = { 0 };
	struct pci_dev *pdev = xpriv->pcidev;

	snprintf(dname, QDMA_DEV_NAME_SZ, "%02x:%02x:%x", pdev->bus->number,
		 PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));

	/* create a directory for the device in debugfs */
	xpriv->debugfs_dev_root = debugfs_create_dir(dname, qep_debugfs_root);
	if (!xpriv->debugfs_dev_root) {
		qep_err(drv, "Failed to create device directory\n");
		goto func_exit;
	}

	temp = debugfs_create_file("qep_config", 0644, xpriv->debugfs_dev_root,
				   xpriv, &config_fops);
	if (!temp) {
		qep_err(drv, "qep_dev: failed to create qep_config\n");
		goto func_exit;
	}

	temp = debugfs_create_file("stmn", 0644, xpriv->debugfs_dev_root, xpriv,
				   &stmn_fops);
	if (!temp) {
		qep_err(drv, "qep_dev: failed to create stmn\n");
		goto func_exit;
	}
	temp = debugfs_create_file("cmac", 0644, xpriv->debugfs_dev_root, xpriv,
					   &cmac_fops);
	if (!temp) {
		qep_err(drv, "qep_dev: failed to create stmn\n");
		goto func_exit;
	}

	return 0;

func_exit:
	debugfs_remove_recursive(xpriv->debugfs_dev_root);
	return -1;
}

void qep_debugfs_dev_exit(struct qep_priv *xpriv)
{
	debugfs_remove_recursive(xpriv->debugfs_dev_root);
}

int qep_debugfs_init(void)
{
	qep_debugfs_root = debugfs_create_dir("qep_dev", NULL);
	if (!qep_debugfs_root) {
		pr_err("qep_dev: failed to create qep_dev\n");
		return -EINVAL;
	}

	return 0;
}

void qep_debugfs_exit(void)
{
	debugfs_remove_recursive(qep_debugfs_root);
	qep_debugfs_root = NULL;
}


#if (QEP_DBG_DUMP_EN)
void dump_skb(struct qep_priv *xpriv, struct sk_buff *skb)
{
	if (xpriv && skb) {
		char prefix[30];
		int no_raw_element = 16;

		scnprintf(prefix, sizeof(prefix),
			  "%s: %s: ", dev_name(&xpriv->pcidev->dev), __func__);

		print_hex_dump_debug(prefix, DUMP_PREFIX_NONE, no_raw_element,
				     1, skb->data, skb->len, true);

		/* qep_info(drv, "\n--- Head ----\n");
		 * print_hex_dump_debug(prefix, DUMP_PREFIX_NONE,
		 * no_raw_element, 1, skb->head, skb->len, true);
		 */
	}
}

void dump_pg(struct qep_priv *xpriv, struct page *pg)
{
	if (xpriv && pg) {
		char prefix[30];
		int no_raw_element = 16;

		scnprintf(prefix, sizeof(prefix),
			  "%s: %s: ", dev_name(&xpriv->pcidev->dev), __func__);

		print_hex_dump_debug(prefix, DUMP_PREFIX_NONE, no_raw_element,
				     1, page_address(pg), PAGE_SIZE, true);
	}
}

void dump_link_stats(struct qep_priv *xpriv, struct rtnl_link_stats64 stats)
{
	int i;

	memset(&stats, 0, sizeof(struct rtnl_link_stats64));
	for (i = 0; i < xpriv->netdev->real_num_rx_queues; i++) {
		stats.rx_bytes += xpriv->rx_q[i].stats.rx_bytes;
		stats.rx_packets += xpriv->rx_q[i].stats.rx_packets;
		stats.rx_errors += xpriv->rx_q[i].stats.rx_errors;
		stats.rx_dropped += xpriv->rx_q[i].stats.rx_dropped;
	}
	for (i = 0; i < xpriv->netdev->real_num_tx_queues; i++) {
		stats.tx_bytes += xpriv->tx_q[i].stats.tx_bytes;
		stats.tx_packets += xpriv->tx_q[i].stats.tx_packets;
		stats.tx_errors += xpriv->tx_q[i].stats.tx_errors;
		stats.tx_dropped += xpriv->tx_q[i].stats.tx_dropped;
	}

	qep_info(drv,
		"%s : RX-> pkts:%llu byte:%llu error:%llu dropped:%llu TX-> pkts:%llu byte:%llu error:%llu dropped:%llu",
		__func__, stats.rx_packets, stats.rx_bytes, stats.rx_errors,
		stats.rx_dropped, stats.tx_packets, stats.tx_bytes,
		stats.tx_errors, stats.tx_dropped);
}

void dump_csr_config(struct qep_priv *xpriv, struct global_csr_conf *csr_conf)
{
	int index;

	qep_dbg(xpriv->netdev, "%s: Dumping Ring Size Global CSR\n", __func__);
	for (index = 0; index < QDMA_GLOBAL_CSR_ARRAY_SZ; index++)
		qep_dbg(xpriv->netdev, "%d ", csr_conf->ring_sz[index]);

	qep_dbg(xpriv->netdev, "%s: Dumping c2h_timer_cnt Global CSR\n",
		__func__);
	for (index = 0; index < QDMA_GLOBAL_CSR_ARRAY_SZ; index++)
		qep_dbg(xpriv->netdev, "%d ", csr_conf->c2h_timer_cnt[index]);

	qep_dbg(xpriv->netdev, "%s: Dumping c2h_cnt_th Global CSR\n", __func__);
	for (index = 0; index < QDMA_GLOBAL_CSR_ARRAY_SZ; index++)
		qep_dbg(xpriv->netdev, "%d ", csr_conf->c2h_cnt_th[index]);

	qep_dbg(xpriv->netdev, "%s: Dumping c2h_buf_sz Global CSR\n", __func__);
	for (index = 0; index < QDMA_GLOBAL_CSR_ARRAY_SZ; index++)
		qep_dbg(xpriv->netdev, "%d ", csr_conf->c2h_buf_sz[index]);
}

void dump_qdma_sw_sgl(unsigned int sgcnt, struct qdma_sw_sg *sgl)
{
	struct qdma_sw_sg *temp;
	int i = 0;

	temp = sgl;
	while (temp && i < sgcnt) {
		pr_info("SG:temp->len=%d, temp->pg=%p, temp->dma_addr=%llx ",
			temp->len, temp->pg, temp->dma_addr);
		temp = temp->next;
		i++;
	}
}
#endif
