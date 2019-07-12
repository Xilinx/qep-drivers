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
#include "qep.h"
#include <linux/pci.h>
#include "stmn.h"

#define QDMA_DEV_NAME_SZ 32
#define CMAC_MSG_BUF_LEN_MAX 4096

struct dentry *qep_debugfs_root;

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
	struct qep_priv *xpriv = (struct qep_priv *)(filp->private_data);
	struct global_csr_conf l_global_csr_conf;

	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &l_global_csr_conf);
	if (ret != 0)
		qep_warn(drv,
			 "%s: qdma_global_csr_get() failed with status %d\n",
			 __func__, ret);

	sprintf(temp,
		"rs_fec_en			= %d\n"
		"num_queues			= %d\n"
		"ind_intr_mode			= %d\n"
		"num_msix			= %d\n"
		"rx_desc_rng_sz_idx		= %d\n"
		"rx_desc_rng_sz			= %d\n"
		"tx_desc_rng_sz_idx		= %d\n"
		"tx_desc_rng_sz			= %d\n"
		"rx_buf_sz_idx			= %d\n"
		"rx_buf_sz			= %d\n"
		"cmpl_rng_sz_idx                = %d\n"
		"cmpl_rng_sz			= %d\n",
		xpriv->rs_fec_en, xpriv->num_queues, ind_intr_mode,
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
	u32 len = STMN_MSG_BUF_LEN_MAX;
	struct qep_priv *dev_hndl = (struct qep_priv *)(filp->private_data);

	if (*ppos != 0)
		return 0;

	msg = kmalloc(len * sizeof(unsigned char), GFP_KERNEL);
	if (!msg)
		return 0;

	ret = stmn_print_msg(dev_hndl, msg, len);
	if (ret < STMN_SUCCESS)
		pr_warn("%s : stmn_print_msg failed", __func__);

	stmn_print_ram_status_msg(dev_hndl, msg, len, dev_hndl->num_queues, 0);

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
	struct qep_priv *dev_hndl = (struct qep_priv *)(filp->private_data);

	if (*ppos != 0)
		return 0;

	msg = kmalloc(len * sizeof(unsigned char), GFP_KERNEL);
	if (!msg)
		return 0;


	qep_get_cmac_stats(dev_hndl->netdev, len, msg);

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

static struct file_operations config_fops = {
	.owner = THIS_MODULE,
	.open = config_open,
	.read = config_read,
};

static struct file_operations stmn_fops = {
	.owner = THIS_MODULE,
	.open = stmn_open,
	.read = stmn_read,
};

static struct file_operations cmac_fops = {
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

#ifdef DEBUG_5
static void dump_skb(struct qep_priv *xpriv, struct sk_buff *skb)
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

static void dump_pg(struct qep_priv *xpriv, struct page *pg)
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

/* TODO Make a generic function*/
void dump_link_stats(struct qep_priv *xpriv, struct rtnl_link_stats64 stats)
{
	int i;

	memset(&stats, 0, sizeof(struct rtnl_link_stats64));
	for (i = 0; i < xpriv->num_queues; i++) {
		stats.rx_bytes += xpriv->rx_q[i].stats.rx_bytes;
		stats.rx_packets += xpriv->rx_q[i].stats.rx_packets;
		stats.rx_errors += xpriv->rx_q[i].stats.rx_errors;
		stats.rx_dropped += xpriv->rx_q[i].stats.rx_dropped;

		stats.tx_bytes += xpriv->tx_q[i].stats.tx_bytes;
		stats.tx_packets += xpriv->tx_q[i].stats.tx_packets;
		stats.tx_errors += xpriv->tx_q[i].stats.tx_errors;
		stats.tx_dropped += xpriv->tx_q[i].stats.tx_dropped;
	}

	qep_info(
		drv,
		"%s : RX-> pkts:%llu byte:%llu error:%llu dropped:%llu TX-> pkts:%llu byte:%llu error:%llu dropped:%llu",
		__func__, stats.rx_packets, stats.rx_bytes, stats.rx_errors,
		stats.rx_dropped, stats.tx_packets, stats.tx_bytes,
		stats.tx_errors, stats.tx_dropped);
}

void dump_csr_config(struct qep_priv *xpriv, struct global_csr_conf *csr_conf)
{
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

/*TODO Fix Crash in dump*/
void dump_queue(struct qep_priv *xpriv, int q_no, char error_str[], bool tx)
{
	int len = 0;
	unsigned long handle;

	if (tx)
		handle = xpriv->base_tx_q_handle + q_no;
	else
		handle = xpriv->base_rx_q_handle + q_no;

	memset(error_str, 0, QEP_ERROR_STR_BUF_LEN);
	len = qdma_queue_dump(xpriv->dev_handle, handle, error_str,
			      QEP_ERROR_STR_BUF_LEN);
	if (len < 0) {
		qep_err(drv,
			"%s: qdma_queue_dump() failed for queue %d with error %d(%s)\n",
			__func__, q_no, len, error_str);
	}
	qep_dbg(xpriv->netdev, "%s: Rx Queue DUMP = %s\n", __func__, error_str);
}

void dump_qdma_sw_sgl(unsigned int sgcnt, struct qdma_sw_sg *sgl)
{
	struct qdma_sw_sg *temp;
	int i = 0;

	temp = sgl;
	while (temp && i < sgcnt) {
		pr_info("SG:temp->len=%d, temp->pg=%p , temp->dma_addr=%llx ",
			temp->len, temp->pg, temp->dma_addr);
		temp = temp->next;
		i++;
	}
}
#endif
