/*
 * Copyright (c) 2019-2020 Xilinx, Inc.
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

static int check_valid_args(struct file *filp, char __user *buffer)
{
	struct qep_priv *xpriv;

	if (!filp) {
		pr_err("%s: Invalid debugfs file ptr\n", __func__);
		return -EINVAL;
	}

	xpriv = (struct qep_priv *)(filp->private_data);
	if (!xpriv) {
		pr_err("%s: Invalid xpriv\n", __func__);
		return -EINVAL;
	}

	if (!xpriv->netdev) {
		pr_err("%s: Invalid debugfs netdev ptr\n", __func__);
		return -EINVAL;
	}

	if (!buffer) {
		pr_err("%s: Invalid debugfs buffer ptr\n", __func__);
		return -EINVAL;
	}

	return 0;
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
#define TEMP_BUF_SIZE 512
	char *temp;
	struct qep_priv *xpriv;
	struct global_csr_conf l_global_csr_conf;

	ret = check_valid_args(filp, buffer);
	if (ret)
		return ret;

	if (*ppos != 0)
		return 0;

	xpriv = (struct qep_priv *)(filp->private_data);
	temp = kzalloc(TEMP_BUF_SIZE, GFP_KERNEL);
	if (!temp)
		return -ENOMEM;
	memset(temp, '\0', TEMP_BUF_SIZE);

	ret = qdma_global_csr_get(xpriv->dev_handle, 0,
				  QDMA_GLOBAL_CSR_ARRAY_SZ, &l_global_csr_conf);
	if (ret != 0)
		qep_warn(drv,
			 "%s: qdma_global_csr_get() failed with status %d\n",
			 __func__, ret);

	snprintf(temp, TEMP_BUF_SIZE,
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
		"cmpl_rng_sz_idx		= %d\n"
		"cmpl_rng_sz			= %d\n"
		"rx_int_cnt			= %llu\n"
		"tx_int_cnt			= %llu\n"
		"rx_napi_cnt			= %llu\n",
		xpriv->rs_fec_en, xpriv->netdev->real_num_rx_queues,
		xpriv->netdev->real_num_rx_queues,
		xpriv->qdma_dev_conf.msix_qvec_max, xpriv->rx_desc_rng_sz_idx,
		l_global_csr_conf.ring_sz[xpriv->rx_desc_rng_sz_idx],
		xpriv->tx_desc_rng_sz_idx,
		l_global_csr_conf.ring_sz[xpriv->tx_desc_rng_sz_idx],
		xpriv->rx_buf_sz_idx,
		l_global_csr_conf.c2h_buf_sz[xpriv->rx_buf_sz_idx],
		xpriv->cmpl_rng_sz_idx,
		l_global_csr_conf.ring_sz[xpriv->cmpl_rng_sz_idx],
		xpriv->drv_stats->rx_int_cnt, xpriv->drv_stats->tx_int_cnt,
		xpriv->drv_stats->rx_napi_cnt);

	if (*ppos >= strlen(temp)) {
		goto cleanup;
		ret = 0;
	}

	if (*ppos + count > strlen(temp))
		count = strlen(temp) - *ppos;

	if (copy_to_user(buffer, temp, count)) {
		goto cleanup;
		ret = -EFAULT;
	}
	*ppos += count;
cleanup:
	kfree(temp);
	return (ret < 0) ? ret : count;
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
	struct qep_priv *xpriv;

	ret = check_valid_args(filp, buffer);
	if (ret)
		return ret;

	if (*ppos != 0)
		return 0;

	xpriv = (struct qep_priv *)(filp->private_data);
	len = STMN_MSG_BUF_LEN_MAX
		+ (STMN_MSG_RAM_BUF_LEN * xpriv->netdev->real_num_tx_queues)
		+ (STMN_MSG_RAM_BUF_LEN * xpriv->netdev->real_num_rx_queues);
	msg = kcalloc(len, sizeof(unsigned char), GFP_KERNEL);
	if (!msg) {
		pr_err("%s: debugfs OOM\n", __func__);
		return 0;
	}

	ret = stmn_print_msg(xpriv, msg, len);
	if (ret < STMN_SUCCESS)
		pr_warn("%s : stmn_print_msg() failed", __func__);

	stmn_print_ram_status_msg(xpriv, msg, len,
				  xpriv->netdev->real_num_tx_queues,
				  xpriv->netdev->real_num_rx_queues,
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
	struct qep_priv *xpriv;
	int ret;
	unsigned int i, max_q = 0, len = CMAC_MSG_BUF_LEN_MAX;
	u64 rx_pkt = 0, tx_pkt = 0;

	ret = check_valid_args(filp, buffer);
	if (ret)
		return ret;

	len += qep_lbus_get_buf_len();
	msg = kcalloc(len, sizeof(unsigned char), GFP_KERNEL);
	if (!msg)
		return -EINVAL;

	xpriv = (struct qep_priv *)(filp->private_data);
	if (xpriv->dev_state != QEP_DEV_STATE_UP)
		goto func_exit;


	ret = qep_cmac_stats_get(&xpriv->cmac_dev, xpriv->mac_stats);
	if (ret) {
		pr_err(" %s: qep_cmac_stats_get() failed\n", __func__);
		goto func_exit;
	}

	ret = qep_drv_stats_snprintf(xpriv->drv_stats, msg + strlen(msg),
			len - strlen(msg));
	if (ret) {
		pr_err(" %s: qep_drv_stats_snprintf() failed\n", __func__);
		goto func_exit;
	}

	ret = qep_cmac_stats_snprintf(xpriv->mac_stats, msg + strlen(msg),
			len - strlen(msg));
	if (ret) {
		pr_err(" %s: qep_cmac_stats_snprintf() failed\n", __func__);
		goto func_exit;
	}

	ret = qep_cmac_status_snprintf(&xpriv->cmac_dev,
			msg + strlen(msg), len - strlen(msg));
	if (ret) {
		pr_err(" %s: qep_cmac_status_snprintf() failed\n", __func__);
		goto func_exit;
	}

	snprintf(msg + strlen(msg), len - strlen(msg), "QID TX RX\n");
	max_q =  max_t(unsigned int, xpriv->netdev->real_num_tx_queues,
			xpriv->netdev->real_num_rx_queues);
	for (i = 0; i < max_q; i++) {
		rx_pkt = 0;
		tx_pkt = 0;
		if (i < xpriv->netdev->real_num_tx_queues)
			tx_pkt = xpriv->tx_qstats[i].tx_packets;
		if (i < xpriv->netdev->real_num_rx_queues)
			rx_pkt = xpriv->rx_qstats[i].rx_packets;
		snprintf(msg + strlen(msg), len - strlen(msg),
				"%d %llu %llu\n", i, tx_pkt, rx_pkt);
	}

	ret = qep_lbus_snprintf(xpriv->bar_base + QEP_TOP_BASE,
			msg + strlen(msg),  len - strlen(msg));
	if (ret) {
		pr_err(" %s: qep_lbus_snprintf() failed\n", __func__);
		goto func_exit;
	}
	snprintf(msg + strlen(msg), len - strlen(msg), "\n");

func_exit:
	kfree(msg);

	if (*ppos >= strlen(msg))
		return 0;

	if (*ppos + count > strlen(msg))
		count = strlen(msg) - *ppos;

	if (copy_to_user(buffer, msg, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static ssize_t pm_suspend_write(struct file *filp, const char __user *buffer,
			size_t count, loff_t *ppos)
{
	struct qep_priv *xpriv;
	int ret;

	struct device *dev;

	ret = check_valid_args(filp, (char *)buffer);
	if (ret) {
		pr_err("%s: Invalid arguments passed\n", __func__);
		return ret;
	}

	xpriv = (struct qep_priv *)(filp->private_data);
	if (xpriv->dev_state != QEP_DEV_STATE_UP) {
		qep_err(drv, "Device is not active");
		return -EINVAL;
	}

	qep_info(drv, "Called %s()\n", __func__);

	dev = &xpriv->pcidev->dev;
	if (!dev) {
		qep_err(drv, "%s: Caught NULL pointer for dev\n", __func__);
		return -EINVAL;
	}

	ret = qep_suspend(dev);
	if (ret != 0) {
		qep_err(drv, "%s: qep_suspend() failed\n", __func__);
		return ret;
	}

	return count;
}

static ssize_t pm_resume_write(struct file *filp, const char __user *buffer,
			size_t count, loff_t *ppos)
{
	struct qep_priv *xpriv;
	int ret;

	struct device *dev;

	ret = check_valid_args(filp, (char *)buffer);
	if (ret) {
		pr_err("%s: Invalid arguments passed\n", __func__);
		return ret;
	}

	xpriv = (struct qep_priv *)(filp->private_data);
	if (xpriv->dev_state != QEP_DEV_STATE_DOWN) {
		qep_err(drv, "%s: Device is already active\n", __func__);
		return -EINVAL;
	}

	qep_info(drv, "Called %s()", __func__);

	dev = &xpriv->pcidev->dev;
	if (!dev) {
		qep_err(drv, "%s: Caught NULL pointer for dev", __func__);
		return -EINVAL;
	}

	ret = qep_resume(dev);
	if (ret) {
		qep_err(drv, "%s: qep_resume() is failed\n", __func__);
		return ret;
	}

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

static const struct file_operations pm_suspend_fops = {
	.owner = THIS_MODULE,
	.open =  stmn_open,
	.write = pm_suspend_write,
};

static const struct file_operations pm_resume_fops = {
	.owner = THIS_MODULE,
	.open = stmn_open,
	.write = pm_resume_write,
};

int qep_debugfs_dev_init(struct qep_priv *xpriv)
{
	struct dentry *temp;
	char dname[QDMA_DEV_NAME_SZ] = { 0 };
	struct pci_dev *pdev = xpriv->pcidev;
	struct dentry *debugfs_dev_pm_root;

	snprintf(dname, QDMA_DEV_NAME_SZ, "%02x:%02x:%x", pdev->bus->number,
		 PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));

	/* create a directory for the device in debugfs */
	xpriv->debugfs_dev_root = debugfs_create_dir(dname, qep_debugfs_root);
	if (!xpriv->debugfs_dev_root) {
		qep_err(drv, "%s: Failed to create qep debugfs device directory\n",
			__func__);
		goto func_exit;
	}
	/* create a directory for the pm ops in debugfs_dev_root directory */
	debugfs_dev_pm_root =
			debugfs_create_dir("power", xpriv->debugfs_dev_root);
	if (!xpriv->debugfs_dev_root) {
		qep_err(drv,
			"%s: Failed to create debugfs directory for Power Mgmt ops\n",
			__func__);
		goto func_exit;
	}

	temp = debugfs_create_file("qep_config", 0644, xpriv->debugfs_dev_root,
				   xpriv, &config_fops);
	if (!temp) {
		qep_err(drv,
		"%s: Failed to create debugfs file for qep_config\n", __func__);
		goto func_exit;
	}

	temp = debugfs_create_file("stmn", 0644, xpriv->debugfs_dev_root, xpriv,
				   &stmn_fops);
	if (!temp) {
		qep_err(drv, "%s: Failed to create debugfs file for stmn\n",
			__func__);
		goto func_exit;
	}
	temp = debugfs_create_file("cmac", 0644, xpriv->debugfs_dev_root, xpriv,
					   &cmac_fops);
	if (!temp) {
		qep_err(drv, "%s: Failed to create debugfs file for CMAC\n",
			__func__);
		goto func_exit;
	}

	temp = debugfs_create_file("suspend", 0644,
			debugfs_dev_pm_root, xpriv, &pm_suspend_fops);
	if (!temp) {
		qep_err(drv,
			"%s: Failed to create debugfs file for Power Mgmt ops (suspend)\n",
			__func__);
		goto func_exit;
	}
	temp = debugfs_create_file("resume", 0644,
			debugfs_dev_pm_root, xpriv, &pm_resume_fops);
	if (!temp) {
		qep_err(drv,
			"%s: Failed to create debugfs file for Power Mgmt ops (resume)\n",
			__func__);
		goto func_exit;
	}
	return 0;

func_exit:
	debugfs_remove_recursive(xpriv->debugfs_dev_root);
	return -ENODEV;
}

void qep_debugfs_dev_exit(struct qep_priv *xpriv)
{
	debugfs_remove_recursive(xpriv->debugfs_dev_root);
}

int qep_debugfs_init(void)
{
	qep_debugfs_root = debugfs_create_dir("qep_dev", NULL);
	if (!qep_debugfs_root) {
		pr_err("%s: Failed to create qep_dev\n", __func__);
		return -ENODEV;
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
