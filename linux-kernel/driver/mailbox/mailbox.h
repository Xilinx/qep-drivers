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
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#ifndef MAILBOX_H_
#define MAILBOX_H_

#include <linux/platform_device.h>
#include <linux/fs.h>
#include "mailbox_proto.h"
#include <linux/slab.h>

#define XOCL_MAX_DEVICES	24

#define  NO_MMAP

#define SUBDEV_SUFFIX	".q"
#define XOCL_DEVNAME(str)	str SUBDEV_SUFFIX
#define	XOCL_MAILBOX		"qep_mailbox"
#define NODE_MAILBOX_MGMT "ep_mailbox_mgmt_00"
#define NODE_MAILBOX_USER "ep_mailbox_user_00"

typedef	void *xdev_handle_t;
typedef	void (*mailbox_msg_cb_t)(void *arg, void *data, size_t len,
	u64 msgid, int err, bool sw_ch);


enum mb_kind {
	DAEMON_STATE,
	CHAN_STATE,
	CHAN_SWITCH,
	COMM_ID,
	VERSION,
};

struct xocl_drv_private {
	void			*ops;
	const struct file_operations	*fops;
	dev_t			dev;
	char			*cdev_name;
};


struct xocl_subdev_funcs {
	int (*offline)(struct platform_device *pdev);
	int (*online)(struct platform_device *pdev);
};

#define offline_cb common_funcs.offline
#define online_cb common_funcs.online

struct xocl_mailbox_funcs {
	struct xocl_subdev_funcs common_funcs;
	int (*request)(struct platform_device *pdev, void *req,
		size_t reqlen, void *resp, size_t *resplen,
		mailbox_msg_cb_t cb, void *cbarg, u32 timeout);
	int (*post_notify)(struct platform_device *pdev, void *req, size_t len);
	int (*post_response)(struct platform_device *pdev,
		enum xcl_mailbox_request req, u64 reqid, void *resp, size_t len);
	int (*listen)(struct platform_device *pdev,
		mailbox_msg_cb_t cb, void *cbarg);
	int (*set)(struct platform_device *pdev, enum mb_kind kind, u64 data);
	int (*get)(struct platform_device *pdev, enum mb_kind kind, u64 *data);
};

struct mailbox;
extern  struct mailbox *mbx_data;

static inline void *xocl_drvinst_alloc(struct device *dev, u32 size)
{
	return kzalloc(size, GFP_KERNEL);
}

static inline void *xocl_drvinst_open(void *file_dev)
{
	return mbx_data;
}

static inline void xocl_drvinst_close(void *data)
{
	return;
}

static inline void xocl_drvinst_free(void* data) 
{
	kfree(data);
}



/* #define	SUBDEV(xdev, id)	\
	(XDEV(xdev)->subdevs[id][0])

#define	MAILBOX_DEV(xdev)	SUBDEV(xdev, XOCL_SUBDEV_MAILBOX).pldev
#define	MAILBOX_OPS(xdev)	\
	((struct xocl_mailbox_funcs *)SUBDEV(xdev, XOCL_SUBDEV_MAILBOX).ops)
#define MAILBOX_READY(xdev, cb)	\
	(MAILBOX_DEV(xdev) && MAILBOX_OPS(xdev) && MAILBOX_OPS(xdev)->cb)
#define	xocl_peer_request(xdev, req, reqlen, resp, resplen, cb, cbarg, timeout)	\
	(MAILBOX_READY(xdev, request) ? MAILBOX_OPS(xdev)->request(MAILBOX_DEV(xdev), \
	req, reqlen, resp, resplen, cb, cbarg, timeout) : -ENODEV)
#define	xocl_peer_response(xdev, req, reqid, buf, len)			\
	(MAILBOX_READY(xdev, post_response) ? MAILBOX_OPS(xdev)->post_response(	\
	MAILBOX_DEV(xdev), req, reqid, buf, len) : -ENODEV)
#define	xocl_peer_notify(xdev, req, reqlen)				\
	(MAILBOX_READY(xdev, post_notify) ? MAILBOX_OPS(xdev)->post_notify(		\
	MAILBOX_DEV(xdev), req, reqlen) : -ENODEV)
#define	xocl_peer_listen(xdev, cb, cbarg)				\
	(MAILBOX_READY(xdev, listen) ? MAILBOX_OPS(xdev)->listen(MAILBOX_DEV(xdev), \
	cb, cbarg) : -ENODEV)
#define	xocl_mailbox_set(xdev, kind, data)				\
	(MAILBOX_READY(xdev, set) ? MAILBOX_OPS(xdev)->set(MAILBOX_DEV(xdev), \
	kind, data) : -ENODEV)
#define	xocl_mailbox_get(xdev, kind, data)				\
	(MAILBOX_READY(xdev, get) ? MAILBOX_OPS(xdev)->get(MAILBOX_DEV(xdev), \
	kind, data) : -ENODEV) */

/*
 * Msg will be sent to peer and reply will be received.
 */
int mailbox_request(struct platform_device *pdev, void *req, size_t reqlen,
	void *resp, size_t *resplen, mailbox_msg_cb_t cb,
	void *cbarg, u32 resp_ttl);

/*
 * Request will be posted, no wait for reply.
 */
int mailbox_post_notify(struct platform_device *pdev, void *buf, size_t len);
/*
 * Response will be always posted, no waiting.
 */
int mailbox_post_response(struct platform_device *pdev,
	enum xcl_mailbox_request req, u64 reqid, void *buf, size_t len);

/*
 * Register  callback
 */
int mailbox_listen(struct platform_device *pdev,
	mailbox_msg_cb_t cb, void *cbarg);
/*
 * connect to peer
 */
int  mailbox_connect(struct platform_device *pdev);

int mailbox_get(struct platform_device *pdev, enum mb_kind kind, u64 *data);
int mailbox_set(struct platform_device *pdev, enum mb_kind kind, u64 data);

void xocl_fini_mailbox(void);
int __init xocl_init_mailbox(void);

#endif