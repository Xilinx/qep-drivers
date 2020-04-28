/*
 * Copyright(c) 2019-2020 Xilinx, Inc. All rights reserved.
 *
 *This source code is free software; you can redistribute it and/or modify it
 *under the terms and conditions of the GNU General Public License,
 *version 2, as published by the Free Software Foundation.
 *
 *This program is distributed in the hope that it will be useful, but WITHOUT
 *ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *more details.
 *
 *The full GNU General Public License is included in this distribution in
 *the file called "COPYING".
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>

#include "i2c/xiic_l.h"
#include "ethtool_qsfp/internal.h"
#include "ethtool_qsfp/qsfp.h"

/****************************************************************************/
#define QEPCTL_PF0_IIC_BASE_OFFSET (0x00041000)

#define QEPCTL_QSFP_SLAVE_ADDR (0x50)
#define QEPCTL_PCA9546A_SLAVE_ADDRESS (0x74)
#define QEPCTL_PCA9546A_MAX_CHANNELS (4)
#define QEPCTL_PCA9546A_QSFP0_CHAN (0)

#define QEPCTL_SFF8636_MAX_PAGE (5)
#define QEPCTL_SFF8636_PAGE_SIZE (128)
#define QEPCTL_SFF8636_PAGE_SELECT_BYTE (127)

#define QEPCTL_SFF8636_PAGE1_FLAT_OFFSET (256)
#define QEPCTL_SFF8636_PAGE2_FLAT_OFFSET (384)
#define QEPCTL_SFF8636_PAGE3_FLAT_OFFSET (512)

#define QSFP_VN_START (148)
#define QSFP_VN_END (164)
#define MAX_REG_DESC_LEN (128)
/****************************************************************************/
/* Struct to dump registers */
struct i2c_reg {
	u32 offset;
	char name[MAX_REG_DESC_LEN];
} i2c_ctr[] = {
		{ 0x1C, "Global Interrupt Enable Register" },
		{ 0x20,  "Interrupt Status Register"},
		{ 0x28,  "Interrupt Enable Register"},
		{ 0x40, "TR Soft Reset Register"},
		{ 0x100, "Control Register"},
		{ 0x104, "Status Register"},
		{ 0x108, "FIFO Transmit FIFO Register" },
		{ 0x10C, "FIFO Receive FIFO Register" },
		{ 0x110,  "Slave Address Register"},
		{ 0x114, "Transmit FIFO Occupancy Register" },
		{ 0x118, "Receive FIFO Occupancy Register" },
		{ 0x11C, "Slave Ten Bit Address Register" },
		{ 0x120,  "Receive FIFO Programmable Depth Register" },
		{ 0x124,  "General Purpose Output Register" },
		{ 0x128, "Timing Parameter Register" },
		{ 0x12C, "Timing Parameter Register" },
		{ 0x130, "Timing Parameter Register" },
		{ 0x134, "Timing Parameter Register" },
		{ 0x138, "Timing Parameter Register" },
		{ 0x13C, "Timing Parameter Register" },
		{ 0x140, "Timing Parameter Register" },
		{ 0x144,  "Timing Parameter Register" }
};

/****************************************************************************/
/* Read @byte_count bytes on i2C slave at @slave_addr into @read_buffer */
static int qepctl_i2c_read(void *base, u8 slave_addr,
		u8 register_addr, u8 *read_buffer, u8 byte_count)
{
	u8 received_byte_count = 0;

	if (!read_buffer || byte_count == 0) {
		fprintf(stderr, " %s, Invalid args\n", __func__);
		return -EINVAL;
	}

	/* send slave address and register offset on bus*/
	received_byte_count = XIic_Send(((u64)base + QEPCTL_PF0_IIC_BASE_OFFSET),
			slave_addr, &register_addr, sizeof(register_addr),
			XIIC_STOP);
	if (received_byte_count != sizeof(register_addr)) {
		fprintf(stderr, " %s, XIic_Send failed\n", __func__);
		return -EAGAIN;
	}

	usleep(300);

	/* Read bytes from bus  */
	received_byte_count = XIic_Recv(
			((u64)base + QEPCTL_PF0_IIC_BASE_OFFSET),
			slave_addr, read_buffer, byte_count, XIIC_STOP);
	if (received_byte_count != byte_count) {
		fprintf(stderr, "%s, XIic_Recv failed\n", __func__);
		return -EAGAIN;
	}

	return 0;
}

/* Write @byte_count byte from @write_buffer on I2C slave @slave_addr */
static int qepctl_i2c_write(void *base, u8 slave_addr, u8 *write_buffer,
		u8 byte_count)
{
	u8 sent_byte_count = 0;

	if (!write_buffer || byte_count == 0) {
		fprintf(stderr, " %s, Invalid args\n", __func__);
		return -EINVAL;
	}

	sent_byte_count = XIic_Send(
			((u64)base + QEPCTL_PF0_IIC_BASE_OFFSET),
			slave_addr, write_buffer, byte_count, XIIC_STOP);

	if (sent_byte_count != byte_count) {
		fprintf(stderr, " %s, XIic_Send failed\n", __func__);
		return -EAGAIN;
	}

	return 0;
}

/* Dump I2C pg-090 axi-i2c registers*/
void qepctl_i2c_dump_reg(void *base)
{
	u32 *base_i2c = (u32 *)base + QEPCTL_PF0_IIC_BASE_OFFSET;
	int i;
	int num_reg = sizeof(i2c_ctr) / sizeof(struct i2c_reg);

	for (i = 0; i < num_reg; i++)
		printf(" %s=0x%x\n", i2c_ctr[i].name,
			(u32)*(base_i2c+i2c_ctr[i].offset));
}

/* Select channel by writing PCA9548A control register */
static int qepctl_pca9546a_write(void *base, u8 slave_addr, u8 channel_num)
{
	u8 channel_number = 0, bytes_to_send = 1;
	int ret = 0;

	if (channel_num >= QEPCTL_PCA9546A_MAX_CHANNELS) {
		fprintf(stderr, " %s, Invalid channel number Err:%d\n",
				__func__, ret);
		return -EINVAL;
	}

	channel_number = 1 << channel_num;

	ret = qepctl_i2c_write(base, slave_addr, &channel_number,
			bytes_to_send);
	if (ret < 0) {
		fprintf(stderr, " %s, qepctl_i2c_write failed Err:%d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

/* Read MUX channel */
static int qepctl_pca9546_read(void *base, u8 slave_addr, u8 *channel_num)
{
	u8 received_byte_count = 0;

	received_byte_count = XIic_Recv(
			((u64)base + QEPCTL_PF0_IIC_BASE_OFFSET),
			slave_addr, channel_num, 1, XIIC_STOP);
	if (received_byte_count != 1) {
		fprintf(stderr, "%s, XIic_Recv failed\n", __func__);
		return -EAGAIN;
	}

	return 0;
}

/* Read from QSFP I2C slave*/
static int qepctl_qsfp_read(void *base, u8 qsfp_register_offset, u8 *qsfp_data,
		u8 num_of_bytes_to_read)
{
	int ret;

	ret = qepctl_i2c_read(base, QEPCTL_QSFP_SLAVE_ADDR,
			qsfp_register_offset, qsfp_data,
			num_of_bytes_to_read);

	if (ret < 0) {
		fprintf(stderr, " %s, qepctl_i2c_read failed Err:%d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

/* Write on qsfp slave*/
static int qepctl_qsfp_write(void *base, u8 qsfp_register_offset, u8 qsfp_data)
{
	u8 num_of_bytes_to_write;
	u8 write_data[2];
	int ret;

	write_data[0] = qsfp_register_offset;
	write_data[1] = qsfp_data;
	num_of_bytes_to_write = 2;

	ret = qepctl_i2c_write(base, QEPCTL_QSFP_SLAVE_ADDR,
			write_data, num_of_bytes_to_write);

	if (ret < 0) {
		fprintf(stderr, " %s, qepctl_i2c_write failed Err:%d\n",
			__func__, ret);
		return ret;
	}
	return 0;
}

/* SFF8636 has lower page and upper page. Upper page content depend on page
 * select byte at  QEPCTL_SFF8636_PAGE_SIZE. To read a page content we write
 * the page number then read the content from upper page
 */
static int qepctl_qsfp_read_page(void *base, int page_num, u8 *data)
{
	int i = 0;
	int ret;

	if (page_num > QEPCTL_SFF8636_MAX_PAGE) {
		fprintf(stderr, "%s, invalid page no\n", __func__);
		return -EINVAL;
	}

	/*Read lower page*/
	if (page_num == -1) {
		for (i = 0; i < QEPCTL_SFF8636_PAGE_SIZE; i++) {
			ret = qepctl_qsfp_read(base, i, &data[i], 1);
			if (ret) {
				fprintf(stderr, "%s, qepctl_qsfp_read failed Err:%d\n",
						__func__, ret);
				return ret;
			}
		}
	} else {
		/*select page no and read the upper page*/
		ret = qepctl_qsfp_write(base, QEPCTL_SFF8636_PAGE_SELECT_BYTE, page_num);
		if (ret) {
			fprintf(stderr, "%s, qepctl_qsfp_write failed Err:%d\n",
					__func__, ret);
			return ret;
		}
		for (i = 0; i < QEPCTL_SFF8636_PAGE_SIZE; i++) {
			ret = qepctl_qsfp_read(base, i + QEPCTL_SFF8636_PAGE_SIZE,
					&data[i], 1);
			if (ret) {
				fprintf(stderr, "%s, qepctl_qsfp_read failed Err:%d\n",
						__func__, ret);
				return ret;
			}
		}
	}
	return 0;
}

/* Read page data from QSFP module */
static int qepctl_qsfp_read_data(void *base, u8 *data, int len)
{
	u16 offset = 0;
	u8 page_size = QEPCTL_SFF8636_PAGE_SIZE;
	int page_num = -1;
	int ret;

	while (offset < len) {
		ret = qepctl_qsfp_read_page(base, page_num, data);
		if (ret) {
			fprintf(stderr, "%s, read_qsfp_page failed Err:%d\n",
					__func__, ret);
			return ret;
		}
		data = data + page_size;
		offset = offset + page_size;
		page_num++;
	}

	return 0;
}

static int qepctl_qsfp_select_channel(void *base, int channel)
{

	u8 channel_num = 0;
	int ret = 0;

	ret = qepctl_pca9546a_write(base, QEPCTL_PCA9546A_SLAVE_ADDRESS,
			channel);
	if (ret) {
		fprintf(stderr, " %s, qepctl_pca9546a_write failed Err:%d\n",
				__func__, ret);
		return ret;
	}

	ret = qepctl_pca9546_read(base, QEPCTL_PCA9546A_SLAVE_ADDRESS,
			&channel_num);
	if (ret) {
		fprintf(stderr, " %s, qepctl_pca9546_read failed Err:%d\n",
				__func__, ret);
		return ret;
	}

	printf("channel_num: %d\n", channel_num);
	assert(channel_num - 1 == channel);

	return 0;
}

/* Display QSFP module info to stdout as per SFF8636 spec */
int qepctl_qsfp_show_info(void *base, bool alarm)
{
	u8 *data = NULL;
	int ret;
	int len = ETH_MODULE_SFF_8636_LEN;

	if (alarm)
		len = ETH_MODULE_SFF_8436_MAX_LEN;

	data = calloc(len, sizeof(char));
	if (!data) {
		fprintf(stderr, " %s, calloc failed\n", __func__);
		return -ENOMEM;
	}

	ret = XIic_DynInit((u64)base + QEPCTL_PF0_IIC_BASE_OFFSET);
	if (ret) {
		fprintf(stderr, " %s, XIic_DynInit failed Err:%d\n",
				__func__, ret);
		free(data);
		return ret;
	}

	ret = qepctl_qsfp_select_channel(base, QEPCTL_PCA9546A_QSFP0_CHAN);
	if (ret) {
		fprintf(stderr, " %s, qepctl_qsfp_select_channel failed Err:%d\n",
				__func__, ret);
		free(data);
		return ret;
	}

	ret = qepctl_qsfp_read_data(base, data, ETH_MODULE_SFF_8636_LEN);
	if (ret) {
		fprintf(stderr, " %s, qepctl_qsfp_select_channel failed Err:%d\n",
			__func__, ret);
		free(data);
		return ret;
	}


	if (alarm) {
		if (data[SFF8636_OPTION_4_OFFSET] & SFF8636_O4_PAGE_01_PRESENT)
			ret = qepctl_qsfp_read_page(base,  0x1,
				data + QEPCTL_SFF8636_PAGE1_FLAT_OFFSET);

		if (data[SFF8636_OPTION_4_OFFSET] & SFF8636_O4_PAGE_02_PRESENT)
			ret = qepctl_qsfp_read_page(base, 0x2,
				data + QEPCTL_SFF8636_PAGE2_FLAT_OFFSET);

		if (!(data[SFF8636_STATUS_2_OFFSET] &
				SFF8636_STATUS_PAGE_3_PRESENT))
			ret = qepctl_qsfp_read_page(base, 0x3,
				data + QEPCTL_SFF8636_PAGE3_FLAT_OFFSET);
	}

	if (ret)
		len = ETH_MODULE_SFF_8636_LEN;

	sff8636_show_all(data, len);

	free(data);
	return 0;
}


int qepctl_qsfp_print_vendor(void *base)
{
	u8 regs_buff[QSFP_VN_END - QSFP_VN_START];
	int i = 0;
	int ret;

	memset(regs_buff, 0, sizeof(regs_buff));

	/* Setting the mux channel as 0 as QSFP is placed at mux channel 0 */
	ret = qepctl_qsfp_select_channel(base, QEPCTL_PCA9546A_QSFP0_CHAN);
	if (ret) {
		fprintf(stderr, " %s, qepctl_qsfp_select_channel failed Err:%d\n"
				, __func__, ret);
		return -EINVAL;
	}

	qepctl_qsfp_write(base, QEPCTL_SFF8636_PAGE_SELECT_BYTE, 0);
	for (i = QSFP_VN_START; i < QSFP_VN_END; i++) {
		ret = qepctl_qsfp_read(base, i, &regs_buff[i - QSFP_VN_START],
			1);
		if (ret) {
			fprintf(stderr, " %s, qepctl_qsfp_read failed Err:%d\n"
				, __func__, ret);
			return -EINVAL;
		}
	}
	fprintf(stdout, " vendor: %s\n", (char *)regs_buff);

	return 0;
}

