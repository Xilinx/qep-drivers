/*-
 *   BSD LICENSE
 *
 *	 Copyright(c) 2019 Xilinx, Inc. All rights reserved.
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

#ifndef __QDMA_XCMAC_H__
#define __QDMA_XCMAC_H__

#define QEP_BASE_OFFSET                    (0x0)
#define QEP_BASE_CMAC_RESET_CONTROL_OFFSET (0x00000020)

#define QEP_CMAC_CTRL_BASE                 (0x00010000)
#define QEP_CMAC_CTRL_GT_OFFSET            (0x00000010)
#define QEP_CMAC_CTRL_RST_OFFSET           (0x00000014)
#define QEP_CMAC_CTRL_RST_RX_GTWIZ_SHIFT   (5)
#define QEP_CMAC_CTRL_RST_TX_GTWIZ_SHIFT   (4)
#define QEP_CMAC_CTRL_ERR_OFFSET           (0x0000001C)
#define QEP_CMAC_CTRL_GT_LOOPBACK          (0x0000024A)
#define QEP_CMAC_CTRL_TX_CTRL_OFFSET       (0x00000308)
#define QEP_CMAC_CTRL_TX_CTRL_TX_INHIBIT_SHIFT  (12)

#define QDMA_XCMAC_H
#define CMAC_100G_IP_BASE      (0x00020000)
/* CMAC DRP MTU Access */
#define QEP_CMAC_DRP_OFFSET                    (0xA4000)
/* Actual address is 0xAF, need to multiply by 4 */
#define QEP_CMAC_CTL_RX_MAX_PACKET_LEN_OFFSET  (0x2BC)
void xcmac_set_mtu(struct rte_eth_dev *dev, uint16_t new_mtu);
int xcmac_setup(struct rte_eth_dev *dev);
int xcmac_start(struct rte_eth_dev *dev);
int xcmac_stop(struct rte_eth_dev *dev);
void xcmac_link_check_thread_start(struct rte_eth_dev *dev);
void xcmac_link_check_thread_stop(struct rte_eth_dev *dev);

#endif /* end of protection macro */
