/*-
 * BSD LICENSE
 *
 * Copyright(c) 2018-2019 Xilinx, Inc. All rights reserved.
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

#ifndef __QDMA_USER_H__
#define __QDMA_USER_H__
/**
 * @file
 * @brief This file contains example design/user logic controlled
 * data structures and functions
 * The driver is specific to an example design, if the example design
 * changes user controlled parameters, this file needs to be modified
 * appropriately.
 * Structures for Completion entry, Descriptor bypass can be added here.
 */

#include "qdma_rxtx.h"

/**
 * @union - c2h_metadata
 * @brief	STMN Completion entry metadata
 */
union c2h_metadata {
	volatile uint64_t user:40;
	struct {
		/* Indicates if the L4 checksum is bad or good */
		volatile uint32_t hw_l4_chksum_bad:1;
		/* Indicates if L4 checksum is found or not */
		volatile uint32_t hw_l4_chksum_valid:1;
		/* Indicates if the L3 checksum is bad or good */
		volatile uint32_t hw_l3_chksum_bad:1;
		/* Indicates if the L3 checksum is found or not */
		volatile uint32_t hw_l3_chksum_valid:1;
		/* Unused by H/W (set to 0) as length of the header
		 * is not required to calculate L4 checksum
		 */
		volatile uint32_t l4_hdr_dw:4;
		volatile uint32_t l4_offset:6;
		volatile uint32_t l3_hdr_dw:4;
		volatile uint32_t l3_offset:6;
		volatile uint32_t is_ipfrag:1;
		volatile uint32_t is_udp:1;
		volatile uint32_t is_tcp:1;
		volatile uint32_t is_ipv6:1;
		volatile uint32_t is_ipv4:1;
		volatile uint32_t is_vlan:1;
		volatile uint32_t reserved:10;
	};
};

 /**
  * C2H Completion entry structure
  * This structure is specific for the example design.
  * Processing of this ring happens in qdma_rxtx.c.
  */
struct __attribute__ ((packed)) qdma_ul_st_cmpt_ring
{
	volatile uint32_t	data_frmt:1; /* For 2018.2 IP, this field
					      * determines the Standard or User
					      * format of completion entry
					      */
	volatile uint32_t	color:1;     /* This field inverts every time
					      * PIDX wraps the completion ring
					      */
	volatile uint32_t	err:1;       /* Indicates that C2H engine
					      * encountered a descriptor
					      * error
					      */
	volatile uint32_t	desc_used:1;   /* Indicates that the completion
						* packet consumes descriptor in
						* C2H ring
						*/
	volatile uint32_t	length:16;     /* Indicates length of the data
						* packet
						*/
	volatile uint32_t	rsvd:3;    /* Reserved field */
	volatile uint32_t	user_err:1;    /* User error field */
	union c2h_metadata	metadata; /* User fields */
};


 /**
  * Completion entry structure
  * This structure is specific for the example design.
  * Currently this structure is used for the processing
  * of the MM completion ring in rte_pmd_qdma.c.
  */
struct __attribute__ ((packed)) qdma_ul_cmpt_ring
{
	volatile uint32_t	data_frmt:1; /* For 2018.2 IP, this field
					      * determines the Standard or User
					      * format of completion entry
					      */
	volatile uint32_t	color:1;     /* This field inverts every time
					      * PIDX wraps the completion ring
					      */
	volatile uint32_t	err:1;       /* Indicates that C2H engine
					      * encountered a descriptor
					      * error
					      */
	volatile uint32_t	rsv:1;   /* Reserved */
	volatile uint8_t	user_def[];    /* User logic defined data of
						* length based on CMPT entry
						* length
						*/
};

/** ST C2H Descriptor **/
struct __attribute__ ((packed)) qdma_ul_st_c2h_desc
{
	volatile uint64_t	dst_addr;
};

#define S_H2C_DESC_F_SOP		1
#define S_H2C_DESC_F_EOP		2

/**
 * @union - h2c_metadata
 * @brief	h2c descriptor metadata
 */
union h2c_metadata {
	volatile uint32_t user;
	struct {
		/* L3 start offset from L2 in bytes */
		volatile uint32_t l3_hdr_off:7;
		/* Generate L3 checksum */
		volatile uint32_t l3_cksum_gen:1;
		/* L4 start offset from L2 in bytes */
		volatile uint32_t l4_hdr_off:7;
		/* Generate L4 checksum */
		volatile uint32_t l4_cksum_gen:1;
		/* L3 optional header length in words */
		volatile uint32_t l3_opt_dw:2;
		volatile uint32_t l4_len:4; /* Unused. Set to 0 */
		volatile uint32_t udp:1; /* 0 - TCP, 1 - UDP */
		volatile uint32_t rsvd:9;
	};
};

/* pld_len and flags members are part of custom descriptor format needed
 * by example design for ST loopback and desc bypass
 */

/** ST H2C Descriptor **/
struct __attribute__ ((packed)) qdma_ul_st_h2c_desc
{
	volatile union h2c_metadata	metadata;
	volatile uint16_t	len;
	volatile uint16_t	flags:5;
	volatile uint16_t	gl:5;
	volatile uint16_t	rsvd:6;
	volatile uint64_t	src_addr;
};

/** MM Descriptor **/
struct __attribute__ ((packed)) qdma_ul_mm_desc
{
	volatile uint64_t	src_addr;
	volatile uint64_t	len:28;
	volatile uint64_t	dv:1;
	volatile uint64_t	sop:1;
	volatile uint64_t	eop:1;
	volatile uint64_t	rsvd:33;
	volatile uint64_t	dst_addr;
	volatile uint64_t	rsvd2;
};

/**
 * Extract the fields of given completion entry in the completion ring.
 *
 * @param ul_cmpt_entry
 *   Pointer to completion entry to be extracted.
 * @param cmpt_info
 *   Pointer to structure to which completion entry details needs to be filled.
 * @param cmpt_desc_len
 *   Completion entry length.
 *
 * @return
 *   0 on success and -ve on error.
 */
int qdma_ul_extract_st_cmpt_info(void *ul_cmpt_entry,
		void *cmpt_info, uint16_t cmpt_desc_len);

/**
 * Extract the packet length from the given completion entry.
 *
 * @param ul_cmpt_entry
 *   Pointer to completion entry to be extracted.
 * @param pkt_len
 *   Pointer filled with packet length.
 *
 * @return
 *   0 on success and -ve on error.
 */
int qdma_ul_get_cmpt_pkt_len(void *ul_cmpt_entry, uint16_t *pkt_len);

/**
 * Update mbuf flags based on the completion information of the packet.
 *
 * @param qhndl
 *   Pointer to RX queue handle.
 * @param cmpt_info
 *   Pointer to structure in which completion entry details are filled.
 * @param q_offloads
 *   Offloads supported for the queue.
 * @param mb
 *   Pointer to rte_mbuf structure representing packet information.
 *
 * @return
 *   None.
 */
void qdma_update_st_c2h_mbuf_flags(void *qhndl,
					void *cmpt_info,
					uint64_t q_offloads,
					struct rte_mbuf *mb);

/**
 * Processes the immediate data for the given completion ring entry
 * and stores the immediate data in a file.
 *
 * @param qhndl
 *   Pointer to RX queue handle.
 * @param cmpt_entry
 *   Pointer to completion entry to be processed.
 * @param cmpt_desc_len
 *   Completion descriptor length.
 *
 * @return
 *   None.
 */
int qdma_ul_process_immediate_data_st(void *qhndl, void *cmpt_entry,
			uint16_t cmpt_desc_len);

/**
 * Updates the ST c2h descriptor.
 *
 * @param mb
 *   Pointer to memory buffer.
 * @param desc
 *   Pointer to descriptor entry.
 *
 * @return
 *   None.
 */
int qdma_ul_update_st_c2h_desc(struct rte_mbuf *mb, void *desc);

/**
 * Updates the ST H2C descriptor
 *
 * @param qhndl
 *   Pointer to TX queue handle.
 * @param q_offloads
 *   Offloads supported for the queue.
 * @param mb
 *   Pointer to memory buffer.
 *
 * @return
 *   None.
 */
int qdma_ul_update_st_h2c_desc(void *qhndl, uint64_t q_offloads,
				struct rte_mbuf *mb);

/**
 * Updates the MM c2h descriptor.
 *
 * @param qhndl
 *   Pointer to RX queue handle.
 * @param mb
 *   Pointer to memory buffer.
 * @param desc
 *   Pointer to descriptor entry.
 *
 * @return
 *   None.
 */
int qdma_ul_update_mm_c2h_desc(void *qhndl, struct rte_mbuf *mb, void *desc);

/**
 * updates the MM h2c descriptor.
 *
 * @param qhndl
 *   Pointer to TX queue handle.
 * @param mb
 *   Pointer to memory buffer.
 *
 * @return
 *   None.
 */
int qdma_ul_update_mm_h2c_desc(void *qhndl, struct rte_mbuf *mb);

/**
 * Processes the completion data from the given completion entry.
 *
 * @param cmpt_entry
 *   Pointer to completion entry to be processed.
 * @param cmpt_desc_len
 *   Completion descriptor length.
 * @param cmpt_buff
 *   Pointer to the data buffer to which the data will be extracted.
 *
 * @return
 *   None.
 */
int qdma_ul_process_immediate_data(void *cmpt_entry, uint16_t cmpt_desc_len,
			char *cmpt_buff);

#endif /* ifndef __QDMA_USER_H__ */
