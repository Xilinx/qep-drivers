/*
 * Copyright(c) 2015-2019 Xilinx, Inc. All rights reserved.
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

#ifndef XCMAC_H
#define XCMAC_H

/* standard header files */
#ifndef __KERNEL__
#include <stdint.h>
#include <stdbool.h>
#endif
#include <linux/types.h>
#include <linux/errno.h>
#include "xcmac_hw.h"

/**< component has been initialized */
#define XIL_COMPONENT_IS_READY 0x11111111
/**< component has been started */
#define XIL_COMPONENT_IS_STARTED 0x22222222
/**< Half word mask */
#define HALF_WORD_MASK 0xFFFF
/**< Macro to indicate 16 bits */
#define SIXTEEN_BITS 16
/**< 16 LSB bits mask */
#define TWO_LSB_MASK 0xFFFF
/**< 16 MSB bits mask */
#define TWO_MSB_MASK 0xFFFF0000
/**< Number of bytes in a WORD */
#define NUM_WORD_BYTES 4
/**< Number of bytes in a DWORD */
#define NUM_DWORD_BYTES 8
/**< Number of bits in a WORD */
#define NUM_WORD_BITS 32
/**< Total refresh timers in the CMAC IP */
#define TOTAL_TX_PAUSE_REFRESH_TIMERS 8
/** Total Pause Refresh Timers */
#define TOTAL_PAUSE_REFRESH_TIMERS 8
/** Total Pause Quantas */
#define TOTAL_PAUSE_QUANTAS 8
/**< WORD mask */
#define WORD_MASK 0xffffffff
/**< XCMAC_PCS_LANE_COUNT specifies the count of PCS lanes. */
#define XCMAC_PCS_LANE_COUNT (20)

#ifdef DEBUG
#define DEBUG_PRINT printf
#else
#define DEBUG_PRINT(...)
#endif

/**
 * This enum represents CMAC core
 */
enum xcmac_core_type {
	XCMAC_CORE_RX, /*!< Indicates Rx path */
	XCMAC_CORE_TX, /*!< Indicates Tx path */
	XCMAC_CORE_BOTH, /*!< Indicates Rx and Tx path */
};

/**
 * This enum represents CMAC CAUI mode
 */
enum xcmac_caui_mode {
	XCMAC_CAUI_MODE_10 = 0, /*!< Indicates CAUI-10 mode */
	XCMAC_CAUI_MODE_4 = 1, /*!< Indicates CAUI-4 mode */
	XCMAC_RUNTIME_SWITCHABLE_CAUI_MODE_10 = 2,
	/*!< RO Indicates Runtime Switchable CAUI-10 mode */
	XCMAC_RUNTIME_SWITCHABLE_CAUI_MODE_4 = 3,
	/*!< RO Indicates Runtime Switchable CAUI-4 mode */
	XCMAC_CAUI_INVALID,
};

/**
 * This enum represents flow control packet type
 */
enum xcmac_flow_control_packet_type {
	XCMAC_FC_GLOBAL_CONTROL_PACKET, /*!< Indicates global control packet */
	XCMAC_FC_PRIORITY_CONTROL_PACKET,
	/*!< Indicates priority control packet */
	XCMAC_FC_GLOBAL_PAUSE_PACKET, /*!< Indicates global pause packet */
	XCMAC_FC_PRIORITY_PAUSE_PACKET, /*!< Indicates priority pause packet */
};

/**
 * This enum represents GT loopback type
 */
enum xcmac_gt_loopback_type {
	XCMAC_GT_LOOPBACK_NORMAL,
	/*!< Indicates normal operation (External Loopback) */
	XCMAC_GT_LOOPBACK_INTERNAL,
	/*!< Indicates near end PMA internal Loopback */
	XCMAC_GT_LOOPBACK_INVALID,
};

/**
 * @brief Structure for CMAC driver instance specific variables.
 *
 */
struct xcmac {
	/**< PCIe BAR in which CMAC IP is present */
	uint64_t base_address;
	/**< Indicates device is initialized and ready */
	uint32_t is_ready;
	/**< Indicates device has been started */
	uint32_t is_started;
	/**< Global variable to hold the PCI Bar Data details */
};

/**
 * @brief Structure for fault indication specific variables.
 *
 */
struct xcmac_fault_indication {
	/**< Send Remote Fault Indication on Tx */
	uint8_t tx_send_rfi;
};

/**
 * @brief Structure for Bip7 specific variables.
 */
struct xcmac_bip7_config {
	/**< bip7 feature enable/disable */
	uint8_t bip7_valid;
	/**< 8-bit bip7 value */
	uint8_t bip7_value;
};

/*
 * @brif Structure for Tx OTN status variables.
 */
struct xcmac_tx_otn_status {
	uint8_t remote_fault;
	uint8_t internal_local_fault;
	uint8_t received_local_fault;
	uint8_t test_pattern_mismatch;
	uint8_t bad_preamble;
	uint8_t bad_sfd;
	uint8_t got_signal_os;
};

/*
 * @brif Structure for Tx packet length variables.
 */
struct xcmac_tx_otn_pkt_length {
	uint8_t min_pkt_length;
	uint16_t max_pkt_length;
};

/*
 * @brif Structure for Tx OTN control configuration variables.
 */
struct xcmac_tx_otn_ctl_config {
	uint8_t check_sfd;
	uint8_t check_preamble;
	uint8_t ignore_fcs;
};

/**
 * @brief Structure for Tx Flow control specific variables.
 */
struct xcmac_tx_flow_control {
	/**< Pause packets enable/disable */
	uint16_t pause_enable;
	/**< Refresh interval to send pause packet */
	uint16_t refresh_interval[TOTAL_PAUSE_REFRESH_TIMERS];
	/**< Pause quanta */
	uint16_t pause_quanta[TOTAL_PAUSE_QUANTAS];
};

/**
 * @brief Structure for Rx Flow control specific variables.
 */
struct xcmac_rx_flow_control {
	/**< Packet processing enable/disable */
	uint8_t enable;
	/**< Multicast destination address processing */
	uint8_t check_multicast;
	/**< Unicast destination address processing */
	uint8_t check_unicast;
	/**< Enable/Disable source address processing */
	uint8_t check_source_address;
	/**< Enable/Disable Ethertype processing */
	uint8_t check_ether_type;
	/**< Enable/Disable opcode processing */
	uint8_t check_opcode;
};

/**
 * @brief Structure for Tx status specific variables.
 * PTP - Precise Timing Protocol
 */
struct xcmac_tx_status {
	/**< Tx Local Fault status */
	uint8_t local_fault;
	/**< Tx PTP FIFO read error */
	uint8_t ptp_fifo_read_error;
	/**< Tx PTP FIFO write error */
	uint8_t ptp_fifo_write_error;
};

/**
 * @brief Structure for Rx lane status.
 */
struct xcmac_rx_lane_am_status {
	/**< PCS status */
	uint8_t pcs_status;
	/**< All PCS aligned/de-skewed status */
	uint8_t aligned;
	/**< Alignment Error */
	uint8_t mis_aligned;
	/**< Loss of lane alignment/De-Skew status */
	uint8_t aligned_error;
	/**< PCS block lock status */
	uint8_t block_lock[XCMAC_PCS_LANE_COUNT];
	/**< Word boundary sync status */
	uint8_t synced[XCMAC_PCS_LANE_COUNT];
	/**< Sync error status */
	uint8_t synced_error[XCMAC_PCS_LANE_COUNT];
	/**< Lane marker error status */
	uint8_t lane_marker_error[XCMAC_PCS_LANE_COUNT];
	/**< Lane marker length error status */
	uint8_t lane_marker_len_error[XCMAC_PCS_LANE_COUNT];
	/**< Consecutive lane marker error status */
	uint8_t lane_marker_repeat_error[XCMAC_PCS_LANE_COUNT];
};

/**
 * @brief Structure for  Rx Fault status.
 */
struct xcmac_rx_fault_status {
	/**< remote fault indication */
	uint8_t remote_fault;
	/**< local fault indication */
	uint8_t local_fault;
	/**< internal local fault indication */
	uint8_t internal_local_fault;
	/**< received local fault indication */
	uint8_t received_local_fault;
};

/**
 * @brief Structure for Rx Ethernet data status.
 */
struct xcmac_rx_packet_status {
	/**< High Bit error rate indication */
	uint8_t hi_ber;
	/**< Bad Preamble indication */
	uint8_t bad_preamble;
	/**< Invalid Start of frame delimiter indication */
	uint8_t bad_sfd;
	/**< Signal Ordered Sets indication */
	uint8_t got_signal_os;
};

/**
 * @brief Structure for Rx Virtual lane Demux status.
 */
struct xcmac_rx_vldemux_status {
	/**< pcs lane demultiplex status */
	uint8_t vldemuxed[XCMAC_PCS_LANE_COUNT];
	/**< pcs lane mapping, 5 lines per 20 lanes */
	uint8_t vlnumber[XCMAC_PCS_LANE_COUNT];
};

/**
 * @brief Structure for packets histogram.
 */
struct xcmac_packet_count_statistics {
	/**< Total number of packets */
	uint64_t total_packets;
	/**< Total number of good packets */
	uint64_t total_good_packets;
	/**< Total number of bytes */
	uint64_t total_bytes;
	/**< Total number of good bytes */
	uint64_t total_good_bytes;
	/**< Total packets of size 64 bytes */
	uint64_t packets_0_to_64_bytes;
	/**< packets size between 64 & 127 bytes */
	uint64_t packets_65_to_127_bytes;
	/**< packets size between 128 & 255 bytes */
	uint64_t packets_128_to_255_bytes;
	/**< packets size between 256 & 511 bytes */
	uint64_t packets_256_to_511_bytes;
	/**< packets size between 512 & 1023 bytes */
	uint64_t packets_512_to_1023_bytes;
	/**< packets size between 1024 & 1518 bytes */
	uint64_t packets_1024_to_1518_bytes;
	/**< packets size between 1519 & 1522 bytes */
	uint64_t packets_1519_to_1522_bytes;
	/**< packets size between 1523 & 1548 bytes */
	uint64_t packets_1523_to_1548_bytes;
	/**< packets size between 1549 & 2047 bytes */
	uint64_t packets_1549_to_2047_bytes;
	/**< packets size between 2048 & 4095 bytes */
	uint64_t packets_2048_to_4095_bytes;
	/**< packets size between 4096 & 8191 bytes */
	uint64_t packets_4096_to_8191_bytes;
	/**< packets size between 8192 & 9215 bytes */
	uint64_t packets_8192_to_9215_bytes;
	/**< packets size less than 64 bytes */
	uint64_t packets_small;
	/**< packets size more than 9215 bytes */
	uint64_t packets_large;
};

/**
 * @brief Structure for packet type statistics.
 */
struct xcmac_packet_type_statistics {
	/**< Unicast packets count */
	uint64_t packets_unicast;
	/**< Multicast packets count */
	uint64_t packets_multicast;
	/**< Broadcast packets count */
	uint64_t packets_broadcast;
	/**< VLAN packets count */
	uint64_t packets_vlan;
	/**< Pause packets with good FCS */
	uint64_t packets_pause;
	/**< Priority pause packets with good FCS */
	uint64_t packets_priority_pause;
};

/**
 * @brief Structure for Malformed packets statistics.
 */
struct xcmac_malformed_statistics {
	/**< packets less than min packet length with good FCS */
	uint64_t under_size_count;
	/**< packets less than min packet length with bad FCS */
	uint64_t fragment_count;
	/**< packets longer than max packet length with good FCS */
	uint64_t oversize_count;
	/**< packets longer than max packet length with good and bad FCS */
	uint64_t toolong_count;
	/**< packets longer than max packet length with bad FCS */
	uint64_t jabber_count;
	/**< crc32 error */
	uint64_t bad_fcs_count;
	/**< packets between 64 bytes and max packet length with FCS errors */
	uint64_t packet_bad_fcs_count;
	/**< packets with bitwise inverse of expected good FCS */
	uint64_t stomped_fcs_count;
};

/**
 * @brief Structure for Auto-negotiation status
 */
struct xcmac_autoneg_status {
	uint8_t fec_enable;
	/**< Fec Enable status */
	uint8_t rs_fec_enable;
	/**< RS-FEC Enable status */
	uint8_t autoneg_complete;
	/**< Auto-negotiation completion status */
	uint8_t parallel_detection_fault;
	/**< Parallel  fault status detection during auto-negotiation */
	uint8_t tx_pause_enable;
	/**< Used to enable station-to-station (global) pause packet generation
	 * in the transmit path to control data flow in the receive path
	 */
	uint8_t rx_pause_enable;
	/**<  Used to enable station-to- (global) pause packet interpretation
	 * in the receive path, to control data flow from the transmitter.
	 */
	uint8_t lp_ability_valid;
	/**< This signal indicates when all of the
	 * link partner advertisements become valid
	 */
	uint8_t lp_autoneg_able;
	/**< Indicates link partner is able to perform autonegotiation */
	uint8_t lp_pause;
	/**< This signal indicates the advertised value of the PAUSE bit,
	 * (C0), in the receive link codeword from the link partner
	 */
	uint8_t lp_asm_dir;
	/**< This signal indicates the advertised value of the ASMDIR bit,
	 * (C1), in the receive link codeword from the link partner
	 */
	uint8_t lp_rf;
	/**< This bit indicates link partner remote fault. */
	uint8_t lp_fec_10g_ability;
	/**< This signal indicates the clause 74 FEC ability associated
	 *  with 10Gb/s laneprotocols being advertised by the link partner.
	 *  valid when the output signal stat_an_lp_ability_valid is asserted
	 */
	uint8_t lp_fec_10g_request;
	/**< This signal indicates that the link partner is requesting that
	 * the clause 74 FEC be used on the 10Gb/s lane protocols.
	 * valid when the output signal stat_an_lp_ability_valid is asserted
	 */
	uint8_t lp_ext_ability_valid;
	/**< Indicates that the detected extended abilities are valid. */
	uint8_t lp_ability_ext_fec;
	/**< indicates the extended FEC abilities as defined */
	uint8_t lp_fec_25g_rs_ability;
	/**< This signal indicates that the link partner is requesting
	 * the clause 91 (or 108) rs FEC be used for the 25gb/s lane protocols.
	 * It becomes valid when the output signal
	 * stat_an_lp_ability_valid is asserted.
	 */
	uint8_t lp_fec_25g_baser_request;
	/**< This signal indicates that the link partner is requesting
	 * the clause 74 FEC be used for the 25Gb/s lane base-r protocols.
	 * It becomes valid when the output signal
	 * stat_an_lp_ability_valid is asserted.
	 */
};

/**
 * @brief Structure for Auto negotiation control
 */
struct xcmac_autoneg_config {
	uint8_t enable;
	/**< Enable signal for autonegotiation */
	uint8_t bypass;
	/**< Input to disable autonegotiation
	 * and bypass the autonegotiation function
	 */
	uint16_t nonce_seed;
	/**< Seed to initialize the nonce field Polynomial generator */
	uint8_t pseudo_sel;
	/**< Selects the polynomial generator for the bit 49 random bit
	 * generator depending on 0 or 1
	 */
	uint8_t restart;
	/**< Trigger a restart of the auto negotiation,
	 * regardless of what state the circuit is currently in
	 */
	uint8_t local_fault;
	/**< Used to set the local_fault bit of the transmit link codeword */
	uint8_t pause;
	/**< This input signal is used to set the 'PAUSE' bit, (C0),
	 * of the transmit link codeword.
	 */
	uint8_t asmdir;
	/**< This input signal is used to set the 'ASMDIR bit', (C1),
	 * of the transmit link codeword.
	 */
	uint8_t cl91_fec_request;
	/**< Used to request clause 91 FEC. */
	uint8_t cl91_fec_ability;
	/**< Used to indicate clause 91 FEC ability */
	uint8_t fec_25g_rs_request;
	/**< Used to indicate clause 91 FEC ability */
	uint8_t loc_np;
	/** < Local Next Page indicator */
	uint8_t lp_np_ack;
	/** < Link Partner Next Page Acknowledge */
};

/**
 * @brief Structure for Auto negotiation ability.
 *   These inputs identify the Ethernet
 *    protocol abilities that is advertised
 *    in the transmit link codeword to
 *    the link partner. A value of 1
 *    indicates that the interface
 *    advertises that it supports the
 *    protocol
 */
struct xcmac_autoneg_ability {
	uint8_t ctl_autoneg_ability_1000base_kx;
	uint8_t ctl_autoneg_ability_10gbase_kx4;
	uint8_t ctl_autoneg_ability_10gbase_kr;
	uint8_t ctl_autoneg_ability_40gbase_kr4;
	uint8_t ctl_autoneg_ability_40gbase_cr4;
	uint8_t ctl_autoneg_ability_100gbase_cr10;
	uint8_t ctl_autoneg_ability_100gbase_kp4;
	uint8_t ctl_autoneg_ability_100gbase_kr4;
	uint8_t ctl_autoneg_ability_100gbase_cr4;
	uint8_t ctl_autoneg_ability_25gbase_kr;
	uint8_t ctl_autoneg_ability_25gbase_cr;
	uint8_t ctl_autoneg_ability_25gbase_kr1;
	uint8_t ctl_autoneg_ability_25gbase_cr1;
	uint8_t ctl_autoneg_ability_50gbase_kr2;
	uint8_t ctl_autoneg_ability_50gbase_cr2;
};

/**
 * @brief Structure for Auto negotiation control Status.
 *  These agreed identify the Ethernet
 *  protocol cntro status
 */
struct xcmac_autoneg_link_ctrl {
	/** These agreed identify the Ethernet protocol control status */
	uint8_t stat_an_link_cntl_1000base_kx;
	uint8_t stat_an_link_cntl_10gbase_kx4;
	uint8_t stat_an_link_cntl_10gbase_kr;
	uint8_t stat_an_link_cntl_40gbase_kr4;
	uint8_t stat_an_link_cntl_40gbase_cr4;
	uint8_t stat_an_link_cntl_100gbase_cr10;
	uint8_t stat_an_link_cntl_100gbase_kp4;
	uint8_t stat_an_link_cntl_100gbase_kr4;
	uint8_t stat_an_link_cntl_100gbase_cr4;
	uint8_t stat_an_link_cntl_25gbase_krcr_s;
	uint8_t stat_an_link_cntl_25gbase_krcr;
	uint8_t stat_an_link_cntl_25gbase_kr1;
	uint8_t stat_an_link_cntl_25gbase_cr1;
	uint8_t stat_an_link_cntl_50gbase_kr2;
	uint8_t stat_an_link_cntl_50gbase_cr2;
};

/**
 * @brief Structure for Link Training configuration
 */
struct xcmac_lt_config {
	uint8_t ctl_lt_training_enable;
	/**< Indicates enabling LT */
	uint8_t ctl_lt_restart_training;
	/**< Indicates restarting LT */
	uint8_t ctl_lt_rx_trained;
	/**< Indicates if LT is done */
	uint8_t ctl_lt_preset_to_tx;
	/**< Indicates presets for Tx */
	uint8_t ctl_lt_initialize_to_tx;
	/**< Indicates inialize to Tx */
	uint16_t ctl_lt_pseudo_seed0;
	/**< Provoides seeds for training sequence generator */
	uint16_t ctl_lt_pseudo_seed1;
	/**< Provoides seeds for training sequence generator */
	uint16_t ctl_lt_pseudo_seed2;
	/**< Provoides seeds for training sequence generator */
	uint16_t ctl_lt_pseudo_seed3;
	/**< Provoides seeds for training sequence generator */
};

/**
 * @brief Structure for holding LT coefficients
 */
struct xcmac_link_training_coefficients {
	uint8_t k_p1_to_tx0;
	uint8_t k0_to_tx0;
	uint8_t k_m1_to_tx0;
	uint8_t stat_p1_to_tx0;
	uint8_t stat0_to_tx0;
	uint8_t stat_m1_to_tx0;
	uint8_t k_p1_to_tx1;
	uint8_t k0_to_tx1;
	uint8_t k_m1_to_tx1;
	uint8_t stat_p1_to_tx1;
	uint8_t stat0_to_tx1;
	uint8_t stat_m1_to_tx1;

	uint8_t k_p1_to_tx2;
	uint8_t k0_to_tx2;
	uint8_t k_m1_to_tx2;
	uint8_t stat_p1_to_tx2;
	uint8_t stat0_to_tx2;
	uint8_t stat_m1_to_tx2;
	uint8_t k_p1_to_tx3;
	uint8_t k0_to_tx3;
	uint8_t k_m1_to_tx3;
	uint8_t stat_p1_to_tx3;
	uint8_t stat0_to_tx3;
	uint8_t stat_m1_to_tx3;
};

/**
 * @brief Structure for Link Training Status.
 */
struct xcmac_link_training_status {
	uint8_t link_training_init;
	/**< Initialize LT */
	uint8_t link_training_preset;
	/**< LT preset status */
	uint8_t link_training_training;
	/**< LT training status */
	uint8_t link_training_frame_lock;
	/**< LT frame lock status */
	uint8_t link_training_signal_detect;
	/**< LT signal detection status */
	uint8_t link_training_training_fail;
	/**< LT training faile status */
	uint8_t link_training_rx_sof;
	/**< LT start of link training frame */
};

/** Function macro to left shift 1 by x times */
#define bit(x) (0x1 << x)
/** Function macro that sets a bit in the given value
 *  If user wants to set a bit num 0 in bits [31:0], then pass Pos as 0
 */
#define set_bit(val, pos) (val |= bit(pos))
/** Function macro that clears a bit in the given value
 *  If user wants to clear a bit num 0 in bits [31:0], then pass pos as 0
 */
#define clear_bit(val, pos) (val &= ~bit(pos))
/** Function macro to check if the argument is NULL */
#define is_null(arg)                                                           \
	do { if (!(arg))                                                       \
		return -EINVAL; } while (0)
/** Function macro that  that return the value at a position
 *  If user wants to check a bit at num 0 in bits [31:0], then pass pos as 0
 */
#define get_bit_status(val, pos) ((val & bit(pos)) ? 1 : 0)

/************************** Function Prototypes ******************************/
void xcmac_out32(uint64_t address, uint32_t data);
uint32_t xcmac_in32(uint64_t address);

int xcmac_initialize(struct xcmac *instance, uint64_t cmac_va_base);
int xcmac_reset_gt(struct xcmac *instance);
int xcmac_reset_core(struct xcmac *instance, enum xcmac_core_type core);
uint32_t xcmac_get_version(struct xcmac *intance);
int xcmac_enable(struct xcmac *instance, enum xcmac_core_type core);
int xcmac_disable(struct xcmac *instance, enum xcmac_core_type core);
int xcmac_get_device_config(struct xcmac *instance, uint8_t *rx_en_status,
			    uint8_t *tx_en_status);
int xcmac_set_caui_mode(struct xcmac *instance, enum xcmac_caui_mode caui_mode);
enum xcmac_caui_mode xcmac_get_caui_mode(struct xcmac *instance);
enum xcmac_caui_mode xcmac_get_core_mode(struct xcmac *instance);
int xcmac_set_fault_indication(struct xcmac *instance, uint8_t tx_send_rfi);
uint8_t xcmac_get_fault_indication(struct xcmac *instance);
int xcmac_send_tx_idle(struct xcmac *instance);
uint8_t xcmac_get_tx_idle_status(struct xcmac *instance);
int xcmac_enable_test_pattern(struct xcmac *instance,
			      enum xcmac_core_type core);
int xcmac_disable_test_pattern(struct xcmac *instance,
			       enum xcmac_core_type core);
uint8_t xcmac_get_test_pattern_status(struct xcmac *instance,
				      enum xcmac_core_type core);
int xcmac_force_rx_resync(struct xcmac *instance);
int xcmac_set_tx_bip7_config(struct xcmac *instance,
			     struct xcmac_bip7_config *bip7_config);
int xcmac_get_bip7_config(struct xcmac *instance,
			  struct xcmac_bip7_config *bip7_config);
int xcmac_get_otn_status(struct xcmac *instance,
			 struct xcmac_tx_otn_status *otn_status);
int xcmac_set_otn_pkt_length(struct xcmac *instance,
			     struct xcmac_tx_otn_pkt_length *pkt_len);
int xcmac_get_otn_pkt_length(struct xcmac *instance,
			     struct xcmac_tx_otn_pkt_length *pkt_len);
int xcmac_set_otn_control_config(struct xcmac *instance,
				 struct xcmac_tx_otn_ctl_config *otn_ctl_cfg);
int xcmac_get_otn_control_config(struct xcmac *instance,
				 struct xcmac_tx_otn_ctl_config *otn_ctl_cfg);
int xcmac_set_gt_loopback_config(struct xcmac *instance,
				 enum xcmac_gt_loopback_type loopback_type);
enum xcmac_gt_loopback_type
xcmac_get_gt_loopback_config(struct xcmac *instance);

int xcmac_set_tx_flow_control_config(
	struct xcmac *instance, struct xcmac_tx_flow_control *flow_control);
int xcmac_get_tx_flow_control_config(
	struct xcmac *instance, struct xcmac_tx_flow_control *flow_control);
int xcmac_set_rx_flow_control_pause_enable(struct xcmac *instance,
					   uint16_t enable);
uint16_t xcmac_get_rx_flow_control_pause_enable(struct xcmac *instance);
int xcmac_set_rx_flow_control_pause_ack(struct xcmac *instance, uint16_t ack);
uint16_t xcmac_get_rx_flow_control_pause_ack(struct xcmac *instance);
int xcmac_set_rx_flow_control_packet_config(
	struct xcmac *instance, enum xcmac_flow_control_packet_type packet,
	struct xcmac_rx_flow_control *flow_control);
int xcmac_get_rx_flow_control_packet_config(
	struct xcmac *instance, enum xcmac_flow_control_packet_type packet,
	struct xcmac_rx_flow_control *flow_control);

int xcmac_get_tx_status(struct xcmac *instance,
			struct xcmac_tx_status *tx_status);
int xcmac_get_rx_lane_status(struct xcmac *instance,
			     struct xcmac_rx_lane_am_status *rx_lane_status);
int xcmac_get_rx_fault_status(struct xcmac *instance,
			      struct xcmac_rx_fault_status *rx_fault_status);
int xcmac_get_rx_packet_status(struct xcmac *instance,
			       struct xcmac_rx_packet_status *rx_packet_status);
int xcmac_get_rx_vldemux_status(
	struct xcmac *instance,
	struct xcmac_rx_vldemux_status *rx_vldemux_status);
uint8_t xcmac_get_rx_test_pattern_status(struct xcmac *instance);
int xcmac_latch_statistics(struct xcmac *instance);
uint64_t xcmac_get_cycle_count(struct xcmac *instance);
int xcmac_get_rx_bip_error_count(
	struct xcmac *instance,
	uint64_t xcmac_bip8_error[XCMAC_PCS_LANE_COUNT]);
int xcmac_get_rx_framing_error_count(
	struct xcmac *instance,
	uint64_t framing_err_count[XCMAC_PCS_LANE_COUNT]);
uint64_t xcmac_get_rx_bad_64b66b_code_count(struct xcmac *instance);
uint64_t xcmac_get_tx_bad_64b66b_code_count(struct xcmac *instance);
uint64_t xcmac_get_tx_frame_error_count(struct xcmac *instance);
int xcmac_get_tx_packet_statistics(
	struct xcmac *instance,
	struct xcmac_packet_count_statistics *packet_stats);
int xcmac_get_rx_packet_statistics(
	struct xcmac *instance,
	struct xcmac_packet_count_statistics *packet_stats);
int xcmac_get_tx_malformed_statistics(
	struct xcmac *instance,
	struct xcmac_malformed_statistics *malformed_stats);
int xcmac_get_rx_malformed_statistics(
	struct xcmac *instance,
	struct xcmac_malformed_statistics *malformed_stats);
int xcmac_get_tx_packet_type_statistics(
	struct xcmac *instance,
	struct xcmac_packet_type_statistics *packet_type_stats);
int xcmac_get_rx_packet_type_statistics(
	struct xcmac *instance,
	struct xcmac_packet_type_statistics *packet_type_stats);
uint64_t xcmac_get_rx_range_error_count(struct xcmac *instance);
uint64_t xcmac_get_rx_truncated_count(struct xcmac *instance);
int xcmac_deintialize(struct xcmac *instance);

int xcmac_get_autoneg_status(struct xcmac *instance,
			     struct xcmac_autoneg_status *status);
int xcmac_get_autoneg_ability(struct xcmac *instance,
			      struct xcmac_autoneg_ability *ability);
int xcmac_get_autoneg_link_ctrl(struct xcmac *instance,
				struct xcmac_autoneg_link_ctrl *an_link_cntl);
int xcmac_get_link_training_status(struct xcmac *instance,
				   struct xcmac_link_training_status *status);

int xcmac_config_autoneg(struct xcmac *instance,
			 struct xcmac_autoneg_config *config);
int xcmac_get_config_autoneg(struct xcmac *instance,
			     struct xcmac_autoneg_config *config);
int xcmac_config_autoneg_ability(struct xcmac *instance,
				 struct xcmac_autoneg_ability *autoneg_ability);
int xcmac_get_config_autoneg_ability(
	struct xcmac *instance, struct xcmac_autoneg_ability *autoneg_ability);
int xcmac_config_link_training(struct xcmac *instance,
			       struct xcmac_lt_config *config);
int xcmac_get_config_link_training(struct xcmac *instance,
				   struct xcmac_lt_config *config);
int xcmac_config_link_training_coefficient(
	struct xcmac *instance,
	struct xcmac_link_training_coefficients *link_training_coefs);
int xcmac_get_link_training_coeff(
	struct xcmac *instance,
	struct xcmac_link_training_coefficients *coeffs);

int xcmac_config_rsfec(struct xcmac *instance, bool enable);

#endif /* end of protection macro */
