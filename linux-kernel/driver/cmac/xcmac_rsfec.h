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

/* prevent circular inclusions */
#ifndef XCMAC_RSFEC_H
#define XCMAC_RSFEC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The following are the offsets to access RSFEC registers set.
 */
#define XCMAC_RSFEC_CONFIGURATION_OFFSET (0x1000) /**< Configuration Bits */
#define XCMAC_RSFEC_STATUS_OFFSET (0x1004) /**< Status Bits */
#define XCMAC_RSFEC_CORRECT_CW_COUNTER_LSB_OFFSET                              \
	(0x1008) /**< Codeword Correct counter LSB */
#define XCMAC_RSFEC_CORRECT_CW_COUNTER_MSB_OFFSET                              \
	(0x100C) /**< Codeword Correct counter MSB */
#define XCMAC_RSFEC_UNCORRECT_CW_COUNTER_LSB_OFFSET                            \
	(0x1010) /**< Codeword UnCorrect counter LSB */
#define XCMAC_RSFEC_UNCORRECT_CW_COUNTER_MSB_OFFSET                            \
	(0x1014) /**< Codeword UnCounter Counter MSB */
#define XCMAC_RSFEC_STATUS_FEC_LINEMAP_OFFSET (0x1018) /**< FEC Lane mapping */
#define XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_0_LSB_OFFSET                          \
	(0x101C) /**< Symbol Error Counter0 LSB */
#define XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_0_MSB_OFFSET                          \
	(0x1020) /**< Symbol Error Counter0 MSB */
#define XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_1_LSB_OFFSET                          \
	(0x1024) /**< Symbol Error Counter1 LSB */
#define XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_1_MSB_OFFSET                          \
	(0x1028) /**< Symbol Error Counter1 MSB */
#define XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_2_LSB_OFFSET                          \
	(0x102C) /**< Symbol Error Counter2 LSB */
#define XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_2_MSB_OFFSET                          \
	(0x1030) /**< Symbol Error Counter2 MSB */
#define XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_3_LSB_OFFSET                          \
	(0x1034) /**< Symbol Error Counter3 LSB */
#define XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_3_MSB_OFFSET                          \
	(0x1038) /**< Symbol Error Counter4 MSB */
#define XCMAC_RSFEC_BLOCK_LOCK_STATUS_OFFSET                                   \
	(0x103C) /**< RSFEC Block Lock status */
#define XCMAC_RSFEC_PCS_AM_LOCK_STATUS_OFFSET                                  \
	(0x1040) /**< PCS AM lock status */
#define XCMAC_RSFEC_RX_LANE_DELAY01_OFFSET (0x1074) /**< Rx Lane Delay01 */
#define XCMAC_RSFEC_RX_LANE_DELAY23_OFFSET (0x1078) /**< Rx Lane Delay23 */
#define XCMAC_RSFEC_BLOCK_BYPASS_OFFSET (0x107C) /**< Block Bypass */
#define XCMAC_RSFEC_RX_RESET_REQUEST_OFFSET                                    \
	(0x1060) /**< Rx path Reset Request */
#define XCMAC_RSFEC_TX_RESET_REQUEST_OFFSET                                    \
	(0x1064) /**< Tx Path Reset Request */

/*
 * The following are the Masks defined based on the bit maps.
 */
#define XCMAC_RSFEC_BYPASS_CORRECTION_ENABLE_MASK (0x01 << 0)
#define XCMAC_RSFEC_BYPASS_INDICATION_ENABLE_MASK (0x01 << 1)
#define XCMAC_RSFEC_ERROR_INDICATION_MODE_MASK (0x01 << 2)

#define XCMAC_RSFEC_BYPASS_CORRECTION_ENABLE_STATUS_MASK (0x01 << 8)
#define XCMAC_RSFEC_BYPASS_INDICATION_ENABLE_STATUS_MASK (0x01 << 9)
#define XCMAC_RSFEC_ERROR_INDICATION_MODE_STATUS_MASK (0x01 << 10)

#define XCMAC_RSFEC_STATUS_BYPASS_CORRECTION_ABILITY_MASK (0x01 << 0)
#define XCMAC_RSFEC_STATUS_BYPASS_INDICATION_ABILITY_MASK (0x01 << 1)
#define XCMAC_RSFEC_STATUS_HI_SER_MASK (0x01 << 2)
#define XCMAC_RSFEC_STATUS_LH_HI_SER_MASK (0x01 << 3)

#define XCMAC_RSFEC_STATUS_FEC_AM_CLOCK0_MASK (0x01 << 8)
#define XCMAC_RSFEC_STATUS_FEC_AM_CLOCK1_MASK (0x01 << 9)
#define XCMAC_RSFEC_STATUS_FEC_AM_CLOCK2_MASK (0x01 << 10)
#define XCMAC_RSFEC_STATUS_FEC_AM_CLOCK3_MASK (0x01 << 11)

#define XCMAC_RSFEC_STATUS_FEC_LANE_ALIGNMENT_MASK (0x01 << 14)
#define XCMAC_RSFEC_STATUS_PCS_LANE_ALIGNMENT_MASK (0x01 << 15)

#define XCMAC_RSFEC_STATUS_FEC_LANE0_MASK (0x03 << 0)
#define XCMAC_RSFEC_STATUS_FEC_LANE1_MASK (0x03 << 2)
#define XCMAC_RSFEC_STATUS_FEC_LANE2_MASK (0x03 << 4)
#define XCMAC_RSFEC_STATUS_FEC_LANE3_MASK (0x03 << 6)

#define XCMAC_RSFEC_RX_LANE_DELAY0_MASK (0x1FFF << 0)
#define XCMAC_RSFEC_RX_LANE_DELAY1_MASK (0x1FFF << 16)
#define XCMAC_RSFEC_RX_LANE_DELAY2_MASK (0x1FFF << 0)
#define XCMAC_RSFEC_RX_LANE_DELAY3_MASK (0x1FFF << 16)

#define XCMAC_RSFEC_BLOCK_BYPASS_MASK (0x01 << 0)
#define XCMAC_RSFEC_RX_BYPASS_STATUS_MASK (0x01 << 8)
#define XCMAC_RSFEC_TX_BYPASS_STATUS_MASK (0x01 << 9)

/**< Rx and Tx Path reset value */
#define RESET_RXTX_PATH_VALUE 0xAA55F0F0

/**************************** Type Definitions *******************************/

/**
 * @brief Enum for RsFec Bypass Configuration.
 *
 */
enum xcmac_rsfec_bypass_type {
	/**< Indicates Bypass Correction */
	RSFEC_CORRECTION,
	/**< Bypass Indication */
	RSFEC_INDICATION,
};

/**
 * @brief Structure for RSFEC AM Lock.
 *
 */
struct xcmac_rsfec_am_lock {
	/**< Rx Lane AM Lock0 */
	uint8_t am_lock0;
	/**< Rx Lane AM Lock1 */
	uint8_t am_lock1;
	/**< Rx Lane AM Lock2 */
	uint8_t am_lock2;
	/**< Rx Lane AM Lock3 */
	uint8_t am_lock3;
};

/**
 * @brief Structure for CW counter
 *
 */
struct xcmac_rsfec_cw_counters {
	/**< FEC Corrected CW Counter */
	uint64_t correct_counter;
	/**< FEC UnCorrected CW Counter */
	uint64_t uncorrect_counter;
};

/**
 * @brief Structure for FEC Lane Mapping for each lane.
 *
 */
struct xcmac_rsfec_lane {
	/**< Lane Mapping for lane0 */
	uint8_t lane0;
	/**< Lane Mapping for lane1 */
	uint8_t lane1;
	/**< Lane Mapping for lane2 */
	uint8_t lane2;
	/**< Lane Mapping for lane3 */
	uint8_t lane3;
};

/**
 * @brief Structure for FEC symbol error counters for 4 lanes ( 0 to 3).
 *
 */
struct xcmac_rsfec_sym_err_counters {
	/**< Error Counter0  */
	uint64_t err_counter0;
	/**< Error Counter1  */
	uint64_t err_counter1;
	/**< Error Counter2  */
	uint64_t err_counter2;
	/**< Error Counter3  */
	uint64_t err_counter3;
};

/************************** Function Prototypes ******************************/
int xcmac_rsfec_set_bypass(struct xcmac *instance,
		enum xcmac_rsfec_bypass_type bypass_type);
int xcmac_rsfec_clear_bypass(struct xcmac *instance,
		enum xcmac_rsfec_bypass_type bypass_type);
int xcmac_rsfec_set_err_indication_mode(struct xcmac *instance);
int xcmac_rsfec_clear_err_indication_mode(struct xcmac *instance);
uint8_t xcmac_rsfec_get_bypass_correction_settings(struct xcmac *instance);
uint8_t xcmac_rsfec_get_bypass_indication_settings(struct xcmac *instance);
uint8_t xcmac_rsfec_get_error_indication_settings(struct xcmac *instance);
uint8_t xcmac_rsfec_get_bypass_ability(struct xcmac *instance,
		enum xcmac_rsfec_bypass_type bypass_type);
uint8_t xcmac_rsfec_get_hiser_status(struct xcmac *instance);
uint8_t xcmac_rsfec_get_lhhiser_status(struct xcmac *instance);
int xcmac_rsfec_get_lane_alignment_status(struct xcmac *instance,
		uint8_t *fec_lane,
		uint8_t *pcs_lane);
int xcmac_rsfec_get_lane_mapping(struct xcmac *instance,
		struct xcmac_rsfec_lane *lane_map);
uint8_t xcmac_rsfec_get_block_lock_status(struct xcmac *instance);
int xcmac_rsfec_get_am_lock_status(struct xcmac *instance,
		struct xcmac_rsfec_am_lock *am_lock_status);
uint8_t xcmac_rsfec_get_pcs_am_lock_status(struct xcmac *instance);
int xcmac_rsfec_get_cwcounter(struct xcmac *instance,
		struct xcmac_rsfec_cw_counters *xcmac_rsfec_cw_counters);
int xcmac_rsfec_request_reset(struct xcmac *instance,
		enum xcmac_core_type interface);
int xcmac_rsfec_get_rx_lane_delay(struct xcmac *instance,
		uint32_t *rxlanedelay01,
		uint32_t *rxlanedelay23);
int xcmac_rsfec_set_block_bypass(struct xcmac *instance, uint8_t flag);
int xcmac_rsfec_get_bypass_rx_tx_setting(struct xcmac *instance,
		uint8_t *rx_setting,
		uint8_t *tx_setting);
int xcmac_rsfec_get_symbol_error_counters(
		struct xcmac *instance,
		struct xcmac_rsfec_sym_err_counters *err_counters);

#ifdef __cplusplus
}
#endif
#endif /* end of protection macro */
