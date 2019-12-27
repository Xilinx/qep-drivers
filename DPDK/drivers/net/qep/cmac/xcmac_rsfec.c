/*
 * Copyright(c) 2015-2019 Xilinx, Inc. All rights reserved.
 *
 * BSD LICENSE
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

#include "xcmac.h"
#include "xcmac_rsfec.h"

/** Function macro to check if the argument is NULL */
#define is_null(arg)                                                           \
	do { if (!(arg))                                                       \
		return -EINVAL; } while (0)

/**< Number of bits in a WORD */
#define NUM_WORD_BITS 32

static uint64_t xcmac_read_64bit_value(struct xcmac *instance, uint32_t offset);

/*****************************************************************************/
/** This function reads 64-bit data from the offset
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param offset - Location from which data to be read.
 *
 *  @return 64-bit value
 *
 *****************************************************************************/
static uint64_t xcmac_read_64bit_value(struct xcmac *instance, uint32_t offset)
{
	uint32_t lsb = 0, msb = 0;
	uint64_t data = 0;

	lsb = xcmac_in32(instance->base_address + offset);
	offset += 4;
	msb = xcmac_in32(instance->base_address + offset);
	data = msb;
	data = ((data << NUM_WORD_BITS) | lsb);
	return data;
}

/*****************************************************************************/
/** This function is to reset Rx or Tx Path based on the given interface.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param core - RSFEC interface i.e Rx or Tx.
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_request_reset(struct xcmac *instance, enum xcmac_core_type core)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	switch (core) {
	case XCMAC_CORE_RX:
		xcmac_out32(instance->base_address +
			XCMAC_RSFEC_RX_RESET_REQUEST_OFFSET,
			RESET_RXTX_PATH_VALUE);
		break;
	case XCMAC_CORE_TX:
		xcmac_out32(instance->base_address +
			XCMAC_RSFEC_TX_RESET_REQUEST_OFFSET,
			RESET_RXTX_PATH_VALUE);
		break;
	case XCMAC_CORE_BOTH:
		xcmac_out32(instance->base_address +
			XCMAC_RSFEC_RX_RESET_REQUEST_OFFSET,
			RESET_RXTX_PATH_VALUE);
		xcmac_out32(instance->base_address +
			XCMAC_RSFEC_TX_RESET_REQUEST_OFFSET,
			RESET_RXTX_PATH_VALUE);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*****************************************************************************/
/** This function is to get RSFEC current ByPass correction settings.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *
 *  @return Bypass correction settings
 *
 *****************************************************************************/
uint8_t xcmac_rsfec_get_bypass_correction_settings(struct xcmac *instance)
{
	uint32_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_CONFIGURATION_OFFSET);
	return ((data & XCMAC_RSFEC_BYPASS_CORRECTION_ENABLE_STATUS_MASK) >> 8);
}

/*****************************************************************************/
/** This function is to get RSFEC Current ByPass indication settings.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *
 *  @return Bypass indication settings
 *
 *****************************************************************************/
uint8_t xcmac_rsfec_get_bypass_indication_settings(struct xcmac *instance)
{
	uint32_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_CONFIGURATION_OFFSET);
	return ((data & XCMAC_RSFEC_BYPASS_INDICATION_ENABLE_STATUS_MASK) >> 9);
}

/*****************************************************************************/
/** This function is to get RSFEC Error Indication settings.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *
 *  @return Error indication settings
 *
 *****************************************************************************/
uint8_t xcmac_rsfec_get_error_indication_settings(struct xcmac *instance)
{
	uint32_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_CONFIGURATION_OFFSET);
	return ((data & XCMAC_RSFEC_ERROR_INDICATION_MODE_STATUS_MASK) >> 10);
}

/*****************************************************************************/
/** This function is to get RSFEC ByPass correction ability.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param bypass_type - ByPass type i.e Correction or Indication.
 *
 *  @return RSFEC ByPass ability
 *          Error number on failure
 *
 *****************************************************************************/
uint8_t xcmac_rsfec_get_bypass_ability(struct xcmac *instance,
			 enum xcmac_rsfec_bypass_type bypass_type)
{
	uint32_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address + XCMAC_RSFEC_STATUS_OFFSET);
	if (bypass_type == RSFEC_CORRECTION) {
		data &= XCMAC_RSFEC_STATUS_BYPASS_CORRECTION_ABILITY_MASK;
		return (uint8_t)data;
	} else if (bypass_type == RSFEC_INDICATION) {
		data &= XCMAC_RSFEC_STATUS_BYPASS_INDICATION_ABILITY_MASK;
		return (uint8_t)(data >> 1);
	} else {
		return -EINVAL;
	}
}

/*****************************************************************************/
/** This function is to get RSFEC Hi Ser status.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *
 *
 *  @return RSFEC Hi Ser status.
 *
 *****************************************************************************/
uint8_t xcmac_rsfec_get_hiser_status(struct xcmac *instance)
{
	uint32_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address + XCMAC_RSFEC_STATUS_OFFSET);
	data &= XCMAC_RSFEC_STATUS_HI_SER_MASK;
	return (uint8_t)(data >> 2);
}

/*****************************************************************************/
/** This function is to get RSFEC Lh Hi Ser status.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *
 *  @return RSFEC Lh Hi Ser status.
 *
 *****************************************************************************/
uint8_t xcmac_rsfec_get_lhhiser_status(struct xcmac *instance)
{
	uint32_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address + XCMAC_RSFEC_STATUS_OFFSET);
	data &= XCMAC_RSFEC_STATUS_LH_HI_SER_MASK;
	return (uint8_t)(data >> 3);
}

/*****************************************************************************/
/** This function is to get RSFEC AM lock status.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param am_lock_status - Am Lock status.
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_get_am_lock_status(struct xcmac *instance,
		 struct xcmac_rsfec_am_lock *am_lock_status)
{
	uint32_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address + XCMAC_RSFEC_STATUS_OFFSET);
	/* FecAMClock 0 bit 8 of 0x04 */
	am_lock_status->am_lock0 =
		(uint8_t)((data & XCMAC_RSFEC_STATUS_FEC_AM_CLOCK0_MASK) >> 8);
	/* FecAMClock 1 bit 9 of 0x04 */
	am_lock_status->am_lock1 =
		(uint8_t)((data & XCMAC_RSFEC_STATUS_FEC_AM_CLOCK1_MASK) >> 9);
	/* FecAMClock 2 bit 10 of 0x04 */
	am_lock_status->am_lock2 =
		(uint8_t)((data & XCMAC_RSFEC_STATUS_FEC_AM_CLOCK2_MASK) >> 10);
	/* FecAMClock 3 bit 11 of 0x04 */
	am_lock_status->am_lock3 =
		(uint8_t)((data & XCMAC_RSFEC_STATUS_FEC_AM_CLOCK3_MASK) >> 11);
	return 0;
}

/*****************************************************************************/
/** This function is to get RS-FEC Lane alignment status.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param fec_lane  - RS-FEC FEC Lane Alignment status
 *  @param pcs_lane  - RS-FEC PCS Lane Alignment status
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_get_lane_alignment_status(struct xcmac *instance,
		uint8_t *fec_lane, uint8_t *pcs_lane)
{
	uint32_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address + XCMAC_RSFEC_STATUS_OFFSET);
	*fec_lane = ((data & XCMAC_RSFEC_STATUS_FEC_LANE_ALIGNMENT_MASK) >> 14);
	*pcs_lane = ((data & XCMAC_RSFEC_STATUS_PCS_LANE_ALIGNMENT_MASK) >> 15);
	return 0;
}

/*****************************************************************************/
/** This function is to Set ByPass Configuration settings.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param bypass_type  - Enum for ByPass type i.e Correction or Indication.
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_set_bypass(struct xcmac *instance,
		enum xcmac_rsfec_bypass_type bypass_type)
{
	uint16_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_CONFIGURATION_OFFSET);
	if (bypass_type == RSFEC_CORRECTION)
		data |= XCMAC_RSFEC_BYPASS_CORRECTION_ENABLE_MASK;
	else if (bypass_type == RSFEC_INDICATION)
		data |= XCMAC_RSFEC_BYPASS_INDICATION_ENABLE_MASK;
	else
		return -EINVAL;

	xcmac_out32(instance->base_address + XCMAC_RSFEC_CONFIGURATION_OFFSET,
				data);
	return 0;
}

/*****************************************************************************/
/** This function is to Clear ByPass Configuration settings.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param bypass_type  - Enum to indicate ByPass type i.e Correction or
 *  Indication.
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_clear_bypass(struct xcmac *instance,
		 enum xcmac_rsfec_bypass_type bypass_type)
{
	uint16_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_CONFIGURATION_OFFSET);
	if (bypass_type == RSFEC_CORRECTION)
		data &= (~XCMAC_RSFEC_BYPASS_CORRECTION_ENABLE_MASK);
	else if (bypass_type == RSFEC_INDICATION)
		data &= (~XCMAC_RSFEC_BYPASS_INDICATION_ENABLE_MASK);
	else
		return -EINVAL;

	xcmac_out32(instance->base_address + XCMAC_RSFEC_CONFIGURATION_OFFSET,
				data);
	return 0;
}

/*****************************************************************************/
/** This function is to Set Error Indication mode.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_set_err_indication_mode(struct xcmac *instance)
{
	uint16_t data;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_CONFIGURATION_OFFSET);
	data |= XCMAC_RSFEC_ERROR_INDICATION_MODE_MASK;
	xcmac_out32(instance->base_address + XCMAC_RSFEC_CONFIGURATION_OFFSET,
				data);
	return 0;
}

/*****************************************************************************/
/** This function is to Clear Error Indication mode.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_clear_err_indication_mode(struct xcmac *instance)
{
	uint16_t data;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_CONFIGURATION_OFFSET);
	data &= (~XCMAC_RSFEC_ERROR_INDICATION_MODE_MASK);
	xcmac_out32(instance->base_address + XCMAC_RSFEC_CONFIGURATION_OFFSET,
				data);
	return 0;
}

/*****************************************************************************/
/** This function is to get Codeword counter.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param cw_counters  - A structure to hold Codeword counters.
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_get_cwcounter(struct xcmac *instance,
			struct xcmac_rsfec_cw_counters *cw_counters)
{
	uint32_t cw_counter_lsb = 0, cw_counter_msb = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	cw_counter_lsb = xcmac_in32(instance->base_address +
			XCMAC_RSFEC_CORRECT_CW_COUNTER_LSB_OFFSET);
	cw_counter_msb = xcmac_in32(instance->base_address +
			XCMAC_RSFEC_CORRECT_CW_COUNTER_MSB_OFFSET);
	cw_counters->correct_counter = cw_counter_msb;
	cw_counters->correct_counter =
		((cw_counters->correct_counter << NUM_WORD_BITS) |
		cw_counter_lsb);
	cw_counter_lsb = xcmac_in32(instance->base_address +
			 XCMAC_RSFEC_UNCORRECT_CW_COUNTER_LSB_OFFSET);
	cw_counter_msb = xcmac_in32(instance->base_address +
			 XCMAC_RSFEC_UNCORRECT_CW_COUNTER_MSB_OFFSET);
	cw_counters->uncorrect_counter = cw_counter_msb;
	cw_counters->uncorrect_counter =
		((cw_counters->uncorrect_counter << NUM_WORD_BITS) |
		cw_counter_lsb);
	return 0;
}

/*****************************************************************************/
/** This function is to Get Lane mapping for each Lane.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param lane_map  - Lane to hold lane mapping.
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_get_lane_mapping(struct xcmac *instance,
		struct xcmac_rsfec_lane *lane_map)
{
	uint8_t line = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	line = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_STATUS_FEC_LINEMAP_OFFSET);
	lane_map->lane0 = (line & XCMAC_RSFEC_STATUS_FEC_LANE0_MASK) >> 0;
	lane_map->lane1 = (line & XCMAC_RSFEC_STATUS_FEC_LANE1_MASK) >> 2;
	lane_map->lane2 = (line & XCMAC_RSFEC_STATUS_FEC_LANE2_MASK) >> 4;
	lane_map->lane3 = (line & XCMAC_RSFEC_STATUS_FEC_LANE3_MASK) >> 6;
	return 0;
}

/*****************************************************************************/
/** This function is to get RS-FEC block lock status.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *
 *  @return Block lock status
 *
 *****************************************************************************/
uint8_t xcmac_rsfec_get_block_lock_status(struct xcmac *instance)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	/**
	 * 1: lane is locked     0: lane is not locked Signals from the 100G
	 * Ethernet MAC are in sync, all lanes lock and unlock at the same time,
	 * so 1 bit would suffice; however all 32 bits are provided for
	 * specification compatibility.
	 */
	return (uint8_t)(xcmac_in32(instance->base_address +
			XCMAC_RSFEC_BLOCK_LOCK_STATUS_OFFSET) & 0x01);
}

/*****************************************************************************/
/** This function is to get RS-FEC AM/PCS lock status.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *
 *  @return PCS AM lock status
 *
 *****************************************************************************/
uint8_t xcmac_rsfec_get_pcs_am_lock_status(struct xcmac *instance)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	return (xcmac_in32(instance->base_address +
		 XCMAC_RSFEC_PCS_AM_LOCK_STATUS_OFFSET) & 0x01);
}

/*****************************************************************************/
/** This function is to get rx lane delay.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param rx_lane_delay01  - Rx Lane delay for lane0 and lane1.
 *  @param rx_lane_delay23  - Rx Lane delay for lane2 and lane3.
 *
 *  @return 0 for success
 *          Error number if there is error
 *
 *****************************************************************************/
int xcmac_rsfec_get_rx_lane_delay(struct xcmac *instance,
		uint32_t *rx_lane_delay01, uint32_t *rx_lane_delay23)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	/* Read Rx lane delay0 and 1*/
	*rx_lane_delay01 = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_RX_LANE_DELAY01_OFFSET);
	/* Read Rx lane delay2 and 3*/
	*rx_lane_delay23 = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_RX_LANE_DELAY23_OFFSET);
	return 0;
}

/*****************************************************************************/
/** This function is to Set Block Bypass control bit.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param flag  - Indicates to set or Clear.
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_set_block_bypass(struct xcmac *instance, uint8_t flag)
{
	uint32_t data = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	data = xcmac_in32(instance->base_address +
				XCMAC_RSFEC_BLOCK_BYPASS_OFFSET);
	/*Clear bit before update */
	data &= (~XCMAC_RSFEC_BLOCK_BYPASS_MASK);
	data |= flag;
	xcmac_out32(instance->base_address + XCMAC_RSFEC_BLOCK_BYPASS_OFFSET,
				data);
	return 0;
}

/*****************************************************************************/
/** This function is to Get Bypass Rx Tx Settings.
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param rx_setting  - Rx Settings.
 *  @param tx_setting  - Tx Settings.
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_get_bypass_rx_tx_setting(struct xcmac *instance,
		uint8_t *rx_setting, uint8_t *tx_setting)
{
	uint32_t rx_tx_setting = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	rx_tx_setting = xcmac_in32(instance->base_address +
			 XCMAC_RSFEC_BLOCK_BYPASS_OFFSET);
	/* FEC_bypass_RX_setting (Bit 8 of 0x7C)*/
	*rx_setting = (uint8_t)(
		(rx_tx_setting & XCMAC_RSFEC_RX_BYPASS_STATUS_MASK) >> 8);
	/* FEC_bypass_TX_setting Bit 9 of 0x7C */
	*tx_setting = (uint8_t)(
		(rx_tx_setting & XCMAC_RSFEC_TX_BYPASS_STATUS_MASK) >> 9);
	return 0;
}

/*****************************************************************************/
/** This function is to get symbol error counters
 *
 *  @param instance - A pointer to the CMAC instance to be worked on.
 *  @param err_counters  - Pointer to structure of Symbol Error Counters.
 *
 *  @return 0 on Success.
 *          Error number on Failure/Error.
 *****************************************************************************/
int xcmac_rsfec_get_symbol_error_counters(struct xcmac *instance,
		struct xcmac_rsfec_sym_err_counters *err_counters)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(err_counters);
	/* Read FEC_symbol_error_counter_0, offset 0x1C and 0x20 */
	err_counters->err_counter0 = xcmac_read_64bit_value(
		instance, XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_0_LSB_OFFSET);
	/* Read FEC_symbol_error_counter_1, offset 0x24 and 0x28 */
	err_counters->err_counter1 = xcmac_read_64bit_value(
		instance, XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_1_LSB_OFFSET);
	/* Read FEC_symbol_error_counter_2, offset 0x2C and 0x30 */
	err_counters->err_counter2 = xcmac_read_64bit_value(
		instance, XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_2_LSB_OFFSET);
	/* Read FEC_symbol_error_counter_3, offset 0x34 and 0x38 */
	err_counters->err_counter3 = xcmac_read_64bit_value(
		instance, XCMAC_RSFEC_SYMBOL_ERROR_COUNTER_3_LSB_OFFSET);
	return 0;
}
