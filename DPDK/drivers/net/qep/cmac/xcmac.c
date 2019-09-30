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

#ifdef __KERNEL__
#include <linux/delay.h>
#else
#include <unistd.h>
#endif
#include "xcmac.h"

/** function macro to return TRUE/FALSE values */
#define get_status(offset, mask)                                               \
	((xcmac_in32(instance->base_address + offset) & mask) ? true : false)
/** function macro to read word with mask */
#define read_mem_with_mask(offset, mask)                                       \
	(xcmac_in32(instance->base_address + offset) & mask)

void xcmac_out32(uint64_t address, uint32_t data)
{
	*((uint32_t *)(address)) = data;
}

uint32_t xcmac_in32(uint64_t address)
{
	uint32_t val;

	val = *((uint32_t *)(address));

	return val;
}

/*****************************************************************************/
/**
 * This function reads 48 bit value from the registers
 *
 * @param    instance is the driver instance that is working on
 * @param    lower_address is lower address of the register.
 *
 * @return  48 bit value
 *
 * @note    none.
 *
 *****************************************************************************/
static uint64_t xcmac_read_48bit_value(struct xcmac *instance,
		uint32_t lower_address)
{
	uint64_t data = 0, result = 0;

	data = read_mem_with_mask((lower_address + NUM_WORD_BYTES),
			HALF_WORD_MASK);
	result = (uint64_t)(data << NUM_DWORD_BYTES);
	data = read_mem_with_mask(lower_address, WORD_MASK);
	result |= (data);
	return result;
}

/*****************************************************************************/
/**
 * This function initializes xcmac driver data structures.
 * it should be called before any other function calls to the driver.
 *
 * @param    instance is the driver instance that is working on
 * @param    cmac_va_base is the virtual address where the CMAC IP is mapped.
 *
 * @return
 *        - 0 for success
 *        - Error number for failure
 *
 * @note    none.
 *
 *****************************************************************************/
int xcmac_initialize(struct xcmac *instance, uint64_t cmac_va_base)
{
	is_null(instance);
	/*
	 * If the device is started, disallow the initialize and return a status
	 * indicating it is started.  This allows the user to stop the device
	 * and reinitialize, but prevents a user from inadvertently initializing
	 */
	if (instance->is_started == XIL_COMPONENT_IS_STARTED)
		return -EPERM;
	/* Set some default values */
	instance->is_ready = 0;
	instance->is_started = 0; /* not started */
	instance->base_address = cmac_va_base;
	/* Indicate instance is now ready to use, initialized without error */
	instance->is_ready = XIL_COMPONENT_IS_READY;
	return 0;
}

/*****************************************************************************/
/**
 * This function resets the CMAC GT
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return
 *        - 0 for success.
 *        - Error number for failure.
 *
 * @note   none.
 *
 ******************************************************************************/
int xcmac_reset_gt(struct xcmac *instance)
{
	uint32_t offset = XCMAC_GT_RESET_REG_OFFSET, value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	/* read the value */
	value = xcmac_in32(instance->base_address + offset);
	/* Assert the bit */
	value = (value | XCMAC_GT_RESET_MASK);
	xcmac_out32(instance->base_address + offset, value);
	/* De-Assert the bit */
	value = (value & ~(XCMAC_GT_RESET_MASK));
	xcmac_out32(instance->base_address + offset, value);
	return 0;
}

/*****************************************************************************/
/**
 * This function performs Rx SERDES reset and Rx, Tx Cores reset
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    core specifies Rx or Tx or both Rx, Tx interfaces
 *
 * @return
 *        - 0 for success.
 *        - Error number for failure.
 *
 * @note    none
 *
 ******************************************************************************/
int xcmac_reset_core(struct xcmac *instance, enum xcmac_core_type core)
{
	uint32_t offset = XCMAC_RESET_REG_OFFSET, mask = 0, read_value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	read_value = xcmac_in32(instance->base_address + offset);
	/* reset usr_rx_serdes_reset */
	mask = XCMAC_USR_RX_SERDES_RESET_MASK;
	switch (core) {
	case XCMAC_CORE_TX:
		mask |= XCMAC_TX_CORE_RESET_MASK;
		break;
	case XCMAC_CORE_RX:
		mask |= XCMAC_RX_CORE_RESET_MASK;
		break;
	case XCMAC_CORE_BOTH:
		mask |= (XCMAC_RX_CORE_RESET_MASK | XCMAC_TX_CORE_RESET_MASK);
		break;
	default:
		return -EINVAL;
	}
	/* Assert all the bits simultaneously */
	read_value = (read_value | mask);
	xcmac_out32(instance->base_address + offset, read_value);
#ifdef __KERNEL__
	usleep_range(5000, 10000);
#else
	usleep(10 * 1000);
#endif
	/* De-Assert all the bits simultaneously */
	read_value = (read_value & ~(mask));
	xcmac_out32(instance->base_address + offset, read_value);
	return 0;
}

/*****************************************************************************/
/**
 * This function returns the CMAC core version.
 *
 * @param    instance is the driver instance that is working on
 *
 * @return  CMAC core version number
 *
 * @note    none.
 *
 *****************************************************************************/
uint32_t xcmac_get_version(struct xcmac *instance)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	return xcmac_in32(instance->base_address +
	XCMAC_CMAC_VERSION_REG_OFFSET);
}

/*************************************************************************/
/**
 * This function enables the CMAC Rx/Tx interface.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    core is interface (rx/tx) that is to be enabled.
 *
 * @return
 * - 0 if the device is enabled
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_enable(struct xcmac *instance, enum xcmac_core_type core)
{
	uint32_t value = 0;

	/* Assert the arguments */
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	switch (core) {
	case XCMAC_CORE_TX:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET);
		set_bit(value, XCMAC_CTL_TX_ENABLE_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET,
				value);
		break;
	case XCMAC_CORE_RX:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET);
		set_bit(value, XCMAC_CTL_RX_ENABLE_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET,
				value);
		break;
	case XCMAC_CORE_BOTH:
		/* Tx */
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET);
		set_bit(value, XCMAC_CTL_TX_ENABLE_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET,
				value);
		/* Rx */
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET);
		set_bit(value, XCMAC_CTL_RX_ENABLE_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET,
				value);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*************************************************************************/
/**
 * This function disables the xcmac device.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    core is interface (rx/tx) that is to be disabled.
 *
 * @return
 * - 0 if the device is disabled
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_disable(struct xcmac *instance, enum xcmac_core_type core)
{
	uint32_t value = 0;

	/* Assert the arguments */
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	switch (core) {
	case XCMAC_CORE_TX:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET);
		clear_bit(value, XCMAC_CTL_TX_ENABLE_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET,
				value);
		break;
	case XCMAC_CORE_RX:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET);
		clear_bit(value, XCMAC_CTL_RX_ENABLE_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET,
				value);
		break;
	case XCMAC_CORE_BOTH:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET);
		clear_bit(value, XCMAC_CTL_TX_ENABLE_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET,
				value);
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET);
		clear_bit(value, XCMAC_CTL_RX_ENABLE_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET,
				value);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*************************************************************************/
/**
 * This function gets the current settings (rx/tx enabled status)
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    rx_en_status is a pointer to store the rx enabled status.
 * @param    tx_en_status is a pointer to store the tx enabled status.
 *
 * @return
 * - 0 for success
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_get_device_config(struct xcmac *instance, uint8_t *rx_en_status,
		uint8_t *tx_en_status)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	is_null(rx_en_status);
	is_null(tx_en_status);
	*rx_en_status = get_status(XCMAC_CONFIGURATION_RX_REG1_OFFSET,
			XCMAC_CTL_RX_ENABLE_MASK);
	*tx_en_status = get_status(XCMAC_CONFIGURATION_TX_REG1_OFFSET,
			XCMAC_CTL_TX_ENABLE_MASK);
	return 0;
}

/*************************************************************************/
/**
 * This function sets the 100ge PCS to the given CAUI mode
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    caui_mode is CAUI mode type.
 *
 * @return
 * - 0 if the CAUI mode is set successfully
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_set_caui_mode(struct xcmac *instance, enum xcmac_caui_mode caui_mode)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	if (caui_mode > XCMAC_CAUI_MODE_4)
		return -EINVAL;
	xcmac_out32(instance->base_address + XCMAC_MODE_REG_OFFSET,
			(caui_mode & XCMAC_CAUI_MODE_MASK));
	return 0;
}

/*************************************************************************/
/**
 * This function returns the current CAUI mode configuration
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return current CAUI mode
 *
 * @note none.
 *
 **************************************************************************/
enum xcmac_caui_mode  xcmac_get_caui_mode(struct xcmac *instance)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return XCMAC_CAUI_INVALID;
	return read_mem_with_mask(XCMAC_MODE_REG_OFFSET, XCMAC_CAUI_MODE_MASK);
}

/*************************************************************************/
/**
 * This function returns the current core mode configuration
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return current CAUI mode
 *
 * @note none.
 *
 **************************************************************************/
enum xcmac_caui_mode  xcmac_get_core_mode(struct xcmac *instance)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return XCMAC_CAUI_INVALID;
	return read_mem_with_mask(XCMAC_CORE_MODE_REG_OFFSET,
			XCMAC_CORE_MODE_MASK);
}

/*************************************************************************/
/**
 * This function configures the fault indication signals
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    tx_send_rfi indicates to set/clear rfi.
 *
 * @return
 *
 * - 0 if the fault indication signals are configured
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_set_fault_indication(struct xcmac *instance, uint8_t tx_send_rfi)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	value = xcmac_in32(instance->base_address +
	XCMAC_CONFIGURATION_TX_REG1_OFFSET);
	if (tx_send_rfi == true)
		set_bit(value, XCMAC_CTL_TX_SEND_RFI_EN_BIT);
	else
		clear_bit(value, XCMAC_CTL_TX_SEND_RFI_EN_BIT);
	xcmac_out32(instance->base_address + XCMAC_CONFIGURATION_TX_REG1_OFFSET,
			value);
	return 0;
}

/*************************************************************************/
/**
 * This function gets the current configuration of fault indication signals
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return   fault indication status
 *
 * @note none.
 *
 **************************************************************************/
uint8_t xcmac_get_fault_indication(struct xcmac *instance)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	value = xcmac_in32(instance->base_address +
	XCMAC_CONFIGURATION_TX_REG1_OFFSET);
	return ((value & XCMAC_CTL_TX_SEND_RFI_EN_BIT_MASK) >>
	XCMAC_CTL_TX_SEND_RFI_EN_BIT);
}

/*************************************************************************/
/**
 * This function transmits idle packets
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return
 *
 * - 0 if the transmit idle insertion is success
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_send_tx_idle(struct xcmac *instance)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	value = xcmac_in32(instance->base_address +
	XCMAC_CONFIGURATION_TX_REG1_OFFSET);
	set_bit(value, XCMAC_CTL_TX_SEND_IDLE_EN_BIT);
	xcmac_out32(instance->base_address + XCMAC_CONFIGURATION_TX_REG1_OFFSET,
			value);
	return 0;
}

/*************************************************************************/
/**
 * This function returns the transmit idle insertion value representing
 * transmit idle is enabled or disabled.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return
 *      a boolean value representing the current idle insertion status
 *      with true representing enabled and false representing disabled.
 *
 * @note none.
 *
 **************************************************************************/
uint8_t xcmac_get_tx_idle_status(struct xcmac *instance)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	value = xcmac_in32(instance->base_address +
	XCMAC_CONFIGURATION_TX_REG1_OFFSET);
	return ((value & XCMAC_CTL_TX_SEND_IDLE_EN_BIT_MASK) >>
	XCMAC_CTL_TX_SEND_IDLE_EN_BIT);
}

/*************************************************************************/
/**
 * This function enables test patterns in the rx, tx or both directions.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    core is direction type
 *
 * @return
 *
 * - 0 if test pattern is enabled
 * - Error number if there is an error
 *
 * @note Test pattern generation enable for the Rx and TX cores. A value of
 * 1 enables test mode.
 *
 * 16 RW ctl_tx_test_pattern
 * 8 bit ctl_rx_test_pattern
 *
 **************************************************************************/
int xcmac_enable_test_pattern(struct xcmac *instance, enum xcmac_core_type core)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	switch (core) {
	case XCMAC_CORE_TX:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET);
		set_bit(value, XCMAC_TX_TEST_PATTERN_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET,
				value);
		break;
	case XCMAC_CORE_RX:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET);
		set_bit(value, XCMAC_RX_TEST_PATTERN_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET,
				value);
		break;
	case XCMAC_CORE_BOTH:
		/* Tx */
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET);
		set_bit(value, XCMAC_TX_TEST_PATTERN_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET,
				value);
		/* Rx */
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET);
		set_bit(value, XCMAC_RX_TEST_PATTERN_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET,
				value);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*************************************************************************/
/**
 * This function disables test patterns in the rx, tx or both directions.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    core is direction type
 *
 * @return
 *
 * - 0 if test pattern is disabled
 * - Error number if the direction type is invalid
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_disable_test_pattern(struct xcmac *instance,
		enum xcmac_core_type core)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	switch (core) {
	case XCMAC_CORE_TX:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET);
		clear_bit(value, XCMAC_TX_TEST_PATTERN_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET,
				value);
		break;
	case XCMAC_CORE_RX:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET);
		clear_bit(value, XCMAC_RX_TEST_PATTERN_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET,
				value);
		break;
	case XCMAC_CORE_BOTH:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET);
		clear_bit(value, XCMAC_TX_TEST_PATTERN_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET,
				value);
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET);
		clear_bit(value, XCMAC_RX_TEST_PATTERN_EN_BIT);
		xcmac_out32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET,
				value);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current configuration of test pattern
 * in each direction.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    core is direction type
 *
 * @return   Test pattern is enable status
 *
 * @note none.
 *
 **************************************************************************/
uint8_t xcmac_get_test_pattern_status(struct xcmac *instance,
		enum xcmac_core_type core)
{
	uint8_t status = 0;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	switch (core) {
	case XCMAC_CORE_TX:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_TX_REG1_OFFSET);
		status = ((value & XCMAC_TX_TEST_PATTERN_EN_BIT_MASK) >>
		XCMAC_TX_TEST_PATTERN_EN_BIT);
		break;
	case XCMAC_CORE_RX:
		value = xcmac_in32(instance->base_address +
		XCMAC_CONFIGURATION_RX_REG1_OFFSET);
		status = ((value & XCMAC_RX_TEST_PATTERN_EN_BIT_MASK) >>
		XCMAC_RX_TEST_PATTERN_EN_BIT);
		break;
	default:
		return -EINVAL;
	}
	return status;
}

/*************************************************************************/
/**
 * This function forces a receiver resync.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note This signal is used to force the RX path to reset, re-synchronize,
 * and realign. A value of 1 forces the reset operation.
 * A value of 0 allows normal operation
 *
 **************************************************************************/
int xcmac_force_rx_resync(struct xcmac *instance)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_CONFIGURATION_RX_REG1_OFFSET);
	set_bit(value, XCMAC_RX_TEST_PATTERN_EN_BIT);
	xcmac_out32(instance->base_address + XCMAC_CONFIGURATION_RX_REG1_OFFSET,
			value);
	return 0;
}

/*************************************************************************/
/**
 * This function configures tx bip7 parameters.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    bip7config is a pointer to hold bip7 configuration values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note This value is set for Tx interface
 *
 **************************************************************************/
int xcmac_set_tx_bip7_config(struct xcmac *instance,
		struct xcmac_bip7_config *bip7_config)
{
	uint32_t value = 0;

	is_null(instance);
	is_null(bip7_config);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_CONFIG_TX_BIP_OVERRIDE_OFFSET);
	/* clear the Value field */
	value = value & ~(XCMAC_CTL_TX_LANE0_VLM_BIP7_OVERRIDE_VALUE_MASK);
	/* update Value field */
	value |= bip7_config->bip7_value;
	/* clear the Valid bit */
	value = value & ~(XCMAC_CTL_TX_LANE0_VLM_BIP7_OVERRIDE_BIT_MASK);
	/* set/unset the Valid bit */
	value |= bip7_config->bip7_valid;
	xcmac_out32(instance->base_address +
	XCMAC_CONFIG_TX_BIP_OVERRIDE_OFFSET,
			value);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current bip7 value and its enabled or disabled status
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    bip7config is a pointer to hold bip7 configuration values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note This value is read from one of the Rx interface status register.
 *
 **************************************************************************/
int xcmac_get_bip7_config(struct xcmac *instance,
		struct xcmac_bip7_config *bip7_config)
{
	uint32_t value = 0;

	is_null(instance);
	is_null(bip7_config);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	STAT_RX_BIP_OVERRIDE_OFFSET);
	bip7_config->bip7_value = (value & STAT_RX_BIP_OVERRIDE_BIP7_MASK);
	bip7_config->bip7_valid =
			(uint8_t)((value & STAT_RX_BIP_OVERRIDE_VALID_MASK) >>
			STAT_RX_BIP_OVERRIDE_VALID_BIT);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the OTN status
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    otn_status is a pointer to hold Tx otn status values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 **************************************************************************/
int xcmac_get_otn_status(struct xcmac *instance,
		struct xcmac_tx_otn_status *otn_status)
{
	uint32_t value = 0;

	is_null(instance);
	is_null(otn_status);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_STAT_TX_OTN_STATUS_OFFSET);
	otn_status->remote_fault = (uint8_t)(value &
			STAT_TX_OTN_STATUS_TX_REMOTE_FAULT_BIT);
	otn_status->internal_local_fault = (uint8_t)((value &
			STAT_TX_OTN_STATUS_TX_INTERNAL_LOCAL_FAULT_MASK)
			>> STAT_TX_OTN_STATUS_TX_INTERNAL_LOCAL_FAULT_BIT);
	otn_status->received_local_fault = (uint8_t)((value &
			STAT_TX_OTN_STATUS_TX_RECEIVED_LOCAL_FAULT_MASK)
			>> STAT_TX_OTN_STATUS_TX_RECEIVED_LOCAL_FAULT_BIT);
	otn_status->test_pattern_mismatch = (uint8_t)((value &
			STAT_TX_OTN_STATUS_TX_TEST_PATTERN_MISMATCH_MASK) >>
			STAT_TX_OTN_STATUS_TX_TEST_PATTERN_MISMATCH_BIT);
	otn_status->bad_preamble = (uint8_t)((value
			& STAT_TX_OTN_STATUS_TX_BAD_PREAMBLE_MASK)
			>> STAT_TX_OTN_STATUS_TX_BAD_PREAMBLE_BIT);
	otn_status->bad_sfd = (uint8_t)((value &
			STAT_TX_OTN_STATUS_TX_BAD_SFD_MASK)
			>> STAT_TX_OTN_STATUS_TX_BAD_SFD_BIT);
	otn_status->got_signal_os = (uint8_t)((value
			& STAT_TX_OTN_STATUS_TX_GOT_SIGNAL_OS_MASK)
			>> STAT_TX_OTN_STATUS_TX_GOT_SIGNAL_OS_BIT);

	return 0;
}

/*************************************************************************/
/**
 * This function sets the Tx OTN packet length
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    pkt_len is a pointer to hold Tx packet length values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 **************************************************************************/
int xcmac_set_otn_pkt_length(struct xcmac *instance,
		struct xcmac_tx_otn_pkt_length *pkt_len)
{
	uint32_t value = 0;

	is_null(instance);
	is_null(pkt_len);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_CONFIG_TX_OTN_PKT_LEN_REG_OFFSET);

	/* clear the min pkt length field */
	value = value & ~(XCMAC_CONFIG_TX_OTN_MIN_PKT_LEN_MASK);
	/* update min pkt length field */
	value |= pkt_len->min_pkt_length;

	/* clear the max pkt length bit */
	value = value & ~(XCMAC_CONFIG_TX_OTN_MAX_PKT_LEN_MASK);
	/* set/unset the max pkt length bit */
	value |= (((uint32_t)(pkt_len->max_pkt_length))
			<< XCMAC_CONFIG_TX_OTN_MAX_PKT_LEN_BIT);

	xcmac_out32(instance->base_address +
	XCMAC_CONFIG_TX_OTN_PKT_LEN_REG_OFFSET,
			value);

	return 0;
}

/*************************************************************************/
/**
 * This function get the Tx OTN packet length
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    pkt_len is a pointer to hold Tx packet length values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 **************************************************************************/
int xcmac_get_otn_pkt_length(struct xcmac *instance,
		struct xcmac_tx_otn_pkt_length *pkt_len)
{
	uint32_t value = 0;

	is_null(instance);
	is_null(pkt_len);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_CONFIG_TX_OTN_PKT_LEN_REG_OFFSET);
	pkt_len->min_pkt_length = (uint8_t)(value &
			XCMAC_CONFIG_TX_OTN_MIN_PKT_LEN_MASK);
	pkt_len->max_pkt_length = (uint16_t)(value &
			XCMAC_CONFIG_TX_OTN_MAX_PKT_LEN_MASK)
			>> XCMAC_CONFIG_TX_OTN_MAX_PKT_LEN_BIT;

	return 0;
}

/*************************************************************************/
/**
 * This function sets the OTN control configurations
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    otn_ctl_cfg is a pointer to hold Tx OTN control configuration
 *           values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 **************************************************************************/
int xcmac_set_otn_control_config(struct xcmac *instance,
		struct xcmac_tx_otn_ctl_config *otn_ctl_cfg)
{
	uint32_t value = 0;

	is_null(instance);
	is_null(otn_ctl_cfg);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_CONFIG_TX_OTN_CTL_REG_OFFSET);

	/* clear the check sfd field */
	value = value & ~(XCMAC_CONFIG_TX_OTN_CTL_CHECK_SFD_MASK);
	/* update check sfd field */
	value |= otn_ctl_cfg->check_sfd;

	/* clear the check preamble bit */
	value = value & ~(XCMAC_CONFIG_TX_OTN_CTL_CHECK_PREAMBLE_MASK);
	/* set/unset check preamble bit */
	value |= (((uint32_t)otn_ctl_cfg->check_preamble)
		<< XCMAC_CONFIG_TX_OTN_CTL_CHECK_PREAMBLE_BIT);

	/* clear the ignore fcs bit */
	value = value & ~(XCMAC_CONFIG_TX_OTN_CTL_IGNORE_FCS_MASK);
	/* set/unset ignore fcs bit */
	value |= (((uint32_t)otn_ctl_cfg->ignore_fcs)
		<< XCMAC_CONFIG_TX_OTN_CTL_IGNORE_FCS_BIT);

	xcmac_out32(instance->base_address + XCMAC_CONFIG_TX_OTN_CTL_REG_OFFSET,
		value);

	return 0;
}

/*************************************************************************/
/**
 * This function get the OTN control configurations
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    otn_ctl_cfg is a pointer to hold Tx OTN control configuration
 *           values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 **************************************************************************/
int xcmac_get_otn_control_config(struct xcmac *instance,
		struct xcmac_tx_otn_ctl_config *otn_ctl_cfg)
{
	uint32_t value = 0;

	is_null(instance);
	is_null(otn_ctl_cfg);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_CONFIG_TX_OTN_CTL_REG_OFFSET);
	otn_ctl_cfg->check_sfd = (uint8_t)((value &
			XCMAC_CONFIG_TX_OTN_CTL_CHECK_SFD_MASK)
			>> XCMAC_CONFIG_TX_OTN_CTL_CHECK_SFD_BIT);
	otn_ctl_cfg->check_preamble = (uint8_t)(
			(value & XCMAC_CONFIG_TX_OTN_CTL_CHECK_PREAMBLE_MASK) >>
			XCMAC_CONFIG_TX_OTN_CTL_CHECK_PREAMBLE_BIT);
	otn_ctl_cfg->ignore_fcs = (uint8_t)((value
			& XCMAC_CONFIG_TX_OTN_CTL_IGNORE_FCS_MASK)
			>> XCMAC_CONFIG_TX_OTN_CTL_IGNORE_FCS_BIT);

	return 0;
}

/*************************************************************************/
/**
 * This function sets GT loopback configurations
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    loopback_type is a pointer to hold gt loopback type.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 **************************************************************************/
int xcmac_set_gt_loopback_config(struct xcmac *instance,
		enum xcmac_gt_loopback_type loopback_type)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	if (loopback_type > XCMAC_GT_LOOPBACK_INTERNAL)
		return -EINVAL;

	xcmac_out32(instance->base_address + XCMAC_GT_LOOPBACK_REG_OFFSET,
			(loopback_type & XCMAC_GT_LOOPBACK_CTL_BIT));

	return 0;
}

/*************************************************************************/
/**
 * This function sets GT loopback configurations
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    loopback_type is a pointer to hold gt loopback type.
 *
 * @return current GT loopback mode
 *
 **************************************************************************/
enum xcmac_gt_loopback_type xcmac_get_gt_loopback_config(struct xcmac *instance)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return XCMAC_GT_LOOPBACK_INVALID;

	return read_mem_with_mask(XCMAC_GT_LOOPBACK_REG_OFFSET,
			XCMAC_GT_LOOPBACK_CTL_BIT);
}

/*************************************************************************/
/**
 * This function configures the Tx flow control.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    flow_control hold flow control configuration values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_set_tx_flow_control_config(struct xcmac *instance,
		struct xcmac_tx_flow_control *flow_control)
{
	uint32_t refresh_val = 0, quanta_val = 0, index = 0;
	uint32_t offset_1 = XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG1_OFFSET;
	uint32_t offset_2 = XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG1_OFFSET;

	is_null(instance);
	is_null(flow_control);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;
	/* set refresh intervals and set quantas */
	for (index = 0; index < TOTAL_TX_PAUSE_REFRESH_TIMERS; index += 2) {
		refresh_val = flow_control->refresh_interval[index];
		quanta_val = flow_control->pause_quanta[index];
		if (index != TOTAL_TX_PAUSE_REFRESH_TIMERS) {
			/* refresh */
			refresh_val <<= SIXTEEN_BITS;
			refresh_val = refresh_val |
				flow_control->refresh_interval[index + 1];
			/* quanta */
			quanta_val <<= SIXTEEN_BITS;
			quanta_val = quanta_val |
				flow_control->pause_quanta[index + 1];
		}
		/* This sets the retransmission time of pause packets for
		 * priority 0 and priority 1 and 2, 3 and so on
		 */
		xcmac_out32(instance->base_address + offset_1, refresh_val);
		xcmac_out32(instance->base_address + offset_2, quanta_val);
		offset_1 += NUM_WORD_BYTES;
		offset_2 += NUM_WORD_BYTES;
	}
	/* set CTL_TX_PAUSE_ENABLE bits */
	refresh_val = xcmac_in32(instance->base_address +
	XCMAC_CONFIG_TX_FLOW_CONTROL_REG1_OFFSET);
	/* clear the pause enable bit */
	refresh_val = refresh_val & (~XCMAC_CONFIG_TX_FLOW_CONTROL_REG1_MASK);
	/* set/unset the pause enable bit */
	refresh_val = refresh_val | flow_control->pause_enable;
	xcmac_out32(instance->base_address +
	XCMAC_CONFIG_TX_FLOW_CONTROL_REG1_OFFSET,
			refresh_val);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current the Tx flow control configuration.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    flow_control is a pointer to hold flow control configuration
 *           values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_get_tx_flow_control_config(struct xcmac *instance,
		struct xcmac_tx_flow_control *flow_control)
{
	uint32_t refresh = 0, quanta = 0, index = 0;
	uint32_t offset_1 = XCMAC_CONFIG_TX_FLOW_CONTROL_REFRESH_REG1_OFFSET;
	uint32_t offset_2 = XCMAC_CONFIG_TX_FLOW_CONTROL_QUANTA_REG1_OFFSET;

	is_null(instance);
	is_null(flow_control);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	/* read quanta and refresh timer values */
	for (index = 0; index < TOTAL_TX_PAUSE_REFRESH_TIMERS; index += 2) {
		refresh = xcmac_in32(instance->base_address + offset_1);
		quanta = xcmac_in32(instance->base_address + offset_2);

		flow_control->refresh_interval[index] = refresh & TWO_LSB_MASK;
		flow_control->pause_quanta[index] = quanta & TWO_LSB_MASK;

		if (index != TOTAL_TX_PAUSE_REFRESH_TIMERS) {
			flow_control->refresh_interval[index + 1] =
					(uint16_t)(refresh >> SIXTEEN_BITS);
			flow_control->pause_quanta[index + 1] =
					(uint16_t)(quanta >> SIXTEEN_BITS);
		}
		offset_1 += NUM_WORD_BYTES;
		offset_2 += NUM_WORD_BYTES;
	}
	refresh = xcmac_in32(instance->base_address +
	XCMAC_CONFIG_TX_FLOW_CONTROL_REG1_OFFSET);
	/* get only pause enable bits */
	refresh = refresh & XCMAC_CONFIG_TX_FLOW_CONTROL_REG1_MASK;
	flow_control->pause_enable = (uint16_t)refresh;
	return 0;
}

/*************************************************************************/
/**
 * This function sets the current the Rx flow control pause enable status.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    enable is a value to represent to enable or disable.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_set_rx_flow_control_pause_enable(struct xcmac *instance,
		uint16_t enable)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET);
	/* clear all bits of pause enable field */
	value = value & (~XCMAC_CTL_RX_PAUSE_ENABLE_MASK);
	value = value | enable;
	xcmac_out32(instance->base_address +
	CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET,
			value);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current Rx flow control pause enable status.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return
 * - Rx flow control pause enable status
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
uint16_t xcmac_get_rx_flow_control_pause_enable(struct xcmac *instance)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET);
	return (value & XCMAC_CTL_RX_PAUSE_ENABLE_MASK);
}

/*************************************************************************/
/**
 * This function configures the Rx packet priority and pause ack.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    ack is used to set pause ack field.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_set_rx_flow_control_pause_ack(struct xcmac *instance, uint16_t ack)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET);
	/* clear all bits of pause enable ack field */
	value = value & (~XCMAC_CTL_RX_PAUSE_ACK_ENABLE_MASK);
	value = value | (ack << XCMAC_CTL_RX_PAUSE_ACK_ENABLE_BIT);
	xcmac_out32(instance->base_address +
	CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET,
			value);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current rx packet priority and pause ack status.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return
 * - Rx flow control pause enable ack status
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
uint16_t xcmac_get_rx_flow_control_pause_ack(struct xcmac *instance)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET);
	/* retain all bits of pause enable ack field */
	value = value & XCMAC_CTL_RX_PAUSE_ACK_ENABLE_MASK;
	return (uint16_t)(value >> XCMAC_CTL_RX_PAUSE_ACK_ENABLE_BIT);
}

/*************************************************************************/
/**
 * This function configures the Rx flow control packet types and enable or
 * disables the Rx flow control parameters.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    packet holds packet types parameters.
 * @param    flow_control holds flow control parameters to enable or
 *           disable
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note This function will enable gcp, pcp, gpp, ppp.
 *
 **************************************************************************/
int xcmac_set_rx_flow_control_packet_config(
		struct xcmac *instance,
		enum xcmac_flow_control_packet_type packet,
		struct xcmac_rx_flow_control *flow_control)
{
	uint32_t value = 0, check_value = 0;

	is_null(instance);
	is_null(flow_control);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	/* used to set ctl_rx_pause_enable */
	value = xcmac_in32(instance->base_address +
	CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET);
	check_value = xcmac_in32(instance->base_address +
	CONFIGURATION_RX_FLOW_CONTROL_REG2_OFFSET);
	switch (packet) {
	case XCMAC_FC_GLOBAL_CONTROL_PACKET:
		if (flow_control->enable == true)
			set_bit(value, XCMAC_CTL_RX_ENABLE_GCP_BIT);
		else
			clear_bit(value, XCMAC_CTL_RX_ENABLE_GCP_BIT);

		xcmac_out32(instance->base_address +
		CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET,
				value);
		/* multicast, unicast, source_address, ether_type,
		 * opcode enable/disable
		 */
		if (flow_control->check_multicast == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_MCAST_GCP_BIT);

		if (flow_control->check_unicast == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_UCAST_GCP_BIT);

		if (flow_control->check_source_address == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_SA_GCP_BIT);

		if (flow_control->check_ether_type == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_ETYPE_GCP_BIT);

		if (flow_control->check_opcode == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_OPCODE_GCP_BIT);

		xcmac_out32(instance->base_address +
		CONFIGURATION_RX_FLOW_CONTROL_REG2_OFFSET,
				check_value);
		break;
	case XCMAC_FC_PRIORITY_CONTROL_PACKET:
		if (flow_control->enable == true)
			set_bit(value, XCMAC_CTL_RX_ENABLE_PCP_BIT);
		else
			clear_bit(value, XCMAC_CTL_RX_ENABLE_PCP_BIT);

		xcmac_out32(instance->base_address +
		CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET,
				value);
		if (flow_control->check_multicast == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_MCAST_PCP_BIT);

		if (flow_control->check_unicast == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_UCAST_PCP_BIT);

		if (flow_control->check_source_address == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_SA_PCP_BIT);

		if (flow_control->check_ether_type == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_ETYPE_PCP_BIT);

		if (flow_control->check_opcode == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_OPCODE_PCP_BIT);

		xcmac_out32(instance->base_address +
		CONFIGURATION_RX_FLOW_CONTROL_REG2_OFFSET,
				check_value);
		break;
	case XCMAC_FC_GLOBAL_PAUSE_PACKET:
		if (flow_control->enable == true)
			set_bit(value, XCMAC_CTL_RX_ENABLE_GPP_BIT);
		else
			clear_bit(value, XCMAC_CTL_RX_ENABLE_GPP_BIT);

		xcmac_out32(instance->base_address +
		CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET,
				value);
		if (flow_control->check_multicast == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_MCAST_GPP_BIT);

		if (flow_control->check_unicast == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_UCAST_GPP_BIT);

		if (flow_control->check_source_address == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_SA_GPP_BIT);

		if (flow_control->check_ether_type == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_ETYPE_GPP_BIT);

		if (flow_control->check_opcode == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_OPCODE_GPP_BIT);

		xcmac_out32(instance->base_address +
		CONFIGURATION_RX_FLOW_CONTROL_REG2_OFFSET,
				check_value);
		break;
	case XCMAC_FC_PRIORITY_PAUSE_PACKET:
		if (flow_control->enable == true)
			set_bit(value, XCMAC_CTL_RX_ENABLE_PPP_BIT);
		else
			clear_bit(value, XCMAC_CTL_RX_ENABLE_PPP_BIT);

		xcmac_out32(instance->base_address +
		CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET,
				value);
		if (flow_control->check_multicast == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_MCAST_PPP_BIT);

		if (flow_control->check_unicast == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_UCAST_PPP_BIT);

		if (flow_control->check_source_address == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_SA_PPP_BIT);

		if (flow_control->check_ether_type == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_ETYPE_PPP_BIT);

		if (flow_control->check_opcode == true)
			set_bit(check_value, XCMAC_CTL_RX_CHECK_OPCODE_PPP_BIT);

		xcmac_out32(instance->base_address +
		CONFIGURATION_RX_FLOW_CONTROL_REG2_OFFSET,
				check_value);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current Rx flow control packet configuration.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    packet holds packet types parameters.
 * @param    flow_control is pointer to hold flow control parameters to
 *           enable or disable
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_rx_flow_control_packet_config(
		struct xcmac *instance,
		enum xcmac_flow_control_packet_type packet,
		struct xcmac_rx_flow_control *flow_control)
{
	uint32_t value = 0;

	is_null(instance);
	is_null(flow_control);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	CONFIGURATION_RX_FLOW_CONTROL_REG1_OFFSET);
	switch (packet) {
	case XCMAC_FC_GLOBAL_CONTROL_PACKET:
		flow_control->enable = (value &
				XCMAC_CTL_RX_ENABLE_GCP_BIT_MASK) >>
				XCMAC_CTL_RX_ENABLE_GCP_BIT;
		flow_control->check_multicast = (value &
				XCMAC_CTL_RX_CHECK_MCAST_GCP_BIT_MASK) >>
				XCMAC_CTL_RX_CHECK_MCAST_GCP_BIT;
		flow_control->check_unicast = (value &
				XCMAC_CTL_RX_CHECK_UCAST_GCP_BIT_MASK) >>
				XCMAC_CTL_RX_CHECK_UCAST_GCP_BIT;
		flow_control->check_source_address =
				(value & XCMAC_CTL_RX_CHECK_SA_GCP_BIT_MASK) >>
				XCMAC_CTL_RX_CHECK_SA_GCP_BIT;
		flow_control->check_multicast =
				(value & XCMAC_CTL_RX_CHECK_ETYPE_GCP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_ETYPE_GCP_BIT;
		flow_control->check_opcode =
				(value & XCMAC_CTL_RX_CHECK_OPCODE_GCP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_OPCODE_GCP_BIT;
		break;
	case XCMAC_FC_PRIORITY_CONTROL_PACKET:
		flow_control->enable =
				(value & XCMAC_CTL_RX_ENABLE_PCP_BIT_MASK) >>
				XCMAC_CTL_RX_ENABLE_PCP_BIT;
		/*
		 * multicast, unicast, source_address, ether_type,
		 * opcode enable/disable
		 */
		flow_control->check_multicast =
				(value & XCMAC_CTL_RX_CHECK_MCAST_PCP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_MCAST_PCP_BIT;
		flow_control->check_unicast =
				(value & XCMAC_CTL_RX_CHECK_UCAST_PCP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_UCAST_PCP_BIT;
		flow_control->check_source_address =
				(value & XCMAC_CTL_RX_CHECK_SA_PCP_BIT_MASK) >>
				XCMAC_CTL_RX_CHECK_SA_PCP_BIT;
		flow_control->check_multicast =
				(value & XCMAC_CTL_RX_CHECK_ETYPE_PCP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_ETYPE_PCP_BIT;
		flow_control->check_opcode =
				(value & XCMAC_CTL_RX_CHECK_OPCODE_PCP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_OPCODE_PCP_BIT;
		break;
	case XCMAC_FC_GLOBAL_PAUSE_PACKET:
		flow_control->enable =
				(value & XCMAC_CTL_RX_ENABLE_GPP_BIT_MASK) >>
				XCMAC_CTL_RX_ENABLE_GPP_BIT;
		flow_control->check_multicast =
				(value & XCMAC_CTL_RX_CHECK_MCAST_GPP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_MCAST_GPP_BIT;
		flow_control->check_unicast =
				(value & XCMAC_CTL_RX_CHECK_UCAST_GPP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_UCAST_GPP_BIT;
		flow_control->check_source_address =
				(value & XCMAC_CTL_RX_CHECK_SA_GPP_BIT_MASK) >>
				XCMAC_CTL_RX_CHECK_SA_GPP_BIT;
		flow_control->check_multicast =
				(value & XCMAC_CTL_RX_CHECK_ETYPE_GPP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_ETYPE_GPP_BIT;
		flow_control->check_opcode =
				(value & XCMAC_CTL_RX_CHECK_OPCODE_GPP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_OPCODE_GPP_BIT;
		break;
	case XCMAC_FC_PRIORITY_PAUSE_PACKET:
		flow_control->enable =
				(value & XCMAC_CTL_RX_ENABLE_PPP_BIT_MASK) >>
				XCMAC_CTL_RX_ENABLE_PPP_BIT;
		flow_control->check_multicast =
				(value & XCMAC_CTL_RX_CHECK_MCAST_PPP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_MCAST_GCP_BIT;
		flow_control->check_unicast =
				(value & XCMAC_CTL_RX_CHECK_UCAST_PPP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_UCAST_PPP_BIT;
		flow_control->check_source_address =
				(value & XCMAC_CTL_RX_CHECK_SA_PPP_BIT_MASK) >>
				XCMAC_CTL_RX_CHECK_SA_PPP_BIT;
		flow_control->check_multicast =
				(value & XCMAC_CTL_RX_CHECK_ETYPE_PPP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_ETYPE_PPP_BIT;
		flow_control->check_opcode =
				(value & XCMAC_CTL_RX_CHECK_OPCODE_PPP_BIT_MASK)
				>> XCMAC_CTL_RX_CHECK_OPCODE_PPP_BIT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current Tx status.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    tx_status is a pointer to hold Tx status parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note STAT_RX_STATUS_REG and STAT_STATUS_REG1 both register stats are
 * combined in this function.
 **************************************************************************/
int xcmac_get_tx_status(struct xcmac *instance,
		struct xcmac_tx_status *tx_status)
{
	uint8_t value = 0;

	is_null(instance);
	is_null(tx_status);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_STAT_TX_STATUS_OFFSET);
	tx_status->local_fault = (value & XCMAC_STAT_TX_LOCAL_FAULT_MASK) >>
	XCMAC_STAT_TX_LOCAL_FAULT_BIT;
	value = xcmac_in32(instance->base_address + XCMAC_STAT_STATUS_OFFSET);
	tx_status->ptp_fifo_read_error =
			(value & XCMAC_STAT_TX_PTP_FIFO_R_ERR_MASK) >>
			XCMAC_STAT_TX_PTP_FIFO_R_ERR_BIT;
	tx_status->ptp_fifo_write_error =
			(value & XCMAC_STAT_TX_PTP_FIFO_W_ERR_MASK) >>
			XCMAC_STAT_TX_PTP_FIFO_W_ERR_BIT;
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current Rx lane status.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    rx_lane_status is a pointer to hold Rx status parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_rx_lane_status(struct xcmac *instance,
		struct xcmac_rx_lane_am_status *rx_lane_status)
{
	uint32_t block_lock = 0, sync = 0, sync_err = 0, lm_err = 0,
			lm_len_err = 0;
	uint32_t lm_repeat_err = 0, lane = 0, rx_status = 0;

	is_null(instance);
	is_null(rx_lane_status);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	rx_status = xcmac_in32(instance->base_address +
	XCMAC_STAT_RX_STATUS_OFFSET);
	rx_lane_status->pcs_status =
			(rx_status & XCMAC_STAT_RX_MASK) ? true : false;
	rx_lane_status->aligned =
			(rx_status & XCMAC_STAT_RX_ALIGNED_MASK) ? true : false;
	rx_lane_status->mis_aligned =
			(rx_status & XCMAC_STAT_RX_MISALIGNED_MASK) ?
					true : false;
	rx_lane_status->aligned_error =
			(rx_status & XCMAC_STAT_RX_ALIGNED_ERR_MASK) ?
					true : false;
	block_lock = read_mem_with_mask(XCMAC_STAT_RX_BLOCK_LOCK_OFFSET,
			XCMAC_STAT_RX_BLOCK_LOCK_MASK);
	sync = read_mem_with_mask(XCMAC_STAT_RX_SYNCED_OFFSET,
			XCMAC_STAT_RX_SYNCED_MASK);
	sync_err = read_mem_with_mask(XCMAC_STAT_RX_SYNCED_ERR_OFFSET,
			XCMAC_STAT_RX_SYNCED_ERR_MASK);
	lm_err = read_mem_with_mask(XCMAC_STAT_RX_MF_ERR_OFFSET,
			XCMAC_STAT_RX_MF_ERR_MASK);
	lm_len_err = read_mem_with_mask(XCMAC_STAT_RX_MF_LEN_ERR_OFFSET,
			XCMAC_STAT_RX_MF_LEN_ERR_MASK);
	lm_repeat_err = read_mem_with_mask(XCMAC_STAT_RX_MF_REPEAT_ERR_OFFSET,
			XCMAC_STAT_RX_MF_REPEAT_ERR_MASK);
	for (lane = 0; lane < XCMAC_PCS_LANE_COUNT; lane++) {
		rx_lane_status->block_lock[lane] = (uint8_t)(block_lock & 1);
		block_lock = block_lock >> 1;
		rx_lane_status->synced[lane] = (uint8_t)(sync & 1);
		sync = sync >> 1;
		rx_lane_status->synced_error[lane] = (uint8_t)(sync_err & 1);
		sync_err = sync_err >> 1;
		rx_lane_status->lane_marker_error[lane] = (uint8_t)(lm_err & 1);
		lm_err = lm_err >> 1;
		rx_lane_status->lane_marker_len_error[lane] =
				(uint8_t)(lm_len_err & 1);
		lm_len_err = lm_len_err >> 1;
		rx_lane_status->lane_marker_repeat_error[lane] =
				(uint8_t)(lm_repeat_err & 1);
		lm_repeat_err = lm_repeat_err >> 1;
	}
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current rx fault status.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    rx_fault_status is a pointer to hold rx fault status parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_rx_fault_status(struct xcmac *instance,
		struct xcmac_rx_fault_status *rx_fault_status)
{
	uint32_t value = 0;

	is_null(instance);
	is_null(rx_fault_status);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_STAT_RX_STATUS_OFFSET);
	rx_fault_status->remote_fault =
			(value & XCMAC_STAT_RX_REMOTE_FAULT_MASK) >>
			XCMAC_STAT_RX_REMOTE_FAULT_BIT;
	rx_fault_status->local_fault =
			(value & XCMAC_STAT_RX_LOCAL_FAULT_MASK) >>
			XCMAC_STAT_RX_LOCAL_FAULT_BIT;
	rx_fault_status->internal_local_fault =
			(value & XCMAC_STAT_RX_INTERNAL_LOCAL_FAULT_MASK) >>
			XCMAC_STAT_RX_INTERNAL_LOCAL_FAULT_BIT;
	rx_fault_status->received_local_fault =
			(value & XCMAC_STAT_RX_RECEIVED_LOCAL_FAULT_MASK) >>
			XCMAC_STAT_RX_RECEIVED_LOCAL_FAULT_BIT;
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current Rx packet status.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    rx_packet_status is a pointer to hold rx packet status parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_rx_packet_status(struct xcmac *instance,
		struct xcmac_rx_packet_status *rx_packet_status)
{
	uint32_t value = 0;

	is_null(instance);
	is_null(rx_packet_status);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_STAT_RX_STATUS_OFFSET);
	rx_packet_status->hi_ber =
			(uint8_t)((value & XCMAC_STAT_RX_HI_BER_MASK) >>
			XCMAC_STAT_RX_HI_BER_BIT);
	rx_packet_status->bad_preamble =
			(uint8_t)((value & XCMAC_STAT_RX_BAD_PREAMBLE_MASK) >>
			XCMAC_STAT_RX_BAD_PREAMBLE_BIT);
	rx_packet_status->bad_sfd =
			(uint8_t)((value & XCMAC_STAT_RX_BAD_SFD_MASK) >>
			XCMAC_STAT_RX_BAD_SFD_BIT);
	rx_packet_status->got_signal_os =
			(uint8_t)((value & XCMAC_STAT_RX_GOT_SIGNAL_OS_MASK) >>
			XCMAC_STAT_RX_GOT_SIGNAL_OS_BIT);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current Rx vl demux status of each lane.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    rx_vldemux_status is to hold Rx vl demux status parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_rx_vldemux_status(
		struct xcmac *instance,
		struct xcmac_rx_vldemux_status *rx_vldemux_status)
{
	uint32_t vldemuxed = 0, vlnumber = 0, offset = 0, mask = 0;
	uint32_t lane = 0, bit_num = 0;

	is_null(instance);
	is_null(rx_vldemux_status);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	/* vl demux status */
	vldemuxed = read_mem_with_mask(XCMAC_STAT_RX_VL_DEMUXED_OFFSET,
			STAT_RX_VL_DEMUXED_MASK);
	for (lane = 0; lane < XCMAC_PCS_LANE_COUNT; lane++) {
		rx_vldemux_status->vldemuxed[lane] = (uint8_t)(vldemuxed & 1);
		vldemuxed = vldemuxed >> 1;
	}
	for (lane = 0; lane < XCMAC_PCS_LANE_COUNT; lane++, bit_num += 5) {
		/* 0 to 4 bits for lane 0
		 * 5 to 9 bits for lane 1
		 * 10 to 14 bits for lane 2
		 * 15 to 19 bits for lane 3
		 * 20 to 24 bits for lane 4
		 * 25 to 29 bits for lane 5
		 */
		if (lane == 0) {
			offset = XCMAC_STAT_RX_VL_NUM_0_OFFSET;
			/* reset bit number */
			bit_num = 0;
			mask = XCMAC_STAT_RX_VL_NUM_0_MASK;
		}
		if (lane == 6) {
			offset = XCMAC_STAT_RX_VL_NUM_6_OFFSET;
			/* reset bit number */
			bit_num = 0;
			mask = XCMAC_STAT_RX_VL_NUM_6_MASK;
		}
		if (lane == 12) {
			offset = XCMAC_STAT_RX_VL_NUM_12_OFFSET;
			/* reset bit number */
			bit_num = 0;
			mask = XCMAC_STAT_RX_VL_NUM_12_MASK;
		}
		if (lane == 18) {
			offset = XCMAC_STAT_RX_VL_NUM_18_OFFSET;
			/* reset bit number */
			bit_num = 0;
			mask = XCMAC_STAT_RX_VL_NUM_18_MASK;
		}
		vlnumber = read_mem_with_mask(offset, mask);
		rx_vldemux_status->vlnumber[lane++] =
				(vlnumber >> bit_num)
						& XCMAC_STAT_RX_VL_NUM_MASK;
	}
	return 0;
}

/*************************************************************************/
/**
 * This function reads the current Rx test pattern mismatch status.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return   Rx pattern mismatch status
 *
 * @note none.
 **************************************************************************/
uint8_t xcmac_get_rx_test_pattern_status(struct xcmac *instance)
{
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	value = xcmac_in32(instance->base_address +
	XCMAC_STAT_RX_STATUS_OFFSET);
	return (uint8_t)((value & XCMAC_STAT_RX_TEST_PATTERN_MISMATCH_MASK) >>
	XCMAC_STAT_RX_TEST_PATTERN_MISMATCH_BIT);
}

/*************************************************************************/
/**
 * This function issues tick so that statistics are copied to readable registers
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_latch_statistics(struct xcmac *instance)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	/* write of 1 will trigger a snapshot of counters into registers */
	xcmac_out32(instance->base_address + XCMAC_TICK_REG_OFFSET,
	XCMAC_TICK_REGISTER_EN_MASK);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the cycle count, a count of the number of serdes clock
 * cycles between tick_reg register writes. the values in the
 * counters are held until the next write to the tick_reg register.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return
 *
 * - 64 bit cycle count
 *
 * @note The STAT_CYCLE_COUNT_MSB/LSB register contains a count of the number
 * of SerDes clock cycles between TICK_REG register writes.
 **************************************************************************/
uint64_t xcmac_get_cycle_count(struct xcmac *instance)
{
	is_null(instance);
	is_null(instance->is_ready == XIL_COMPONENT_IS_READY);

	return xcmac_read_48bit_value(instance, XCMAC_STAT_CYCLE_COUNT_OFFSET);
}

/*************************************************************************/
/**
 * This function reads a list of values representing the parity errors for
 * each lane in the system
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    bip_err_count holds the list of values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_get_rx_bip_error_count(struct xcmac *instance,
		uint64_t bip_err_count[XCMAC_PCS_LANE_COUNT])
{
	int index = 0;
	uint32_t offset = 0;

	is_null(instance);
	is_null(bip_err_count);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	offset = XCMAC_STAT_RX_BIP_ERR_0_OFFSET;
	for (index = 0; index < XCMAC_PCS_LANE_COUNT; index++) {
		bip_err_count[index] = xcmac_read_48bit_value(instance, offset);
		offset += NUM_DWORD_BYTES;
	}
	return 0;
}

/*************************************************************************/
/**
 * This function reads the list of values representing the receive framing
 * error counters.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    framing_err_count holds the list of framing error count values.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_rx_framing_error_count(
		struct xcmac *instance,
		uint64_t framing_err_count[XCMAC_PCS_LANE_COUNT])
{
	int index = 0;
	uint32_t offset = 0;

	is_null(instance);
	is_null(framing_err_count);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	offset = XCMAC_STAT_RX_FRAMING_ERR_0_OFFSET;
	for (index = 0; index < XCMAC_PCS_LANE_COUNT; index++) {
		framing_err_count[index] =
				xcmac_read_48bit_value(instance, offset);
		offset += NUM_DWORD_BYTES;
	}
	return 0;
}

/*************************************************************************/
/**
 * This function reads the bad code count.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return 64 bit Bad code value
 *
 * @note none.
 **************************************************************************/
uint64_t xcmac_get_rx_bad_64b66b_code_count(struct xcmac *instance)
{
	is_null(instance);
	return xcmac_read_48bit_value(instance, XCMAC_STAT_RX_BAD_CODE_OFFSET);
}

/*************************************************************************/
/**
 * This function reads the bad code count.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return 64 bit Bad code value
 *
 * @note none.
 **************************************************************************/
uint64_t xcmac_get_tx_bad_64b66b_code_count(struct xcmac *instance)
{
	is_null(instance);
	return xcmac_read_48bit_value(instance, XCMAC_STAT_TX_BAD_CODE_OFFSET);
}

/*************************************************************************/
/**
 * This function reads the frame error count.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return   64 bit Tx frame error count
 *
 * @note none.
 **************************************************************************/
uint64_t xcmac_get_tx_frame_error_count(struct xcmac *instance)
{
	is_null(instance);
	return xcmac_read_48bit_value(instance,
	XCMAC_STAT_TX_FRAME_ERROR_OFFSET);
}

/*************************************************************************/
/**
 * This function reads the counters of different packet length for tx.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    packet_stats is a pointer to hold different packet counts.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_get_tx_packet_statistics(
		struct xcmac *instance,
		struct xcmac_packet_count_statistics *packet_stats)
{
	is_null(instance);
	is_null(packet_stats);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	packet_stats->total_packets = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_TOTAL_PACKETS_OFFSET);
	packet_stats->total_good_packets = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_TOTAL_GOOD_PACKETS_OFFSET);
	packet_stats->total_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_TOTAL_BYTES_OFFSET);
	packet_stats->total_good_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_TOTAL_GOOD_BYTES_OFFSET);
	packet_stats->packets_0_to_64_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_64_BYTES_OFFSET);
	packet_stats->packets_65_to_127_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_65_127_BYTES_OFFSET);
	packet_stats->packets_128_to_255_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_128_255_BYTES_OFFSET);
	packet_stats->packets_256_to_511_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_256_511_BYTES_OFFSET);
	packet_stats->packets_512_to_1023_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_512_1023_BYTES_OFFSET);
	packet_stats->packets_1024_to_1518_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_1024_1518_BYTES_OFFSET);
	packet_stats->packets_1519_to_1522_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_1519_1522_BYTES_OFFSET);
	packet_stats->packets_1523_to_1548_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_1523_1548_BYTES_OFFSET);
	packet_stats->packets_1549_to_2047_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_1549_2047_BYTES_OFFSET);
	packet_stats->packets_2048_to_4095_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_2048_4095_BYTES_OFFSET);
	packet_stats->packets_4096_to_8191_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_4096_8191_BYTES_OFFSET);
	packet_stats->packets_8192_to_9215_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_8192_9215_BYTES_OFFSET);
	packet_stats->packets_large = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_LARGE_OFFSET);
	packet_stats->packets_small = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_SMALL_OFFSET);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the counters of different packet length for rx.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    packet_stats is a pointer to hold different packet counts.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 *
 **************************************************************************/
int xcmac_get_rx_packet_statistics(
		struct xcmac *instance,
		struct xcmac_packet_count_statistics *packet_stats)
{
	is_null(instance);
	is_null(packet_stats);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	packet_stats->total_packets = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_TOTAL_PACKETS_OFFSET);
	packet_stats->total_good_packets = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_TOTAL_GOOD_PACKETS_OFFSET);
	packet_stats->total_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_TOTAL_BYTES_OFFSET);
	packet_stats->total_good_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_TOTAL_GOOD_BYTES_OFFSET);
	packet_stats->packets_0_to_64_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_64_BYTES_OFFSET);
	packet_stats->packets_65_to_127_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_65_127_BYTES_OFFSET);
	packet_stats->packets_128_to_255_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_128_255_BYTES_OFFSET);
	packet_stats->packets_256_to_511_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_256_511_BYTES_OFFSET);
	packet_stats->packets_512_to_1023_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_512_1023_BYTES_OFFSET);
	packet_stats->packets_1024_to_1518_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_1024_1518_BYTES_OFFSET);
	packet_stats->packets_1519_to_1522_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_1519_1522_BYTES_OFFSET);
	packet_stats->packets_1523_to_1548_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_1523_1548_BYTES_OFFSET);
	packet_stats->packets_1549_to_2047_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_1549_2047_BYTES_OFFSET);
	packet_stats->packets_2048_to_4095_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_2048_4095_BYTES_OFFSET);
	packet_stats->packets_4096_to_8191_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_4096_8191_BYTES_OFFSET);
	packet_stats->packets_8192_to_9215_bytes = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_8192_9215_BYTES_OFFSET);
	packet_stats->packets_large = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_LARGE_OFFSET);
	packet_stats->packets_small = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_SMALL_OFFSET);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the malformed packets count for tx.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    malformed_stats is a pointer to hold malformed packet counts.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_tx_malformed_statistics(
		struct xcmac *instance,
		struct xcmac_malformed_statistics *malformed_stats)
{
	is_null(instance);
	is_null(malformed_stats);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	malformed_stats->under_size_count = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_UNDERSIZE_OFFSET);
	malformed_stats->fragment_count =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_TX_FRAGMENT_OFFSET);
	malformed_stats->oversize_count =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_TX_OVERSIZE_OFFSET);
	malformed_stats->toolong_count =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_TX_TOOLONG_OFFSET);
	malformed_stats->jabber_count =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_TX_JABBER_OFFSET);
	malformed_stats->bad_fcs_count =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_TX_BAD_FCS_OFFSET);
	malformed_stats->packet_bad_fcs_count = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_PACKET_BAD_FCS_OFFSET);
	malformed_stats->stomped_fcs_count = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_STOMPED_FCS_OFFSET);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the malformed packets count for Rx.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    malformed_stats is a pointer to hold malformed packet counts.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_rx_malformed_statistics(
		struct xcmac *instance,
		struct xcmac_malformed_statistics *malformed_stats)
{
	is_null(instance);
	is_null(malformed_stats);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	malformed_stats->under_size_count = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_UNDERSIZE_OFFSET);
	malformed_stats->fragment_count =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_RX_FRAGMENT_OFFSET);
	malformed_stats->oversize_count =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_RX_OVERSIZE_OFFSET);
	malformed_stats->toolong_count =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_RX_TOOLONG_OFFSET);
	malformed_stats->jabber_count =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_RX_JABBER_OFFSET);
	malformed_stats->bad_fcs_count =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_RX_BAD_FCS_OFFSET);
	malformed_stats->packet_bad_fcs_count = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_PACKET_BAD_FCS_OFFSET);
	malformed_stats->stomped_fcs_count = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_STOMPED_FCS_OFFSET);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the counters of different packet types for tx.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    packet_type_stats to hold different packet type counts.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_tx_packet_type_statistics(
		struct xcmac *instance,
		struct xcmac_packet_type_statistics *packet_type_stats)
{
	is_null(instance);
	is_null(packet_type_stats);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	packet_type_stats->packets_unicast =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_TX_UNICAST_OFFSET);
	packet_type_stats->packets_multicast = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_MULTICAST_OFFSET);
	packet_type_stats->packets_broadcast = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_BROADCAST_OFFSET);
	packet_type_stats->packets_vlan =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_TX_VLAN_OFFSET);
	packet_type_stats->packets_pause =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_TX_PAUSE_OFFSET);
	packet_type_stats->packets_priority_pause = xcmac_read_48bit_value(
			instance, XCMAC_STAT_TX_USER_PAUSE_OFFSET);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the counters of different packet types for Rx.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 * @param    packet_type_stats hold different packet type counts.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_rx_packet_type_statistics(
		struct xcmac *instance,
		struct xcmac_packet_type_statistics *packet_type_stats)
{
	is_null(instance);
	is_null(packet_type_stats);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	packet_type_stats->packets_unicast =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_RX_UNICAST_OFFSET);
	packet_type_stats->packets_multicast = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_MULTICAST_OFFSET);
	packet_type_stats->packets_broadcast = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_BROADCAST_OFFSET);
	packet_type_stats->packets_vlan =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_RX_VLAN_OFFSET);
	packet_type_stats->packets_pause =
			xcmac_read_48bit_value(instance,
					XCMAC_STAT_RX_PAUSE_OFFSET);
	packet_type_stats->packets_priority_pause = xcmac_read_48bit_value(
			instance, XCMAC_STAT_RX_USER_PAUSE_OFFSET);
	return 0;
}

/*************************************************************************/
/**
 * This function reads the range error count.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return   64 bit Rx range error count
 *
 * @note none
 **************************************************************************/
uint64_t xcmac_get_rx_range_error_count(struct xcmac *instance)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	return xcmac_read_48bit_value(instance,
	XCMAC_STAT_RX_INRANGE_ERR_OFFSET);
}

/*************************************************************************/
/**
 *
 * This function reads the Rx truncated error count.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return   64 bit Rx truncated error count
 *
 * @note Packet Truncation Indicator
 *    A value of 1 indicates that
 *    the current packet in flight is truncated due to its length.
 **************************************************************************/
uint64_t xcmac_get_rx_truncated_count(struct xcmac *instance)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	return xcmac_read_48bit_value(instance, XCMAC_STAT_RX_TRUNCATED_OFFSET);
}

/*************************************************************************/
/**
 * This function de-allocates resources acquired in xcmac_initialize function.
 *
 * @param    instance is a pointer to the CMAC instance to be worked on.
 *
 * @return
 *
 * - 0 if success
 * - Error number if there is an error
 *
 * @note None
 **************************************************************************/
int xcmac_deintialize(struct xcmac *instance)
{
	is_null(instance);
	instance->is_started = 0; /* not started */
	instance->base_address = 0;
	instance->is_ready = 0;
	return 0;
}

/*************************************************************************/
/**
 * This function is used to read the autonegotiation status parameters.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    status pointer to hold autonegotiation status parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_autoneg_status(struct xcmac *instance,
		struct xcmac_autoneg_status *status)
{
	uint32_t offset = XCMAC_STAT_AN_STATUS_OFFSET;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(status);

	value = xcmac_in32(instance->base_address + offset);
	status->fec_enable =
			get_bit_status(value, XCMAC_STAT_AN_FEC_ENABLE_BIT);
	status->rs_fec_enable =
			get_bit_status(value, XCMAC_STAT_AN_RS_FEC_ENABLE_BIT);
	status->autoneg_complete =
			get_bit_status(value,
					XCMAC_STAT_AN_AUTONEG_COMPLETE_BIT);
	status->parallel_detection_fault = get_bit_status(
			value, XCMAC_STAT_AN_PARALLEL_DETECTION_FAULT_BIT);
	status->tx_pause_enable =
			get_bit_status(value,
					XCMAC_STAT_AN_TX_PAUSE_ENABLE_BIT);
	status->rx_pause_enable = get_bit_status(value,
			XCMAC_STAT_AN_RX_PAUSE_ENABLE_BIT);
	status->lp_ability_valid = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_VALID_BIT);
	status->lp_autoneg_able = get_bit_status(value,
			XCMAC_STAT_AN_LP_AUTONEG_ABLE_BIT);
	status->lp_pause = get_bit_status(value, XCMAC_STAT_AN_LP_PAUSE_BIT);
	status->lp_asm_dir = get_bit_status(value,
			XCMAC_STAT_AN_LP_ASM_DIR_BIT);
	status->lp_rf = get_bit_status(value, XCMAC_STAT_AN_LP_RF_BIT);
	status->lp_fec_10g_ability = get_bit_status(value,
			XCMAC_STAT_AN_LP_FEC_10G_ABILITY_BIT);
	status->lp_fec_10g_request = get_bit_status(value,
			XCMAC_STAT_AN_LP_FEC_10G_REQUEST_BIT);
	status->lp_ext_ability_valid = get_bit_status(
			value, XCMAC_STAT_AN_LP_EXTENDED_ABILITY_VALID_BIT);
	status->lp_ability_ext_fec = (value &
			XCMAC_STAT_AN_LP_ABILITY_EXTENDED_FEC_BIT_MASK)
			>> XCMAC_STAT_AN_LP_ABILITY_EXTENDED_FEC_BIT;
	status->lp_fec_25g_rs_ability = get_bit_status(value,
			XCMAC_STAT_AN_LP_FEC_25G_RS_ABILITY_BIT);
	status->lp_fec_25g_baser_request = get_bit_status(
			value, XCMAC_STAT_AN_LP_FEC_25G_BASER_REQUEST_BIT);

	return 0;
}

/*************************************************************************/
/**
 * This function is used to read the autonegotiation ablity parameters.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    ability pointer to hold auto negotiation ability parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_autoneg_ability(struct xcmac *instance,
		struct xcmac_autoneg_ability *ability)
{
	uint32_t offset = XCMAC_STAT_AN_ABILITY_OFFSET;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(ability);

	value = xcmac_in32(instance->base_address + offset);
	ability->ctl_autoneg_ability_1000base_kx = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_1000BASE_KX_BIT);
	ability->ctl_autoneg_ability_10gbase_kx4 = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_10GBASE_KX4_BIT);
	ability->ctl_autoneg_ability_10gbase_kr =  get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_10GBASE_KR_BIT);
	ability->ctl_autoneg_ability_40gbase_kr4 = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_40GBASE_KR4_BIT);
	ability->ctl_autoneg_ability_40gbase_cr4 = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_40GBASE_CR4_BIT);
	ability->ctl_autoneg_ability_100gbase_cr10 = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR10_BIT);
	ability->ctl_autoneg_ability_100gbase_kp4 = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_KP4_BIT);
	ability->ctl_autoneg_ability_100gbase_kr4 = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_KR4_BIT);
	ability->ctl_autoneg_ability_100gbase_cr4 = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	ability->ctl_autoneg_ability_25gbase_kr = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_25GBASE_KR_BIT);
	ability->ctl_autoneg_ability_25gbase_cr = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_25GBASE_CR_BIT);
	ability->ctl_autoneg_ability_25gbase_kr1 = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_25GBASE_KR1_BIT);
	ability->ctl_autoneg_ability_25gbase_cr1 = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_25GBASE_CR1_BIT);
	ability->ctl_autoneg_ability_50gbase_kr2 = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_50GBASE_KR2_BIT);
	ability->ctl_autoneg_ability_50gbase_cr2 = get_bit_status(value,
			XCMAC_STAT_AN_LP_ABILITY_50GBASE_CR2_BIT);

	return 0;
}

/*************************************************************************/
/**
 * This function is used to read the auto negotiation link control status.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    an_link_cntl hold auto negotiation link control parameters.
 *
 * @return
 *           - 0 if success
 *           - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_autoneg_link_ctrl(struct xcmac *instance,
		struct xcmac_autoneg_link_ctrl *an_link_cntl)
{
	uint32_t offset = XCMAC_STAT_AN_LINK_CTL_OFFSET;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(an_link_cntl);

	value = xcmac_in32(instance->base_address + offset);
	an_link_cntl->stat_an_link_cntl_1000base_kx =
			(value & XCMAC_STAT_AN_LINK_CNTL_1000BASE_KX_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_1000BASE_KX_BIT;
	an_link_cntl->stat_an_link_cntl_10gbase_kx4 =
			(value & XCMAC_STAT_AN_LINK_CNTL_10GBASE_KX4_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_10GBASE_KX4_BIT;
	an_link_cntl->stat_an_link_cntl_10gbase_kr =
			(value & XCMAC_STAT_AN_LINK_CNTL_10GBASE_KR_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_10GBASE_KR_BIT;
	an_link_cntl->stat_an_link_cntl_40gbase_kr4 =
			(value & XCMAC_STAT_AN_LINK_CNTL_40GBASE_KR4_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_40GBASE_KR4_BIT;
	an_link_cntl->stat_an_link_cntl_40gbase_cr4 =
			(value & XCMAC_STAT_AN_LINK_CNTL_40GBASE_CR4_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_40GBASE_CR4_BIT;
	an_link_cntl->stat_an_link_cntl_100gbase_cr10 =
			(value & XCMAC_STAT_AN_LINK_CNTL_100GBASE_CR10_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_100GBASE_CR10_BIT;
	an_link_cntl->stat_an_link_cntl_100gbase_kp4 =
			(value & XCMAC_STAT_AN_LINK_CNTL_100GBASE_KP4_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_100GBASE_KP4_BIT;
	an_link_cntl->stat_an_link_cntl_100gbase_kr4 =
			(value & XCMAC_STAT_AN_LINK_CNTL_100GBASE_KR4_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_100GBASE_KR4_BIT;
	an_link_cntl->stat_an_link_cntl_100gbase_cr4 =
			(value & XCMAC_STAT_AN_LINK_CNTL_100GBASE_CR4_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_100GBASE_CR4_BIT;
	an_link_cntl->stat_an_link_cntl_25gbase_krcr_s =
			(value & XCMAC_STAT_AN_LINK_CNTL_25GBASE_KRCR_S_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_25GBASE_KRCR_S_BIT;
	an_link_cntl->stat_an_link_cntl_25gbase_krcr =
			(value & XCMAC_STAT_AN_LINK_CNTL_25GBASE_KRCR_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_25GBASE_KRCR_BIT;
	an_link_cntl->stat_an_link_cntl_25gbase_kr1 =
			(value & XCMAC_STAT_AN_LINK_CNTL_25GBASE_KR1_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_25GBASE_KR1_BIT;
	an_link_cntl->stat_an_link_cntl_25gbase_cr1 =
			(value & XCMAC_STAT_AN_LINK_CNTL_25GBASE_CR1_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_25GBASE_CR1_BIT;
	an_link_cntl->stat_an_link_cntl_50gbase_kr2 =
			(value & XCMAC_STAT_AN_LINK_CNTL_50GBASE_KR2_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_50GBASE_KR2_BIT;
	an_link_cntl->stat_an_link_cntl_50gbase_cr2 =
			(value & XCMAC_STAT_AN_LINK_CNTL_50GBASE_CR2_MASK) >>
			XCMAC_STAT_AN_LINK_CNTL_50GBASE_CR2_BIT;

	return 0;
}

/*************************************************************************/
/**
 * This function is used to read the Link Training parameters status.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    status pointer to hold Link Training parameters status.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_link_training_status(struct xcmac *instance,
		struct xcmac_link_training_status *status)
{
	uint32_t offset;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(status);

	offset = XCMAC_STAT_LT_STATUS_REG1_OFFSET;
	value = xcmac_in32(instance->base_address + offset);

	status->link_training_init =
			(value & XCMAC_STAT_LT_INITIALIZE_FROM_RX_MASK) >>
			XCMAC_STAT_LT_INITIALIZE_FROM_RX_BIT;
	status->link_training_preset =
			(value & XCMAC_STAT_LT_PRESET_FROM_RX_MASK) >>
			XCMAC_STAT_LT_PRESET_FROM_RX_BIT;

	offset = XCMAC_STAT_LT_STATUS_REG2_OFFSET;
	value = xcmac_in32(instance->base_address + offset);

	status->link_training_training =
			(value & XCMAC_STAT_LT_TRAINING_MASK) >>
			XCMAC_STAT_LT_TRAINING_BIT;
	status->link_training_frame_lock =
			(value & XCMAC_STAT_LT_FRAME_LOCK_MASK) >>
			XCMAC_STAT_LT_FRAME_LOCK_BIT;

	offset = XCMAC_STAT_LT_STATUS_REG3_OFFSET;
	value = xcmac_in32(instance->base_address + offset);

	status->link_training_signal_detect =
			(value & XCMAC_STAT_LT_SIGNAL_DETECT_MASK) >>
			XCMAC_STAT_LT_SIGNAL_DETECT_BIT;
	status->link_training_training_fail =
			(value & XCMAC_STAT_LT_TRAINING_FAIL_MASK) >>
			XCMAC_STAT_LT_TRAINING_FAIL_BIT;

	offset = XCMAC_STAT_LT_STATUS_REG4_OFFSET;
	value = xcmac_in32(instance->base_address + offset);

	status->link_training_rx_sof =
			(value & XCMAC_STAT_LT_RX_SOF_BIT_MASK) >>
			XCMAC_STAT_LT_RX_SOF_BIT_BIT;

	return 0;
}

/*************************************************************************/
/**
 * This function is used to configure the autonegotiation parameters.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    autoneg_config pointer to hold autonegotiation parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_config_autoneg(struct xcmac *instance,
		struct xcmac_autoneg_config *config)
{
	uint32_t offset = 0;
	uint32_t value = 0;

	is_null(instance);
	is_null(config);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	/*Write to Autoneg config reg1*/
	offset = XCMAC_CONFIG_AN_CONTROL_REG1_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	if (config->enable == 1)
		set_bit(value, XCMAC_CTL_AUTONEG_ENABLE_BIT);

	if (config->bypass == 1)
		set_bit(value, XCMAC_CTL_AUTONEG_BYPASS_BIT);

	if (config->nonce_seed) {
		value = value & (~XCMAC_CTL_AN_NONCE_SEED_BIT_MASK);
		value = value | ((config->nonce_seed
			<< XCMAC_CTL_AN_NONCE_SEED_BIT) &
			XCMAC_CTL_AN_NONCE_SEED_BIT_MASK);
	}
	if (config->pseudo_sel == 1)
		set_bit(value, XCMAC_CTL_AN_PSEUDO_SEL_BIT);

	if (config->restart == 1)
		set_bit(value, XCMAC_CTL_AN_PSEUDO_SEL_BIT);

	if (config->local_fault == 1)
		set_bit(value, XCMAC_CTL_AN_LOCAL_FAULT_BIT);

	xcmac_out32(instance->base_address + offset, value);

	/*Write to Autoneg config reg2*/
	offset = XCMAC_CONFIG_AN_CONTROL_REG2_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	if (config->pause == 1)
		set_bit(value, XCMAC_CTL_AN_PAUSE_BIT);

	if (config->asmdir == 1)
		set_bit(value, XCMAC_CTL_AN_ASMDIR_BIT);

	if (config->cl91_fec_request == 1)
		set_bit(value, XCMAC_CTL_AN_CL91_FEC_REQUEST_BIT);

	if (config->cl91_fec_ability == 1)
		set_bit(value, XCMAC_CTL_AN_CL91_FEC_ABILITY_BIT);

	if (config->fec_25g_rs_request == 1)
		set_bit(value, XCMAC_CTL_AN_FEC_25G_RS_REQUEST_BIT);

	if (config->loc_np == 1)
		set_bit(value, XCMAC_CTL_AN_LOC_NP_BIT);

	if (config->lp_np_ack == 1)
		set_bit(value, XCMAC_CTL_AN_LP_NP_ACK_BIT);

	xcmac_out32(instance->base_address + offset, value);

	return 0;
}

/*************************************************************************/
/**
 * This function is used to get autonegotiation parameters.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    autoneg_config pointer to hold autonegotiation parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_config_autoneg(struct xcmac *instance,
		struct xcmac_autoneg_config *config)
{
	uint32_t offset = XCMAC_CONFIG_AN_CONTROL_REG1_OFFSET;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(config);

	value = xcmac_in32(instance->base_address + offset);

	config->enable = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	config->bypass = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	config->nonce_seed = (value & XCMAC_CTL_AN_NONCE_SEED_BIT_MASK) >>
	XCMAC_CTL_AN_NONCE_SEED_BIT;
	config->pseudo_sel = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	config->restart = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	config->local_fault = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);

	offset = XCMAC_CONFIG_AN_CONTROL_REG2_OFFSET;
	value = xcmac_in32(instance->base_address + offset);

	config->pause = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	config->asmdir = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	config->cl91_fec_request = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	config->cl91_fec_ability = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	config->fec_25g_rs_request = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	config->loc_np = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);
	config->lp_np_ack = get_bit_status(
			value, XCMAC_STAT_AN_LP_ABILITY_100GBASE_CR4_BIT);

	return 0;
}

/*************************************************************************/
/**
 * This function is used to configure the autonegotiation ability parameters.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    autoneg_ability pointer to hold autonegotiation ability parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_config_autoneg_ability(struct xcmac *instance,
		struct xcmac_autoneg_ability *autoneg_ability)
{
	uint32_t offset = XCMAC_CONFIG_AN_ABILITY_OFFSET;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(autoneg_ability);

	value = xcmac_in32(instance->base_address + offset);
	if (autoneg_ability->ctl_autoneg_ability_1000base_kx == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_1000BASE_KX_BIT);

	if (autoneg_ability->ctl_autoneg_ability_10gbase_kx4 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_10GBASE_KX4_BIT);

	if (autoneg_ability->ctl_autoneg_ability_10gbase_kr == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_10GBASE_KR_BIT);

	if (autoneg_ability->ctl_autoneg_ability_40gbase_kr4 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_40GBASE_KR4_BIT);

	if (autoneg_ability->ctl_autoneg_ability_40gbase_cr4 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_40GBASE_KR4_BIT);

	if (autoneg_ability->ctl_autoneg_ability_100gbase_cr10 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_100GBASE_CR10_BIT);

	if (autoneg_ability->ctl_autoneg_ability_100gbase_kp4 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_100GBASE_KP4_BIT);

	if (autoneg_ability->ctl_autoneg_ability_100gbase_kr4 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_100GBASE_KR4_BIT);

	if (autoneg_ability->ctl_autoneg_ability_100gbase_cr4 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_100GBASE_CR4_BIT);

	if (autoneg_ability->ctl_autoneg_ability_25gbase_kr == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_25GBASE_KR_BIT);

	if (autoneg_ability->ctl_autoneg_ability_25gbase_cr == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_25GBASE_CR_BIT);

	if (autoneg_ability->ctl_autoneg_ability_25gbase_kr1 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_25GBASE_KR1_BIT);

	if (autoneg_ability->ctl_autoneg_ability_25gbase_cr1 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_25GBASE_CR1_BIT);

	if (autoneg_ability->ctl_autoneg_ability_50gbase_kr2 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_50GBASE_KR2_BIT);

	if (autoneg_ability->ctl_autoneg_ability_50gbase_cr2 == true)
		set_bit(value, XCMAC_CTL_AN_ABILITY_50GBASE_CR2_BIT);

	xcmac_out32(instance->base_address + offset, value);
	return 0;
}

/*************************************************************************/
/**
 * This function is used to get autonegotiation ability parameters.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    autoneg_ability pointer to hold autonegotiation ability parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_config_autoneg_ability(
		struct xcmac *instance,
		struct xcmac_autoneg_ability *autoneg_ability)
{
	uint32_t offset = XCMAC_CONFIG_AN_ABILITY_OFFSET;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(autoneg_ability);

	value = xcmac_in32(instance->base_address + offset);

	autoneg_ability->ctl_autoneg_ability_1000base_kx =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_1000BASE_KX_BIT);
	autoneg_ability->ctl_autoneg_ability_10gbase_kx4 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_10GBASE_KX4_BIT);
	autoneg_ability->ctl_autoneg_ability_10gbase_kr =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_10GBASE_KR_BIT);
	autoneg_ability->ctl_autoneg_ability_40gbase_kr4 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_40GBASE_KR4_BIT);
	autoneg_ability->ctl_autoneg_ability_40gbase_cr4 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_40GBASE_CR4_BIT);
	autoneg_ability->ctl_autoneg_ability_100gbase_cr10 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_100GBASE_CR10_BIT);
	autoneg_ability->ctl_autoneg_ability_100gbase_kp4 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_100GBASE_KP4_BIT);
	autoneg_ability->ctl_autoneg_ability_100gbase_kr4 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_100GBASE_KR4_BIT);
	autoneg_ability->ctl_autoneg_ability_100gbase_cr4 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_100GBASE_CR4_BIT);
	autoneg_ability->ctl_autoneg_ability_25gbase_kr =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_25GBASE_KR_BIT);
	autoneg_ability->ctl_autoneg_ability_25gbase_cr =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_25GBASE_CR_BIT);
	autoneg_ability->ctl_autoneg_ability_25gbase_kr1 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_25GBASE_KR1_BIT);
	autoneg_ability->ctl_autoneg_ability_25gbase_cr1 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_25GBASE_CR1_BIT);
	autoneg_ability->ctl_autoneg_ability_50gbase_kr2 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_50GBASE_KR2_BIT);
	autoneg_ability->ctl_autoneg_ability_50gbase_cr2 =
			get_bit_status(value,
					XCMAC_CTL_AN_ABILITY_50GBASE_CR2_BIT);

	return 0;
}

/*************************************************************************/
/**
 * This function is used to enable or restart the LT training.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    option indicates to enable or restart.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_config_link_training(struct xcmac *instance,
		struct xcmac_lt_config *config)
{
	uint32_t offset = XCMAC_CONFIG_LT_CONTROL_REG1_OFFSET;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(config);

	/* LT enable and LT Restart Training*/
	value = xcmac_in32(instance->base_address + offset);
	if (config->ctl_lt_training_enable == 1)
		set_bit(value, XCMAC_CTL_LT_TRAINING_ENABLE_BIT);
	else
		clear_bit(value, XCMAC_CTL_LT_TRAINING_ENABLE_BIT);

	if (config->ctl_lt_restart_training == 1)
		set_bit(value, XCMAC_CTL_LT_RESTART_TRAINING_BIT);
	else
		clear_bit(value, XCMAC_CTL_LT_RESTART_TRAINING_BIT);

	xcmac_out32(instance->base_address + offset, value);

	/* LT trained */
	offset = XCMAC_CONFIG_LT_TRAINED_REG_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	value = value & ~XCMAC_CTL_LT_RX_TRAINED_BIT_MASK;
	value = value | (config->ctl_lt_rx_trained
			& XCMAC_CTL_LT_RX_TRAINED_BIT_MASK);
	xcmac_out32(instance->base_address + offset, value);

	/* LT Preset to Tx*/
	offset = XCMAC_CONFIG_LT_PRESET_REG_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	value = value & ~XCMAC_CTL_LT_PRESET_TO_TX_BIT_MASK;
	value = value | (config->ctl_lt_preset_to_tx &
	XCMAC_CTL_LT_PRESET_TO_TX_BIT_MASK);
	xcmac_out32(instance->base_address + offset, value);

	/* LT initialize to Tx*/
	offset = XCMAC_CONFIG_LT_INIT_REG_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	value = value & ~XCMAC_CTL_LT_INITIALIZE_TO_TX_BIT_MASK;
	value = value | (config->ctl_lt_initialize_to_tx &
	XCMAC_CTL_LT_INITIALIZE_TO_TX_BIT_MASK);
	xcmac_out32(instance->base_address + offset, value);

	/* Lt pseudo seed 0*/
	offset = XCMAC_CONFIG_LT_SEED_REG0_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	value = value & ~XCMAC_CTL_LT_PSEUDO_SEED0_BIT_MASK;
	value = value | (((uint32_t)config->ctl_lt_pseudo_seed0
			<< XCMAC_CTL_LT_PSEUDO_SEED0_BIT) &
	XCMAC_CTL_LT_PSEUDO_SEED0_BIT_MASK);
	value = value & ~XCMAC_CTL_LT_PSEUDO_SEED1_BIT_MASK;
	value = value | (((uint32_t)config->ctl_lt_pseudo_seed1
			<< XCMAC_CTL_LT_PSEUDO_SEED1_BIT) &
	XCMAC_CTL_LT_PSEUDO_SEED1_BIT_MASK);
	xcmac_out32(instance->base_address + offset, value);

	/* Lt pseudo seed 1*/
	offset = XCMAC_CONFIG_LT_SEED_REG1_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	value = value & ~XCMAC_CTL_LT_PSEUDO_SEED2_BIT_MASK;
	value = value | (((uint32_t)config->ctl_lt_pseudo_seed2
			<< XCMAC_CTL_LT_PSEUDO_SEED2_BIT) &
	XCMAC_CTL_LT_PSEUDO_SEED2_BIT_MASK);
	value = value & ~XCMAC_CTL_LT_PSEUDO_SEED3_BIT_MASK;
	value = value | (((uint32_t)config->ctl_lt_pseudo_seed3
			<< XCMAC_CTL_LT_PSEUDO_SEED3_BIT) &
	XCMAC_CTL_LT_PSEUDO_SEED3_BIT_MASK);
	xcmac_out32(instance->base_address + offset, value);

	return 0;
}

/*************************************************************************/
/**
 * This function is used to get the LT training parameters.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    option indicates to enable or restart.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_config_link_training(struct xcmac *instance,
		struct xcmac_lt_config *config)
{
	uint32_t offset = XCMAC_CONFIG_LT_CONTROL_REG1_OFFSET;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(config);

	/* LT enable and LT Restart Training*/
	value = xcmac_in32(instance->base_address + offset);
	config->ctl_lt_training_enable = get_bit_status(value,
			XCMAC_CTL_LT_TRAINING_ENABLE_BIT);
	config->ctl_lt_restart_training = get_bit_status(value,
			XCMAC_CTL_LT_RESTART_TRAINING_BIT);

	/* LT trained */
	offset = XCMAC_CONFIG_LT_TRAINED_REG_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	config->ctl_lt_rx_trained = value & XCMAC_CTL_LT_RX_TRAINED_BIT_MASK;

	/* LT Preset to Tx*/
	offset = XCMAC_CONFIG_LT_PRESET_REG_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	config->ctl_lt_preset_to_tx = value &
			XCMAC_CTL_LT_PRESET_TO_TX_BIT_MASK;

	/* LT initialize to Tx*/
	offset = XCMAC_CONFIG_LT_INIT_REG_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	config->ctl_lt_initialize_to_tx = value &
			XCMAC_CTL_LT_INITIALIZE_TO_TX_BIT_MASK;

	/* Lt pseudo seed 0*/
	offset = XCMAC_CONFIG_LT_SEED_REG0_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	config->ctl_lt_pseudo_seed0 = value &
			XCMAC_CTL_LT_PSEUDO_SEED0_BIT_MASK;
	config->ctl_lt_pseudo_seed1 = (value &
			XCMAC_CTL_LT_PSEUDO_SEED1_BIT_MASK) >>
			XCMAC_CTL_LT_PSEUDO_SEED1_BIT;

	/* Lt pseudo seed 1*/
	offset = XCMAC_CONFIG_LT_SEED_REG1_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	config->ctl_lt_pseudo_seed2 = value &
			XCMAC_CTL_LT_PSEUDO_SEED2_BIT_MASK;
	config->ctl_lt_pseudo_seed3 =
			(value & XCMAC_CTL_LT_PSEUDO_SEED3_BIT_MASK) >>
			XCMAC_CTL_LT_PSEUDO_SEED3_BIT;

	return 0;
}

/*************************************************************************/
/**
 * This function is used to read LT coefficients parameter values.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    coeffs is a pointer to hold Link Training coefficients parameters.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_get_link_training_coeff(
		struct xcmac *instance,
		struct xcmac_link_training_coefficients *coeffs)
{
	uint32_t offset = XCMAC_CONFIG_LT_COEFFICIENT_REG0_OFFSET;
	uint32_t value = 0;

	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	is_null(coeffs);

	value = xcmac_in32(instance->base_address + offset);
	coeffs->k_p1_to_tx0 = (value & XCMAC_STAT_LT_K_P1_FROM_RX0_BIT_MASK) >>
	XCMAC_STAT_LT_K_P1_FROM_RX0_BIT;
	coeffs->k0_to_tx0 = (value & XCMAC_STAT_LT_K0_FROM_RX0_BIT_MASK) >>
	XCMAC_STAT_LT_K0_FROM_RX0_BIT;
	coeffs->k_m1_to_tx0 = (value & XCMAC_STAT_LT_K_M1_FROM_RX0_BIT_MASK) >>
	XCMAC_STAT_LT_K_M1_FROM_RX0_BIT;
	coeffs->stat_p1_to_tx0 =
			(value & XCMAC_STAT_LT_STAT_P1_FROM_RX0_BIT_MASK) >>
			XCMAC_STAT_LT_STAT_P1_FROM_RX0_BIT;
	coeffs->stat0_to_tx0 =
			(value & XCMAC_STAT_LT_STAT0_FROM_RX0_BIT_MASK) >>
			XCMAC_STAT_LT_STAT0_FROM_RX0_BIT;
	coeffs->stat_m1_to_tx0 =
			(value & XCMAC_STAT_LT_STAT_M1_FROM_RX0_BIT_MASK) >>
			XCMAC_STAT_LT_STAT_M1_FROM_RX0_BIT;
	coeffs->k_p1_to_tx1 = (value & XCMAC_STAT_LT_K_P1_TO_TX1_BIT_MASK) >>
	XCMAC_STAT_LT_K_P1_TO_TX1_BIT;
	coeffs->k0_to_tx1 = (value & XCMAC_STAT_LT_K0_TO_TX1_BIT_MASK) >>
	XCMAC_STAT_LT_K0_TO_TX1_BIT;
	coeffs->k_m1_to_tx1 = (value & XCMAC_STAT_LT_K_M1_TO_TX1_BIT_MASK) >>
	XCMAC_STAT_LT_K_M1_TO_TX1_BIT;
	coeffs->stat_p1_to_tx1 =
			(value & XCMAC_STAT_LT_STAT_P1_TO_TX1_BIT_MASK) >>
			XCMAC_STAT_LT_STAT_P1_TO_TX1_BIT;
	coeffs->stat0_to_tx1 = (value & XCMAC_STAT_LT_STAT0_TO_TX1_BIT_MASK) >>
	XCMAC_STAT_LT_STAT0_TO_TX1_BIT;
	coeffs->stat_m1_to_tx1 =
			(value & XCMAC_STAT_LT_STAT_M1_TO_TX1_BIT_MASK) >>
			XCMAC_STAT_LT_STAT_M1_TO_TX1_BIT;

	offset = XCMAC_CONFIG_LT_COEFFICIENT_REG1_OFFSET;
	value = xcmac_in32(instance->base_address + offset);
	coeffs->k_p1_to_tx2 = (value & XCMAC_STAT_LT_K_P1_TO_TX2_BIT_MASK) >>
	XCMAC_STAT_LT_K_P1_TO_TX2_BIT;
	coeffs->k0_to_tx2 = (value & XCMAC_STAT_LT_K0_TO_TX2_BIT_MASK) >>
	XCMAC_STAT_LT_K0_TO_TX2_BIT;
	coeffs->k_m1_to_tx2 = (value & XCMAC_STAT_LT_K_M1_TO_TX2_BIT_MASK) >>
	XCMAC_STAT_LT_K_M1_TO_TX2_BIT;
	coeffs->stat_p1_to_tx2 =
			(value & XCMAC_STAT_LT_STAT_P1_TO_TX2_BIT_MASK) >>
			XCMAC_STAT_LT_STAT_P1_TO_TX2_BIT;
	coeffs->stat0_to_tx2 = (value & XCMAC_STAT_LT_STAT0_TO_TX2_BIT_MASK) >>
	XCMAC_STAT_LT_STAT0_TO_TX2_BIT;
	coeffs->stat_m1_to_tx2 =
			(value & XCMAC_STAT_LT_STAT_M1_TO_TX2_BIT_MASK) >>
			XCMAC_STAT_LT_STAT_M1_TO_TX2_BIT;
	coeffs->k_p1_to_tx3 = (value & XCMAC_STAT_LT_K_P1_TO_TX3_BIT_MASK) >>
	XCMAC_STAT_LT_K_P1_TO_TX3_BIT;
	coeffs->k0_to_tx3 = (value & XCMAC_STAT_LT_K0_TO_TX3_BIT_MASK) >>
	XCMAC_STAT_LT_K0_TO_TX3_BIT;
	coeffs->k_m1_to_tx3 = (value & XCMAC_STAT_LT_K_M1_TO_TX3_BIT_MASK) >>
	XCMAC_STAT_LT_K_M1_TO_TX3_BIT;
	coeffs->stat_p1_to_tx3 =
			(value & XCMAC_STAT_LT_STAT_P1_TO_TX3_BIT_MASK) >>
			XCMAC_STAT_LT_STAT_P1_TO_TX3_BIT;
	coeffs->stat0_to_tx3 = (value & XCMAC_STAT_LT_STAT0_TO_TX3_BIT_MASK) >>
	XCMAC_STAT_LT_STAT0_TO_TX3_BIT;
	coeffs->stat_m1_to_tx3 =
			(value & XCMAC_STAT_LT_STAT_M1_TO_TX3_BIT_MASK) >>
			XCMAC_STAT_LT_STAT_M1_TO_TX3_BIT;

	return 0;
}

/*************************************************************************/
/**
 * This function is used to enable/disable rs-fec from cmac system.
 *
 * @param    instance is a pointer to the 100G  instance to be worked on.
 * @param    enable To enable or disable rs-fec.
 *
 * @return
 * - 0 if success
 * - Error number if there is an error
 *
 * @note none.
 **************************************************************************/
int xcmac_config_rsfec(struct xcmac *instance, bool enable)
{
	is_null(instance);
	if (instance->is_ready != XIL_COMPONENT_IS_READY)
		return -EINVAL;

	if (enable)
		xcmac_out32(instance->base_address +
		XCMAC_RSFEC_CONFIG_ENABLE_OFFSET,
				(XCMAC_RSFEC_CONFIG_RX_ENABLE_MASK |
				XCMAC_RSFEC_CONFIG_TX_ENABLE_MASK));
	else
		xcmac_out32(instance->base_address +
		XCMAC_RSFEC_CONFIG_ENABLE_OFFSET,
				0);

	return 0;
}
