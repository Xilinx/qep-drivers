/*
 * This file is part of the Xilinx DMA IP Core driver for Linux
 *
 * Copyright (c) 2017-2019,  Xilinx, Inc.
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

#ifndef __LIBQDMA_EXPORT_API_H__
#define __LIBQDMA_EXPORT_API_H__


/**
 * DOC: Xilinx QDMA Library Interface Definitions
 *
 * Header file *libqdma_export.h* defines data structures and function
 * signatures exported by Xilinx QDMA(libqdma) Library.
 * libqdma is part of Xilinx QDMA Linux Driver.
 */

#include <linux/types.h>
#include <linux/interrupt.h>
#include "libqdma_config.h"
#include "qdma_access_export.h"

/** Invalid QDMA function number */
#define QDMA_FUNC_ID_INVALID	(QDMA_PF_MAX + QDMA_VF_MAX)

/**
 * enum qdma_q_mode - mode in which Q is initialized
 */
enum qdma_q_mode {
	/** @QDMA_Q_MODE_MM: MM mode */
	QDMA_Q_MODE_MM,
	/** @QDMA_Q_MODE_ST: ST mode */
	QDMA_Q_MODE_ST
};
/**
 * enum qdma_q_dir - direction in which Q is initialized
 */
enum qdma_q_dir {
	/** @QDMA_Q_DIR_H2C: host to card */
	QDMA_Q_DIR_H2C,
	/** @QDMA_Q_DIR_H2C: card to host */
	QDMA_Q_DIR_C2H
};

/**
 * enum qdma_error_codes - List of QDMA error codes
 *
 * When libqdma APIs are invoked by application, if any error occured,
 * the corresponding error code would be returned.
 *
 */
enum qdma_error_codes {
	/** @QDMA_OPERATION_SUCCESSFUL: QDMA driver API operation successful */
	QDMA_OPERATION_SUCCESSFUL                   = 0,
	/** @QDMA_ERR_PCI_DEVICE_NOT_FOUND: not found on the PCIe bus */
	QDMA_ERR_PCI_DEVICE_NOT_FOUND               = -1,
	/** @QDMA_ERR_PCI_DEVICE_ALREADY_ATTACHED: device already attached */
	QDMA_ERR_PCI_DEVICE_ALREADY_ATTACHED        = -2,
	/**
	 * @QDMA_ERR_PCI_DEVICE_ENABLE_FAILED: Failed to enable the PCIe device
	 */
	QDMA_ERR_PCI_DEVICE_ENABLE_FAILED           = -3,
	/** @QDMA_ERR_PCI_DEVICE_INIT_FAILED: Failed to initialize the device */
	QDMA_ERR_PCI_DEVICE_INIT_FAILED             = -4,
	/** @QDMA_ERR_INVALID_INPUT_PARAM: Invalid input parameter received */
	QDMA_ERR_INVALID_INPUT_PARAM                = -5,
	/** @QDMA_ERR_INVALID_PCI_DEV: Invalid PCIe device */
	QDMA_ERR_INVALID_PCI_DEV                    = -6,
	/** @QDMA_ERR_INVALID_QIDX: Invalid Queue ID provided as input */
	QDMA_ERR_INVALID_QIDX                       = -7,
	/**
	 * @QDMA_ERR_INVALID_DESCQ_STATE: Invalid descriptor queue state
	 */
	QDMA_ERR_INVALID_DESCQ_STATE                = -8,
	/**
	 * @QDMA_ERR_INVALID_DIRECTION: Invalid descriptor direction provided
	 */
	QDMA_ERR_INVALID_DIRECTION                  = -9,
	/**
	 * @QDMA_ERR_DESCQ_SETUP_FAILED: Failed to setup the descriptor queue
	 */
	QDMA_ERR_DESCQ_SETUP_FAILED                 = -10,
	/** @QDMA_ERR_DESCQ_FULL: Descriptor queue is full */
	QDMA_ERR_DESCQ_FULL                         = -11,
	/**
	 * @QDMA_ERR_DESCQ_IDX_ALREADY_ADDED: Descriptor queue
	 * index is already added
	 */
	QDMA_ERR_DESCQ_IDX_ALREADY_ADDED            = -12,
	/** @QDMA_ERR_QUEUE_ALREADY_CONFIGURED: Queue is already configured */
	QDMA_ERR_QUEUE_ALREADY_CONFIGURED           = -13,
	/** @QDMA_ERR_OUT_OF_MEMORY: Out of memory */
	QDMA_ERR_OUT_OF_MEMORY                      = -14,
	/**
	 *  @QDMA_ERR_INVALID_QDMA_DEVICE: Invalid QDMA device, QDMA device
	 *  is not yet created
	 */
	QDMA_ERR_INVALID_QDMA_DEVICE                = -15,
	/**
	 *  @QDMA_ERR_INTERFACE_NOT_ENABLED_IN_DEVICE : The ST or MM or Both
	 *  interface not enabled in the device
	 */
	QDMA_ERR_INTERFACE_NOT_ENABLED_IN_DEVICE    = -16,
	/**
	 *  @QDMA_ERR_MISSING_DEVICE_CAPABILITY : Device capability is
	 *  missing
	 */
	QDMA_ERR_MISSING_DEVICE_CAPABILITY          = -17,
};

/**
 * enum qdma_drv_mode - Indicates whether PF/VF qdma driver is loaded in
 *                      poll or interrupt mode
 *
 * QDMA PF/VF drivers can be loaded in one of these modes.
 * Mode options is exposed as a user configurable module parameter
 *
 */
enum qdma_drv_mode {
	/**
	 *  @AUTO_MODE : auto mode decided automatically,
	 *  mix of poll and interrupt mode
	 */
	AUTO_MODE,
	/** @POLL_MODE : driver is inserted in poll mode */
	POLL_MODE,
	/** @DIRECT_INTR_MODE : driver is inserted in direct interrupt mode */
	DIRECT_INTR_MODE,
	/**
	 *  @INDIRECT_INTR_MODE : driver is inserted in
	 *  indirect interrupt mode
	 */
	INDIRECT_INTR_MODE,
	/** @LEGACY_INTR_MODE : driver is inserted in legacy interrupt mode */
	LEGACY_INTR_MODE
};

/**
 * struct drv_mode_name - Structure to hold the driver name and mode
 *
 * QDMA PF/VF drivers can be loaded in one of these modes.
 * This structure holds the driver mode and name of the driver
 *
 */
struct drv_mode_name {
	/** @drv_mode : mode of the driver in which it it loaded. */
	enum qdma_drv_mode drv_mode;
	/** @name     : Driver Name */
	char name[20];
};

struct qdma_ul_cmpt_info {
	/* cmpl entry stat bits */
	union {
		u8 fbits;
		struct cmpl_flag {
			u8 format:1;
			u8 color:1;
			u8 err:1;
			u8 desc_used:1;
			u8 eot:1;
			u8 filler:3;
		} f;
	};
	u8 rsvd;
	u16 len;
	/* for tracking */
	unsigned int pidx;
	__be64 *entry;
};

extern struct drv_mode_name mode_name_list[];

struct pci_dev;

/*****************************************************************************/
/**
 * libqdma_init() - initializes the QDMA core library
 *
 * @num_threads: number of threads to be created each for request
 *  processing and writeback processing
 *
 * Return: 0:	success <0:	error
 *
 *****************************************************************************/
int libqdma_init(unsigned int num_threads, void *debugfs_root);

/*****************************************************************************/
/**
 * libqdma_exit() - cleanup the QDMA core library before exiting
 *
 * cleanup the QDMA core library before exiting
 *
 *****************************************************************************/
void libqdma_exit(void);

/**
 * enum intr_ring_size_sel - qdma interrupt ring size selection
 *
 * Each interrupt vector can be associated with 1 or more interrupt rings.
 * The software can choose 8 different interrupt ring sizes. The ring size
 * for each vector is programmed during interrupt context programming
 *
 */
enum intr_ring_size_sel {
	/** @INTR_RING_SZ_4KB: accommodates 512 entries */
	INTR_RING_SZ_4KB = 0,
	/** @INTR_RING_SZ_8KB: accommodates 1024 entries */
	INTR_RING_SZ_8KB,
	/** @INTR_RING_SZ_12KB: accommodates 1536 entries */
	INTR_RING_SZ_12KB,
	/** @INTR_RING_SZ_16KB: accommodates 2048 entries */
	INTR_RING_SZ_16KB,
	/** @INTR_RING_SZ_20KB: accommodates 2560 entries */
	INTR_RING_SZ_20KB,
	/** @INTR_RING_SZ_24KB: accommodates 3072 entries */
	INTR_RING_SZ_24KB,
	/** @INTR_RING_SZ_28KB: accommodates 3584 entries */
	INTR_RING_SZ_28KB,
	/** @INTR_RING_SZ_32KB: accommodates 4096 entries */
	INTR_RING_SZ_32KB,
};

/**
 * enum qdma_dev_qmax_state: qdma queue states
 *
 * Each PF/VF device can be configured with 0 or more number of queues.
 * When the queue is not assigned to any device, device is in unfonfigured
 * state. When PF/VF is instatiated, it has the default number of queues
 * assigned and the device is in initial state. Sysfs interface enables the
 * users to alter the default number of queues assigned to a device and
 * it moves to user configured state.
 *
 */
enum qdma_dev_qmax_state {
	/** @QMAX_CFG_UNCONFIGURED : device qmax not configured */
	QMAX_CFG_UNCONFIGURED,
	/**
	 *  @QMAX_CFG_INITIAL : device qmax configured with
	 *  initial default values
	 */
	QMAX_CFG_INITIAL,
	/**
	 *  @QMAX_CFG_USER: device qmax configured from
	 *  sysfs as per user choice
	 */
	QMAX_CFG_USER,
};

/**
 *	Maxinum length of the QDMA device name
 */
#define QDMA_DEV_NAME_MAXLEN	32

/**
 * struct qdma_dev_conf defines the per-device qdma property.
 *
 * NOTE: if any of the max requested is less than supported, the value will
 *       be updated
 */
struct qdma_dev_conf {
	/**	@pdev: pointer to pci_dev */
	struct pci_dev *pdev;
	/**	@qsets_max: Maximum number of queue pairs per device */
	unsigned short qsets_max;
	/**	@rsvd2: Reserved */
	unsigned short rsvd2;
	/**
	 *  @zerolen_dma: Indicates whether zero length
	 *  DMA is allowed or not
	 */
	u8 zerolen_dma:1;
	/**
	 *  @master_pf: Indicates whether the current pf
	 *  is master_pf or not
	 */
	u8 master_pf:1;
	/**
	 * @intr_moderation: moderate interrupt generation
	 */
	u8 intr_moderation:1;
	/**	@rsvd1: Reserved1 */
	u8 rsvd1:5;
	/**
	 *  @vf_max: Maximum number of virtual functions for
	 *  current physical function
	 */
	u8 vf_max;
	/**	@intr_rngsz: Interrupt ring size */
	u8 intr_rngsz;
	/**
	 * @msix_qvec_max:
	 * interrupt:
	 * - MSI-X only
	 * max of QDMA_DEV_MSIX_VEC_MAX per function, 32 in Everest
	 * - 1 vector is reserved for user interrupt
	 * - 1 vector is reserved mailbox
	 * - 1 vector on pf0 is reserved for error interrupt
	 * - the remaining vectors will be used for queues
	 */

	/**
	 *  @msix_qvec_max: max. of vectors used for queues.
	 *  libqdma update w/ actual #
	 */
	u8 msix_qvec_max;
	/** @uld: upper layer data, i.e. callback data */
	unsigned long uld;
	/** @qdma_drv_mode: qdma driver mode */
	enum qdma_drv_mode qdma_drv_mode;
	/**
	 * @name: an unique string to identify the dev.
	 * current format: qdma[pf|vf][idx] filled in by libqdma
	 */
	char name[QDMA_DEV_NAME_MAXLEN];

#ifdef RTL2_FEATURE_CONFIGURABLE_BAR
	/** @bar_num_config: dma config bar #, < 0 not present */
	char bar_num_config;
	/** @bar_num_user: user bar */
	char bar_num_user;
	/**	@bar_num_bypass: bypass bar */
	char bar_num_bypass;
#endif
	/** @bar_num_config: dma config bar #, < 0 not present */
	char bar_num_config;
	/** @bar_num_user: user bar */
	char bar_num_user;
	/**	@bar_num_bypass: bypass bar */
	char bar_num_bypass;
	/** @qsets_base: queue base for this funciton */
	int qsets_base;
	/** @bdf: device index */
	u32 bdf;
	/** @idx: index of device in device list */
	u32 idx;
	/**
	 *  @fp_user_isr_handler: user interrupt, if null,
	 *  default libqdma handler is used
	 */
	void (*fp_user_isr_handler)(unsigned long dev_hndl, unsigned long uld);

	/**
	 *  @fp_q_isr_top_dev:  Q interrupt top,
	 *  per-device addtional handling code
	 */
	void (*fp_q_isr_top_dev)(unsigned long dev_hndl, unsigned long uld);

	void *debugfs_dev_root;
};

/*****************************************************************************/
/**
 * intr_legacy_init() - legacy interrupt init
 *
 *****************************************************************************/
void intr_legacy_init(void);

/*****************************************************************************/
/**
 * qdma_device_open()- read the pci bars and configure the fpga
 * This API should be called from probe()
 *
 * User interrupt will not be enabled until qdma_user_isr_enable() is called
 *
 * @mod_name:	the module name, used for request_irq
 * @conf:		device configuration
 * @dev_hndl:	an opaque handle for libqdma to identify the device
 *
 * Return:QDMA_OPERATION_SUCCESSFUL in case of success and <0 in case of error
 *
 *****************************************************************************/
int qdma_device_open(const char *mod_name, struct qdma_dev_conf *conf,
				unsigned long *dev_hndl);

/*****************************************************************************/
/**
 * qdma_device_close()- prepare fpga for removal: disable all interrupts (users
 * and qdma) and release all resources.This API should be called from remove()
 *
 * @pdev:		ptr to struct pci_dev
 * @dev_hndl:	dev_hndl retured from qdma_device_open()
 *
 *****************************************************************************/
void qdma_device_close(struct pci_dev *pdev, unsigned long dev_hndl);

/*****************************************************************************/
/**
 * qdma_device_offline()- Set the device in offline mode
 *
 * @pdev:		ptr to struct pci_dev
 * @dev_hndl:	dev_hndl retured from qdma_device_open()
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
void qdma_device_offline(struct pci_dev *pdev, unsigned long dev_hndl);

/*****************************************************************************/
/**
 * qdma_device_online()- Set the device in online mode and re-initialze it
 *
 * @pdev:		ptr to struct pci_dev
 * @dev_hndl:	dev_hndl retured from qdma_device_open()
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_online(struct pci_dev *pdev, unsigned long dev_hndl);

/*****************************************************************************/
/**
 * qdma_device_flr_quirk_set()- start pre-flr processing
 *
 * @pdev:		ptr to struct pci_dev
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_flr_quirk_set(struct pci_dev *pdev, unsigned long dev_hndl);

/*****************************************************************************/
/**
 * qdma_device_flr_quirk_check()- check if pre-flr processing completed
 *
 * @pdev:		ptr to struct pci_dev
 * @dev_hndl:	dev_hndl retunred from qdma_device_open()
 *
 * Return:	0 for success <0 for error
 *
 *****************************************************************************/
int qdma_device_flr_quirk_check(struct pci_dev *pdev, unsigned long dev_hndl);

/*****************************************************************************/
/**
 * qdma_device_get_config()- retrieve the current device configuration
 *
 * @dev_hndl:   dev_hndl retunred from qdma_device_open()
 * @conf:		device configuration
 * @ebuflen:	input buffer length
 * @ebuf:       error message buffer, can be NULL/0 (i.e., optional)
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_get_config(unsigned long dev_hndl, struct qdma_dev_conf *conf,
				char *ebuf, int ebuflen);

/*****************************************************************************/
/**
 * qdma_device_clear_stats()- clear device statistics
 *
 * @dev_hndl: dev_hndl retunred from qdma_device_open()
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_clear_stats(unsigned long dev_hndl);

/*****************************************************************************/
/**
 * qdma_device_get_mmh2c_pkts()- get mm h2c packets processed
 *
 * @dev_hndl:   dev_hndl retunred from qdma_device_open()
 * @mmh2c_pkts: number of mm h2c packets processed
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_get_mmh2c_pkts(unsigned long dev_hndl,
			       unsigned long long *mmh2c_pkts);

/*****************************************************************************/
/**
 * qdma_device_get_mmc2h_pkts() - get mm c2h packets processed
 *
 * @dev_hndl:   dev_hndl retunred from qdma_device_open()
 * @mmc2h_pkts: number of mm c2h packets processed
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_get_mmc2h_pkts(unsigned long dev_hndl,
				unsigned long long *mmc2h_pkts);

/*****************************************************************************/
/**
 * qdma_device_get_sth2c_pkts()- get st h2c packets processed
 *
 * @dev_hndl:   dev_hndl retunred from qdma_device_open()
 * @sth2c_pkts: number of st h2c packets processed
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_get_sth2c_pkts(unsigned long dev_hndl,
				unsigned long long *sth2c_pkts);

/*****************************************************************************/
/**
 * qdma_device_get_stc2h_pkts()- get st c2h packets processed
 *
 * @dev_hndl:   dev_hndl retunred from qdma_device_open()
 * @stc2h_pkts: number of st c2h packets processed
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_get_stc2h_pkts(unsigned long dev_hndl,
				unsigned long long *stc2h_pkts);

/*****************************************************************************/
/**
 * qdma_device_set_config() - set the current device configuration
 *
 * @dev_hndl:   dev_hndl returned from qdma_device_open()
 * @conf:		device configuration to set
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_set_config(unsigned long dev_hndl, struct qdma_dev_conf *conf);

/*****************************************************************************/
/**
 * qdma_device_sriov_config() -  configure sriov
 *
 * @pdev:		ptr to struct pci_dev
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @num_vfs:	# of VFs to be instantiated
 *
 * Return:	0 for success and <0 for error
 *
 * configures sriov
 *****************************************************************************/
int qdma_device_sriov_config(struct pci_dev *pdev, unsigned long dev_hndl,
				int num_vfs);

/*****************************************************************************/
/**
 * qdma_device_read_config_register() -  read dma config register
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @reg_addr:	register address
 *
 * Return:	value of the config register
 *
 * reads dma config register
 *
 *****************************************************************************/
unsigned int qdma_device_read_config_register(unsigned long dev_hndl,
					unsigned int reg_addr);

/*****************************************************************************/
/**
 * qdma_device_write_config_register() - write dma config register
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @reg_addr:	register address
 * @value:		register value to be writen
 *
 * writes dma config register
 *
 *****************************************************************************/
void qdma_device_write_config_register(unsigned long dev_hndl,
					unsigned int reg_addr, u32 value);

/*****************************************************************************/
/**
 * qdma_device_capabilities_info() - retrieve the capabilities of a device.
 *
 * @dev_hndl:	handle returned from qdma_device_open()
 * @dev_attr: pointer to hold all the device attributes
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_device_capabilities_info(unsigned long dev_hndl,
		struct qdma_dev_attributes *dev_attr);

#define DEVICE_VERSION_INFO_STR_LENGTH            20

/**
 * struct qdma_version_info - defines the per-device version information
 *
 */
struct qdma_version_info {
	/** @rtl_version_str:    Version string */
	char rtl_version_str[DEVICE_VERSION_INFO_STR_LENGTH];
	/** @vivado_release_str: Release string */
	char vivado_release_str[DEVICE_VERSION_INFO_STR_LENGTH];
	/** @everest_ip_str:     Everest IP version string */
	char everest_ip_str[DEVICE_VERSION_INFO_STR_LENGTH];
};

/*****************************************************************************/
/**
 * qdma_device_version_info() - retrieve the RTL version , Vivado Release ID
 *				and Everest IP info
 *
 * @dev_hndl:	handle returned from qdma_device_open()
 * @version_info: pointer to hold all the version details
 *
 * Return:	0 for success and <0 for error
 *
 * retrieves the RTL version , Vivado Release ID and Everest IP info
 *
 *****************************************************************************/
int qdma_device_version_info(unsigned long dev_hndl,
			     struct qdma_version_info *version_info);

/*****************************************************************************/
/**
 * qdma_software_version_info()- retrieve the software version
 *
 * @software_version: A pointer to a null-terminated string
 *
 * Return:	0 for success and <0 for error
 *
 * retrieves the software version
 *
 *****************************************************************************/
int qdma_software_version_info(char *software_version);

/**
 * struct global_csr_conf: global CSR configuration
 *
 */
struct global_csr_conf {
	/** @ring_sz: Descriptor ring size ie. queue depth */
	unsigned int ring_sz[QDMA_GLOBAL_CSR_ARRAY_SZ];
	/** @c2h_timer_cnt: C2H timer count  list */
	unsigned int c2h_timer_cnt[QDMA_GLOBAL_CSR_ARRAY_SZ];
	/** @c2h_cnt_th: C2H counter threshold list*/
	unsigned int c2h_cnt_th[QDMA_GLOBAL_CSR_ARRAY_SZ];
	/** @c2h_buf_sz: C2H buffer size list */
	unsigned int c2h_buf_sz[QDMA_GLOBAL_CSR_ARRAY_SZ];
	/** @wb_intvl: Writeback interval */
	unsigned int wb_intvl;
};

/*****************************************************************************/
/**
 * qdma_global_csr_get() - retrieve the global csr settings
 *
 * @dev_hndl: handle returned from qdma_device_open()
 * @index: Index from where the values needs to read
 * @count: number of entries to be read
 * @csr: data structures to hold the csr values
 *
 * Return:	0 for success and <0 for error
 *
 * retrieves the global csr settings
 *
 *****************************************************************************/
int qdma_global_csr_get(unsigned long dev_hndl, u8 index, u8 count,
		struct global_csr_conf *csr);

/**
 * enum cmpt_desc_sz_t: descriptor sizes
 *
 */
enum cmpt_desc_sz_t {
	/** @CMPT_DESC_SZ_8B: completion size 8B */
	CMPT_DESC_SZ_8B = 0,
	/** @CMPT_DESC_SZ_16B: completion size 16B */
	CMPT_DESC_SZ_16B,
	/** @CMPT_DESC_SZ_32B: completion size 32B */
	CMPT_DESC_SZ_32B,
	/** @CMPT_DESC_SZ_64B: completion size 64B */
	CMPT_DESC_SZ_64B
};

/**
 * enum desc_sz_t: descriptor sizes
 *
 */
enum desc_sz_t {
	/** @DESC_SZ_8B:  descriptor size 8B */
	DESC_SZ_8B = 0,
	/** @DESC_SZ_16B: descriptor size 16B */
	DESC_SZ_16B,
	/** @DESC_SZ_32B: descriptor size 32B */
	DESC_SZ_32B,
	/** @DESC_SZ_64B: descriptor size 64B */
	DESC_SZ_64B
};

/**
 * enum tigger_mode_t: trigger modes
 *
 */
enum tigger_mode_t {
	/** @TRIG_MODE_DISABLE: disable trigger mode */
	TRIG_MODE_DISABLE,
	/** @TRIG_MODE_ANY:     any trigger mode */
	TRIG_MODE_ANY,
	/** @TRIG_MODE_COUNTER: counter trigger mode */
	TRIG_MODE_COUNTER,
	/** @TRIG_MODE_USER:    trigger mode of user choice */
	TRIG_MODE_USER,
	/** @TRIG_MODE_TIMER:   timer trigger mode */
	TRIG_MODE_TIMER,
	/** @TRIG_MODE_COMBO:   timer and counter combo trigger mode */
	TRIG_MODE_COMBO,
};

/**
 * struct qdma_sw_sg - qdma scatter gather request
 *
 */
struct qdma_sw_sg {
	/** @next: pointer to next page */
	struct qdma_sw_sg *next;
	/** @pg: pointer to current page */
	struct page *pg;
	/** @offset: offset in current page */
	unsigned int offset;
	/** @len: length of the page */
	unsigned int len;
	/** @dma_addr: dma address of the allocated page */
	dma_addr_t dma_addr;
};

/** maximum queue name length  */
#define QDMA_QUEUE_NAME_MAXLEN	32

/** invalid queue index */
#define QDMA_QUEUE_IDX_INVALID	0xFFFF

/** invalid MSI-x vector index */
#define QDMA_QUEUE_VEC_INVALID	0xFF

/** struct qdma_request forward declaration*/
struct qdma_request;

/**
 * struct qdma_queue_conf - qdma configuration parameters
 *
 * qdma_queue_conf defines the per-dma Q property.
 * if any of the max requested is less than supported, the value will
 * be updated
 *
 */
struct qdma_queue_conf {
	/**
	 *  @qidx: 0xFFFF: libqdma choose the queue idx 0 ~
	 *  (qdma_dev_conf.qsets_max - 1) the calling function choose the
	 *   queue idx
	 */
	u32 qidx:24;
	/** config flags: byte #1 */
	/** @st: Indicates the streaming mode */
	u32 st:1;
	/** @c2h: c2h direction */
	u32 c2h:1;
	/** @pipe: SDx only: inter-kernel communication pipe */
	u32 pipe:1;
	/** @irq_en: poll or interrupt */
	u32 irq_en:1;

	/** descriptor ring	 */
	/** @desc_rng_sz_idx: global_csr_conf.ringsz[N] */
	u32 desc_rng_sz_idx:4;

	/** config flags: byte #2 */
	/** @wb_status_en: writeback enable, disabled for ST C2H */
	u8 wb_status_en:1;
	/** @cmpl_status_acc_en: sw context.cmpl_status_acc_en */
	u8 cmpl_status_acc_en:1;
	/** @cmpl_status_pend_chk: sw context.cmpl_stauts_pend_chk */
	u8 cmpl_status_pend_chk:1;
	/** @desc_bypass: send descriptor to bypass out */
	u8 desc_bypass:1;
	/** @pfetch_en: descriptor prefetch enable control */
	u8 pfetch_en:1;
	/** @fetch_credit: sw context.frcd_en[32] */
	u8 fetch_credit:1;
	/**
	 *  @st_pkt_mode: SDx only: ST packet mode
	 *  (i.e., with TLAST to identify the packet boundary)
	 */
	u8 st_pkt_mode:1;

	/** config flags: byte #3 */
	/** @c2h_buf_sz_idx: global_csr_conf.c2h_buf_sz[N] */
	u8 c2h_buf_sz_idx:4;

	/**  ST C2H Completion/Writeback ring */
	/** @cmpl_rng_sz_idx: global_csr_conf.ringsz[N] */
	u8 cmpl_rng_sz_idx:4;

	/** config flags: byte #4 */
	/** @cmpl_desc_sz: C2H ST cmpt + immediate data, desc_sz_t */
	u8 cmpl_desc_sz:2;
	/** @cmpl_stat_en: enable status desc. for CMPT */
	u8 cmpl_stat_en:1;
	/** @cmpl_udd_en: C2H Completion entry user-defined data */
	u8 cmpl_udd_en:1;
	/** @cmpl_timer_idx: global_csr_conf.c2h_timer_cnt[N] */
	u8 cmpl_timer_idx:4;

	/** config flags: byte #5 */
	/** @cmpl_cnt_th_idx: global_csr_conf.c2h_cnt_th[N] */
	u8 cmpl_cnt_th_idx:4;
	/** @cmpl_trig_mode: tigger_mode_t */
	u8 cmpl_trig_mode:3;
	/** @cmpl_en_intr: enable interrupt for CMPT */
	u8 cmpl_en_intr:1;

	/** config flags: byte #6 */
	/** @sw_desc_sz: SW Context desc size, desc_sz_t */
	u8 sw_desc_sz:2;
	/** @pfetch_bypass: prefetch bypass en */
	u8 pfetch_bypass:1;
	/** @cmpl_ovf_chk_dis: OVF_DIS C2H ST over flow disable */
	u8 cmpl_ovf_chk_dis:1;
	/** @port_id: Port ID */
	u8 port_id:3;
	/** @at: Address Translation */
	u8 at:1;

	/** @adaptive_rx: Adaptive rx counter threshold */
	u8 adaptive_rx:1;
	/** @latency_optimize: optimize for latency */
	u8 latency_optimize:1;
	/** @init_pidx_dis : Disable pidx initialiaztion for ST C2H */
	u8 init_pidx_dis:1;
	/** @filler: Filler */
	u8 filler:5;

	/** @en_mm_cmpt: MM Completions enabled? */
	u8 en_mm_cmpt;
	/*
	 * TODO: for Platform streaming DSA
	 */
	/** only if pipe = 1 */
	/** @cdh_max: max 16. CDH length per packet */
	u8 cdh_max;
	/** @pipe_gl_max: <= 7, max # gather buf. per packet */
	u8 pipe_gl_max;
	/** @pipe_flow_id: pipe flow id */
	u8 pipe_flow_id;
	/** @pipe_slr_id: pipe SLR id */
	u8 pipe_slr_id;
	/** @pipe_tdest: pipe route id */
	u16 pipe_tdest;

	/** @quld: user provided per-Q irq handler */
	unsigned long quld;		/* set by user for per Q data */

	/**
	 *  @fp_descq_isr_top: Q interrupt top, per-queue additional handling
	 *  code for example, network rx napi_schedule(&Q->napi)
	 */
	void (*fp_descq_isr_top)(unsigned long qhndl, unsigned long quld);

	/**
	 * @fp_descq_c2h_packet:
	 * optional rx packet handler:
	 *	 called from irq BH (i.e.qdma_queue_service_bh())
	 *
	 * udd: user defined data in the completion entry
	 * sgcnt / sgl: packet data in scatter-gather list
	 *
	 *   NOTE: a. do NOT modify any field of sgl
	 *	   b. if zero copy, do a get_page() to prevent page freeing
	 *	   c. do loop through the sgl with sg->next and stop
	 *	      at sgcnt. the last sg may not have sg->next = NULL
	 *
	 * Returns:
	 *	0 to allow libqdma free/re-task the sgl
	 *	< 0, libqdma will keep the packet for processing again
	 *
	 * A single packet could contain:
	 * in the case of c2h_udd_en = 1:
	 *
	 * udd only (udd valid, sgcnt = 0, sgl = NULL), or
	 * udd + packdet data in the case of c2h_udd_en = 0:
	 * packet data (udd = NULL, sgcnt > 0 and sgl valid)
	 *
	 */
	int (*fp_descq_c2h_packet)(unsigned long qhndl, unsigned long quld,
				unsigned int len, unsigned int sgcnt,
				struct qdma_sw_sg *sgl, void *udd);

	/**
	 * @fp_bypass_desc_fill: fill the all the descriptors required for
	 *                        transfer
	 * q_hndl: handle with which bypass module can request back info from
	 *          libqdma
	 * q_mode: mode in which q is initialized
	 * q_dir: diretion in which q is intialized
	 * sgcnt: number of sctter gather entries for this request
	 * sgl: lsit of scatter gather entries
	 *
	 *  On calling this API, bypass module can request for descriptor using
	 *  qdma_q_desc_get and set up as many descriptors as required for each
	 *  scatter gather entry. If descriptors required are not available,
	 *  it can return the number of sgcnt addressed till now and return <0
	 *  in case of any failure
	 */
	int (*fp_bypass_desc_fill)(void *q_hndl, enum qdma_q_mode q_mode,
			enum qdma_q_dir, struct qdma_request *req);
	/**
	 * @fp_proc_bypass_cmpt_entry: parse cmpt entry in bypass mode
	 * q_mode: mode in which q is initialized
	 * cmpt_entry: cmpt entry descriptor
	 * cmpt_info: parsed bypass releated info from cmpt_entry
	 *
	 */
	int (*fp_proc_ul_cmpt_entry)(void *cmpt_entry,
			struct qdma_ul_cmpt_info *cmpt_info);

	/** fill in by libqdma */
	/** @name: name of the qdma device */
	char name[QDMA_QUEUE_NAME_MAXLEN];
	/** @rngsz: ring size of the queue */
	unsigned int rngsz;
	/** @rngsz_cmpt: completion ring size of the queue */
	unsigned int rngsz_cmpt;
	/** @c2h_bufsz: C2H buffer size */
	unsigned int c2h_bufsz;
};

/*****************************************************************************/
/**
 * qdma_queue_add() - add a queue
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @qconf:		queue configuration parameters
 * @qhndl:	list of unsigned long values that are the opaque qhndl
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_queue_add(unsigned long dev_hndl, struct qdma_queue_conf *qconf,
			unsigned long *qhndl, char *buf, int buflen);

/*****************************************************************************/
/**
 * qdma_queue_config() - configure the queue with qcong parameters
 *
 * @dev_hndl:		dev_hndl returned from qdma_device_open()
 * @qid:		queue id
 * @qconf:		queue configuration parameters
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * @return	0: success
 * @return	<0: error
 *****************************************************************************/
int qdma_queue_config(unsigned long dev_hndl, unsigned long qid,
			struct qdma_queue_conf *qconf, char *buf, int buflen);
/*****************************************************************************/
/**
 * qdma_queue_start() - start a queue (i.e, online, ready for dma)
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @id:		    the opaque qhndl
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_queue_start(unsigned long dev_hndl, unsigned long id,
						char *buf, int buflen);

/*****************************************************************************/
/**
 * qdma_queue_stop() - stop a queue (i.e., offline, NOT ready for dma)
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @id:		the opaque qhndl
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_queue_stop(unsigned long dev_hndl, unsigned long id, char *buf,
				int buflen);
/**
 * struct qdma_q_state - display queue state in a string buffer
 *
 */
struct qdma_q_state {
	/** @qstate: current q state */
	u32 qstate;
	/**
	 *  @qidx: 0xFFFF: libqdma choose the queue idx 0 ~
	 *  (qdma_dev_conf.qsets_max - 1) the calling function choose the
	 *   queue idx
	 */
	u32 qidx:24;
	/** @st: Indicates the streaming mode */
	u32 st:1;
	/** @c2h: c2h direction */
	u32 c2h:1;
	/** @reserved: reserved */
	u32 reserved:6;
};


int qdma_get_queue_state(unsigned long dev_hndl, unsigned long id,
		struct qdma_q_state *qstate, char *buf, int buflen);

/*****************************************************************************/
/**
 * qdma_queue_remove() - remove a queue
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @id:		the opaque qhndl
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * Return:	0 for success and <0 for error
 *
 *****************************************************************************/
int qdma_queue_remove(unsigned long dev_hndl, unsigned long id, char *buf,
				int buflen);


/*****************************************************************************/
/**
 * qdma_queue_get_config() - retrieve the configuration of a queue
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @id:		an opaque queue handle of type unsigned long
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * Return: if optional message buffer used then strlen of buf,
 *	 otherwise QDMA_OPERATION_SUCCESSFUL and <0 for error
 *
 *****************************************************************************/
struct qdma_queue_conf *qdma_queue_get_config(unsigned long dev_hndl,
				unsigned long id, char *buf, int buflen);

/*****************************************************************************/
/**
 * qdma_queue_list() - display all configured queues in a string buffer
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * Return: if optional message buffer used then strlen of buf,
 *	 otherwise QDMA_OPERATION_SUCCESSFUL and <0 for error
 *
 *****************************************************************************/
int qdma_queue_list(unsigned long dev_hndl, char *buf, int buflen);

/*****************************************************************************/
/**
 * qdma_queue_dump() - display a queue's state in a string buffer
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @id:		an opaque queue handle of type unsigned long
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * Return: if optional message buffer used then strlen of buf,
 *	 otherwise QDMA_OPERATION_SUCCESSFUL and <0 for error
 *
 *****************************************************************************/
int qdma_queue_dump(unsigned long dev_hndl, unsigned long id, char *buf,
				int buflen);

/*****************************************************************************/
/**
 * qdma_queue_dump_desc() - display a queue's descriptor ring from index start
 *							~ end in a string buffer
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @id:		an opaque queue handle of type unsigned long
 * @start:		start index
 * @end:		end index
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * Return: if optional message buffer used then strlen of buf,
 *	 otherwise QDMA_OPERATION_SUCCESSFUL and <0 for error
 *
 *****************************************************************************/
int qdma_queue_dump_desc(unsigned long dev_hndl, unsigned long id,
				unsigned int start, unsigned int end, char *buf,
				int buflen);

/*****************************************************************************/
/**
 * qdma_queue_dump_cmpt() - display a queue's descriptor ring from index start
 *							~ end in a string buffer
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @id:		an opaque queue handle of type unsigned long
 * @start:		start index
 * @end:		end index
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * Return: if optional message buffer used then strlen of buf,
 *	 otherwise QDMA_OPERATION_SUCCESSFUL and <0 for error
 *
 *****************************************************************************/
int qdma_queue_dump_cmpt(unsigned long dev_hndl, unsigned long id,
				unsigned int start, unsigned int end, char *buf,
				int buflen);

#ifdef ERR_DEBUG
/*****************************************************************************/
/**
 * qdma_queue_set_err_induction() - Induce the error
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @id:		error id
 * @err:		error info
 * @buflen:		length of the input buffer
 * @buf:		message buffer
 *
 * Return: if optional message buffer used then strlen of buf,
 *	 otherwise QDMA_OPERATION_SUCCESSFUL and <0 for error
 *
 *****************************************************************************/
int qdma_queue_set_err_induction(unsigned long dev_hndl, unsigned long id,
				 u32 err, char *buf, int buflen);
#endif


/** maximum request length */
#define QDMA_REQ_OPAQUE_SIZE	80

/** Max length of the user defined data */
#define QDMA_UDD_MAXLEN		32

/**
 * struct qdma_request - qdma request for read or write
 *
 */
struct qdma_request {
	/** @opaque: private to the dma driver, do NOT touch */
	unsigned char opaque[QDMA_REQ_OPAQUE_SIZE];
	/**
	 *  @uld_data: filled in by the calling function
	 *  for the calling function
	 */
	unsigned long uld_data;
	/** @fp_done: set fp_done for non-blocking mode */
	int (*fp_done)(struct qdma_request *req, unsigned int bytes_done,
			int err);
	/** @timeout_ms: timeout in mili-seconds, 0 - no timeout */
	unsigned int timeout_ms;
	/** @count: total data size */
	unsigned int count;
	/** @ep_addr: MM only, DDR/BRAM memory addr */
	u64 ep_addr;
	/** @no_memcpy:  flag to indicate if memcpy is required */
	u8 no_memcpy:1;
	/** @write: if write to the device */
	u8 write:1;
	/** @dma_mapped: if sgt is already dma mapped */
	u8 dma_mapped:1;
	/** @h2c_eot: user defined data present */
	u8 h2c_eot:1;
	/** @udd_len: indicates end of transfer towards user kernel */
	u8 udd_len;
	/** @sgcnt: # of scatter-gather entries < 64K */
	unsigned int sgcnt;
	/** @sgl: scatter-gather list of data bufs */
	struct qdma_sw_sg *sgl;
	/** @udd: udd data */
	u8 udd[QDMA_UDD_MAXLEN];
};

/*****************************************************************************/
/**
 * qdma_request_submit() - submit a scatter-gather list of data for dma
 * operation (for both read and write)
 *
 * @dev_hndl:	hndl returned from qdma_device_open()
 * @id:			queue index
 * @req:		qdma request
 *
 * Return:	# of bytes transferred for success and  <0 for error
 *
 *****************************************************************************/
ssize_t qdma_request_submit(unsigned long dev_hndl, unsigned long id,
			struct qdma_request *req);

/*****************************************************************************/
/**
 * qdma_batch_request_submit() - submit a scatter-gather list of data for dma
 * operation (for both read and write)
 *
 * @dev_hndl:	hndl returned from qdma_device_open()
 * @id:			queue index
 * @count:			number of requests
 * @reqv:		qdma request
 *
 * Return:	# of bytes transferred for success and  <0 for error
 *
 *****************************************************************************/
ssize_t qdma_batch_request_submit(unsigned long dev_hndl, unsigned long id,
			  unsigned long count, struct qdma_request **reqv);

/*****************************************************************************/
/**
 * qdma_queue_c2h_peek() - peek a receive (c2h) queue
 *
 * @dev_hndl:	hndl returned from qdma_device_open()
 * @qhndl:		hndl returned from qdma_queue_add()
 *
 * filled in by libqdma:
 * @udd_cnt:	# of udd received
 * @pkt_cnt:	# of packets received
 * @data_len:	# of bytes of packet data received
 *
 * Return:	# of packets received in the Q or <0 for error
 *****************************************************************************/
int qdma_queue_c2h_peek(unsigned long dev_hndl, unsigned long qhndl,
			unsigned int *udd_cnt, unsigned int *pkt_cnt,
			unsigned int *data_len);


/*****************************************************************************/
/**
 * qdma_queue_avail_desc() -  query of # of free descriptor
 *
 * @dev_hndl:	hndl returned from qdma_device_open()
 * @qhndl:		hndl returned from qdma_queue_add()
 *
 * Return:	# of available desc in the queue or <0 for error
 *****************************************************************************/
int qdma_queue_avail_desc(unsigned long dev_hndl, unsigned long qhndl);

/** packet/streaming interfaces  */

/**
 * struct qdma_cmpl_ctrl - completion control
 */
struct qdma_cmpl_ctrl {
	/** @cnt_th_idx: global_csr_conf.c2h_cnt_th[N] */
	u8 cnt_th_idx:4;
	/** @timer_idx: global_csr_conf.c2h_timer_cnt[N] */
	u8 timer_idx:4;
	/** @trigger_mode: tigger_mode_t */
	u8 trigger_mode:3;
	/** @en_stat_desc: enable status desc. for CMPT */
	u8 en_stat_desc:1;
	/** @cmpl_en_intr: enable interrupt for CMPT */
	u8 cmpl_en_intr:1;
};


/*****************************************************************************/
/**
 * qdma_queue_cmpl_ctrl() - read/set the c2h Q's completion control
 *
 * @dev_hndl:	hndl returned from qdma_device_open()
 * @qhndl:		hndl returned from qdma_queue_add()
 * @cctrl:		completion control
 * @set:		read or set
 *
 * Return:	0 for success or <0 for error
 *
 *****************************************************************************/
int qdma_queue_cmpl_ctrl(unsigned long dev_hndl, unsigned long qhndl,
				struct qdma_cmpl_ctrl *cctrl, bool set);

/*****************************************************************************/
/**
 * qdma_queue_packet_read() - read rcv'ed data (ST C2H dma operation)
 *
 * @dev_hndl:	hndl returned from qdma_device_open()
 * @qhndl:		hndl returned from qdma_queue_add()
 * @req:		pointer to the request data
 * @cctrl:		completion control, if no change is desired,
 *                      set it to NULL
 *
 * Return:	0 for success or <0 for error
 *
 *****************************************************************************/
int qdma_queue_packet_read(unsigned long dev_hndl, unsigned long qhndl,
			struct qdma_request *req, struct qdma_cmpl_ctrl *cctrl);

/*****************************************************************************/
/**
 * qdma_queue_packet_write() - submit data for h2c dma operation
 *
 * @dev_hndl:	hndl returned from qdma_device_open()
 * @qhndl:		hndl returned from qdma_queue_add()
 * @req:		pointer to the list of packet data
 *
 * Return:	0 for success or <0 for error
 *
 *****************************************************************************/
int qdma_queue_packet_write(unsigned long dev_hndl, unsigned long qhndl,
			struct qdma_request *req);

/*****************************************************************************/
/**
 * qdma_queue_service() - service the queue
 *	in the case of irq handler is registered by the user, the user should
 *	call qdma_queue_service() in its interrupt handler to service the queue
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @qhndl:		hndl returned from qdma_queue_add()
 * @budget:		ST C2H only, max number of completions to be processed.
 * @c2h_upd_cmpl:   flag to update the completion
 *
 *****************************************************************************/
void qdma_queue_service(unsigned long dev_hndl, unsigned long qhndl,
			int budget, bool c2h_upd_cmpl);

/*****************************************************************************/
/**
 * qdma_intr_ring_dump() - display the interrupt ring info of a vector
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @vector_idx:	vector number
 * @start_idx:	interrupt ring start idx
 * @end_idx:	interrupt ring end idx
 * @buflen:		length of the input buffer
 * @buf:		message bufferuffer
 *
 * Return:	0 for success or <0 for error
 *
 *****************************************************************************/
int qdma_intr_ring_dump(unsigned long dev_hndl, unsigned int vector_idx,
			int start_idx, int end_idx, char *buf, int buflen);

/*****************************************************************************/
/**
 * qdma_descq_get_cmpt_udd() - function to receive the user defined data
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @qhndl:		queue handle
 * @buflen:		length of the input buffer
 * @buf:		message bufferuffer
 *
 * Return:	0 for success or <0 for error
 *
 *****************************************************************************/
int qdma_descq_get_cmpt_udd(unsigned long dev_hndl, unsigned long qhndl,
		char *buf, int buflen);


/*****************************************************************************/
/**
 * qdma_descq_read_cmpt_data() - function to receive the completion data
 *
 * @dev_hndl:		dev_hndl returned from qdma_device_open()
 * @qhndl:		queue handle
 * @num_entries:	I/O number of entries
 * @cmpt_entries:	List of completion entries
 * @buflen:		length of the input buffer
 * @buf:		message bufferuffer
 *
 * Return:	0 for success or <0 for error
 *
 *****************************************************************************/
int qdma_descq_read_cmpt_data(unsigned long dev_hndl, unsigned long qhndl,
				u32 *num_entries,  u8 **cmpt_entries,
				char *buf, int buflen);
#ifdef __QDMA_VF__
/*****************************************************************************/
/**
 * qdma_vf_qconf - call for VF to request qmax number of Qs
 *
 * @dev_hndl:	dev_hndl returned from qdma_device_open()
 * @qmax:	    number of qs requested by vf
 *
 * Return:	0 for success or <0 for error
 *
 *****************************************************************************/
int qdma_vf_qconf(unsigned long dev_hndl, int qmax);
#endif

#endif
