#/*
# * Copyright (c) 2017-2020,  Xilinx, Inc.
# * All rights reserved.
# *
# *  This source code is free software; you can redistribute it and/or modify it
# *  under the terms and conditions of the GNU General Public License,
# *  version 2, as published by the Free Software Foundation.
# *
# *  This program is distributed in the hope that it will be useful, but WITHOUT
# *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# *  more details.
# *
# *  The full GNU General Public License is included in this distribution in
# *  the file called "COPYING".
# */

# /* BUILD VARIANT
#  * 1. By default, make the driver for QEP design
#  * 2. DESIGN=qap : compiles the driver for QAP only design
#  * 3. DESIGN=qap_stmn : compiles the driver for QAP_STMN  design
#  */


MOD_NAME := qep_drv
QEP_DRV_HOME := $(shell pwd)
QEP_DRV_KVER := $(shell uname -r)


SRC_FOLDERS = . qep_fdt/libfdt qep_fdt cmac stmn libqdma/qdma_access libqdma  \
libqdma/qdma_access/qdma_soft_access libqdma/qdma_access/eqdma_soft_access libqdma/qdma_access/qdma_s80_hard_access mailbox

ifneq ($(SUBDIRS),)
	QEP_OBJS = $(foreach CURR, $(SRC_FOLDERS), $(patsubst $(SUBDIRS)/$(CURR)/%.c, $(CURR)/%.o, $(wildcard $(SUBDIRS)/$(CURR)/*.c)))
	EXTRA_CFLAGS = $(foreach CURR, $(SRC_FOLDERS), -I$$SUBDIRS/$(CURR))
endif


ifeq ($(DESIGN), qap)
EXTRA_CFLAGS += -DQAP_DESIGN
else
ifeq ($(DESIGN), qap_stmn)
EXTRA_CFLAGS += -DQAP_STMN_DESIGN
endif
endif

jumbo_disable ?= 0
sw_counter_en ?= 0
ifeq ($(jumbo_disable), 1)
EXTRA_CFLAGS += -DQEP_JUMBO_DISABLE
endif
ifeq ($(sw_counter_en), 1)
EXTRA_CFLAGS += -DQEP_SW_COUNTER_EN
endif


EXTRA_CFLAGS += -DDEBUGFS
EXTRA_CFLAGS += -DMBOX_INTERRUPT_DISABLE -DUSER_INTERRUPT_DISABLE

obj-m += $(MOD_NAME).o
$(MOD_NAME)-objs += $(QEP_OBJS)

ccflags-y := -Wall

all:
	make -C /lib/modules/$(QEP_DRV_KVER)/build M=$(QEP_DRV_HOME) SUBDIRS=$(shell pwd)  modules

clean:
	make -C /lib/modules/$(QEP_DRV_KVER)/build M=$(QEP_DRV_HOME) SUBDIRS=$(shell pwd) clean
	rm -f libqdma/*.o.ur-safe *.o.ur-safe

install:
	install -d ${MODULES_INSTALL_PATH}
	install -t ${MODULES_INSTALL_PATH} $(MOD_NAME).ko



