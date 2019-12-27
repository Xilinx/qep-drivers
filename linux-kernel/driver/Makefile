#/*
# * Copyright (c) 2017-2018,  Xilinx, Inc.
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
grep = grep 2>/dev/null

# Only if KSRC/KOBJ were not defined on the command line.
KSRC ?= $(wildcard /lib/modules/$(QEP_DRV_KVER)/source)
KINC  = $(KSRC)/include

ifneq ($(SUBDIRS),)
	EXTRA_CFLAGS += -I. 
	EXTRA_CFLAGS += -I$(SUBDIRS)/libqdma -I$(SUBDIRS)/cmac -I$(SUBDIRS)/stmn -I$(SUBDIRS)/libqdma/qdma_access 
	QEP_CORE_OBJS := $(patsubst $(SUBDIRS)/cmac/%.c,cmac/%.o,$(wildcard $(SUBDIRS)/cmac/*.c))
	QEP_CORE_OBJS += $(patsubst $(SUBDIRS)/stmn/%.c,stmn/%.o,$(wildcard $(SUBDIRS)/stmn/*.c))
	QEP_CORE_OBJS += $(patsubst $(SUBDIRS)/%.c,%.o,$(wildcard $(SUBDIRS)/*.c))
	QEP_LIBQDMA_OBJS := $(patsubst $(SUBDIRS)/libqdma/qdma_access/%.c, libqdma/qdma_access/%.o,$(wildcard $(SUBDIRS)/libqdma/qdma_access/*.c))
	QEP_LIBQDMA_OBJS += $(patsubst $(SUBDIRS)/libqdma/%.c,libqdma/%.o,$(wildcard $(SUBDIRS)/libqdma/*.c))
endif

ifeq ($(DESIGN), qap)
EXTRA_CFLAGS += -DQAP_DESIGN
else
ifeq ($(DESIGN), qap_stmn)
EXTRA_CFLAGS += -DQAP_STMN_DESIGN
endif
endif

qep_cso ?= 1
qep_rss ?= 1
ifneq ($(qep_cso), 0)
EXTRA_CFLAGS += -DQEP_CSO_EN
endif
ifneq ($(qep_rss), 0)
EXTRA_CFLAGS += -DQEP_RSS_EN
endif

EXTRA_CFLAGS += -DDEBUGFS
EXTRA_CFLAGS += -DMBOX_INTERRUPT_DISABLE -DUSER_INTERRUPT_DISABLE 

obj-m += $(MOD_NAME).o
$(MOD_NAME)-objs += $(QEP_CORE_OBJS)
$(MOD_NAME)-objs += $(QEP_LIBQDMA_OBJS)

#ccflags-y := -Wno-format -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-strict-aliasing  -Wno-sign-compare -Wno-unused-function -Wno-comment

ccflags-y := -Wall -Wno-format

# linux <= 3.13 pci_msix_vec_count is not defined.
#
ifeq ($(shell $(grep) -c 'int pci_msix_vec_count' $(KINC)/linux/pci.h),0)
  EXTRA_CFLAGS += -D__PCI_MSI_VEC_COUNT__
endif

# linux < 3.13 list_next_entry is not defined.
#
ifeq ($(shell $(grep) -c 'list_next_entry' $(KINC)/linux/list.h),0)
  EXTRA_CFLAGS += -D__LIST_NEXT_ENTRY__
endif

# linux < 3.18.13 READ_ONCE is not defined.
#
ifneq ($(shell $(grep) -c 'READ_ONCE' $(KINC)/linux/compiler.h),0)
  EXTRA_CFLAGS += -D__READ_ONCE_DEFINED__
endif

all:
	make -C /lib/modules/$(QEP_DRV_KVER)/build M=$(QEP_DRV_HOME) SUBDIRS=$(shell pwd)  modules

clean:
	make -C /lib/modules/$(QEP_DRV_KVER)/build M=$(QEP_DRV_HOME) SUBDIRS=$(shell pwd) clean
	rm -f libqdma/*.o.ur-safe *.o.ur-safe

install:
	install -d ${MODULES_INSTALL_PATH}
	install -t ${MODULES_INSTALL_PATH} $(MOD_NAME).ko

