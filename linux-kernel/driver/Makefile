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

MOD_NAME := qep_drv
QEP_DRV_HOME := $(shell pwd)
QEP_DRV_KVER := $(shell uname -r)

ifneq ($(SUBDIRS),)
	EXTRA_CFLAGS += -I. 
	EXTRA_CFLAGS += -I$(SUBDIRS)/libqdma -I$(SUBDIRS)/cmac -I$(SUBDIRS)/stmn -I$(SUBDIRS)/libqdma/qdma_access 
	QEP_CORE_OBJS := $(patsubst $(SUBDIRS)/cmac/%.c,cmac/%.o,$(wildcard $(SUBDIRS)/cmac/*.c))
	QEP_CORE_OBJS += $(patsubst $(SUBDIRS)/stmn/%.c,stmn/%.o,$(wildcard $(SUBDIRS)/stmn/*.c))
	QEP_CORE_OBJS += $(patsubst $(SUBDIRS)/%.c,%.o,$(wildcard $(SUBDIRS)/*.c))
	QEP_LIBQDMA_OBJS := $(patsubst $(SUBDIRS)/libqdma/qdma_access/%.c, libqdma/qdma_access/%.o,$(wildcard $(SUBDIRS)/libqdma/qdma_access/*.c))
	QEP_LIBQDMA_OBJS += $(patsubst $(SUBDIRS)/libqdma/%.c,libqdma/%.o,$(wildcard $(SUBDIRS)/libqdma/*.c))
endif

STM_ENABLE = 1
ifneq ($(STM_ENABLE), 0)
EXTRA_CFLAGS += -DSTMN_ENABLE
else
EXTRA_CFLAGS += -DQEP_ONLY_DESIGN
endif
ifneq ($(QUEUE_BASE),)
EXTRA_CFLAGS += -DQEP_QUEUE_BASE=$(QUEUE_BASE)
endif
EXTRA_CFLAGS += -DDEBUGFS
EXTRA_CFLAGS += -DMAILBOX_INTERRUPT_DISABLE -DUSER_INTERRUPT_DISABLE 

obj-m += $(MOD_NAME).o
$(MOD_NAME)-objs += $(QEP_CORE_OBJS)
$(MOD_NAME)-objs += $(QEP_LIBQDMA_OBJS)


#ccflags-y := -Wno-format -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-strict-aliasing  -Wno-sign-compare -Wno-unused-function -Wno-comment

ccflags-y := -Wall -Wno-format

all:
	make -C /lib/modules/$(QEP_DRV_KVER)/build M=$(QEP_DRV_HOME) SUBDIRS=$(shell pwd)  modules

clean:
	make -C /lib/modules/$(QEP_DRV_KVER)/build M=$(QEP_DRV_HOME) SUBDIRS=$(shell pwd) clean
	rm -f libqdma/*.o.ur-safe *.o.ur-safe

install:
	install -d ${MODULES_INSTALL_PATH}
	install -t ${MODULES_INSTALL_PATH} $(MOD_NAME).ko

