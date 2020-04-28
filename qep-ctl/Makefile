#
# Copyright(c) 2019-2020 Xilinx, Inc. All rights reserved.
#
#This source code is free software; you can redistribute it and/or modify it
#under the terms and conditions of the GNU General Public License,
#version 2, as published by the Free Software Foundation.
#
#This program is distributed in the hope that it will be useful, but WITHOUT
#ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#more details.
#
#The full GNU General Public License is included in this distribution in
#the file called "COPYING".
#
 
FLAGS=
FLAGS += -g3 -Wall -Wextra -Wno-sign-compare

CURRDIR=$(shell pwd)

ETH_SRCS = $(wildcard ethtool_qsfp/*.c)
I2C_SRC =  $(wildcard i2c/*.c)
CMAC_SRC = $(wildcard cmac/*.c)

all: qsfp 

qep-ctl:
	gcc qep_ctl.c -o qep-ctl $(FLAGS)

qsfp:
	gcc  $(FLAGS)  $(ETH_SRCS) $(I2C_SRC) $(CMAC_SRC) -DEN_QSFP qep_qsfp.c qep_ctl.c -o qep-ctl  -lm

clean:
	rm -f qep-ctl *.out

