include $(PWD)/../Makefile.param

CFLAGS += -I$(PWD)/include
CFLAGS += -I$(REL_INC)/svp_npu
CFLAGS += -O3

SMP_SRCS := $(wildcard ./*.c)
SMP_SRCS += $(wildcard ./sample_svp_npu/*.c)
SMP_SRCS += $(wildcard $(PWD)/../common/*.c)
TARGET := sample_svp_npu_main
TARGET_PATH := $(PWD)

# compile linux or LITEOS
include $(PWD)/../../$(ARM_ARCH)_$(OSTYPE).mak
