obj-m	+= gpio-counter.o

ARCH ?= arm
CROSS_COMPILE ?= arm-buildroot-linux-uclibcgnueabihf-
KERNEL_DIR ?= $(HOME)/Workspace/xt4ul/buildroot/output/build/linux-KARO-TXUL-2015-12-04

all:
	make -C $(KERNEL_DIR) \
                ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
                SUBDIRS=$(shell pwd) modules
clean:
	make -C $(KERNEL_DIR) \
                ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
                SUBDIRS=$(shell pwd) clean
