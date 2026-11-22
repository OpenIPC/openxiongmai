# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_dma.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_dma.o
# DMA_CHIP=xm232: the XM232's engine (not a PL330); the XM530V200's
# otherwise
ifeq ($(DMA_CHIP),xm232)
open_dma-objs := xm232_dma.o
else
open_dma-objs := xm_v200_dma.o
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD DMA_CHIP=$(DMA_CHIP) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
