# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_rgn.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_rgn.o
open_rgn-objs := xm_v200_rgn.o
# RGN_CHIP=v200 (XM530V200; default), xm533 or xm535 (XM535D1/D2/W1)
ifeq ($(RGN_CHIP),xm535)
ccflags-y += -DRGN_CHIP_XM535
endif
ifeq ($(RGN_CHIP),xm533)
ccflags-y += -DRGN_CHIP_XM533
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD RGN_CHIP=$(RGN_CHIP) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
