# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_ddr.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_ddr.o
open_ddr-objs := xm_v200_ddr.o
# DDR_CHIP=v200 for the XM530V200 (the counters stopped while read)
ifeq ($(DDR_CHIP),v200)
ccflags-y += -DDDR_V200
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD DDR_CHIP=$(DDR_CHIP) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
