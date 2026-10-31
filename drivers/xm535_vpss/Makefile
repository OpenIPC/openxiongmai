# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_vpss.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_vpss.o
open_vpss-objs := xm535_vpss.o
# WITH_VI=1: open_vi (drivers/xm535w1_vi, WITH_VPSS=1) loads beside it and
# has VI's covers and lines (the XM535D1/D2's online BPU)
ifeq ($(WITH_VI),1)
ccflags-y += -DWITH_VI
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD WITH_VI=$(WITH_VI) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
