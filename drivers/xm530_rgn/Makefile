# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_rgn.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and this one replaces xm530_rgn.ko.
obj-m := open_rgn.o
open_rgn-objs := xm530_rgn.o
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
