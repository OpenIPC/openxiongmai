# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_rgn.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and this one replaces xm510_rgn.ko.
obj-m := open_rgn.o
open_rgn-objs := xm510_rgn.o
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM510/os/kernel/linux-xm510

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
