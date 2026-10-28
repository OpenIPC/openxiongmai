# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_isp.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and this one replaces xm530_isp.ko as drivers/xm510_isp does
# xm510_isp.ko.
obj-m := open_isp.o
open_isp-objs := xm530_isp.o
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
