# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_vi.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the XM510 one replaces xm510_vi.ko as drivers/xm_vi does
# xm530_vi.ko.
obj-m := open_vi.o
open_vi-objs := xm510_vi.o
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM510/os/kernel/linux-xm510

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
