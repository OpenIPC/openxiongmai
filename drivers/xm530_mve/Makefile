# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_mve.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and this one replaces mve_base.ko. It needs the vendor
# mve_rsrc.ko loaded first.
obj-m := open_mve.o
open_mve-objs := xm530_mve.o mve_session.o mve_mem.o
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
