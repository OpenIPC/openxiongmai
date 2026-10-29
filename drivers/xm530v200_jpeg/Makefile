# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_jpeg.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_jpeg.o
open_jpeg-objs := xm530v200_jpeg.o
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
