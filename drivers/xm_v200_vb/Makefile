# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_vb.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_vb.o
open_vb-objs := xm_v200_vb.o
# VB_LAYOUT=v200 (XM530V200, XM533; default) or xm535 (XM535D1/D2/W1)
ifeq ($(VB_LAYOUT),xm535)
ccflags-y += -DVB_LAYOUT_XM535
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD VB_LAYOUT=$(VB_LAYOUT) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
