# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_hevc.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_hevc.o
# HEVC_LAYOUT=v3 (XM530V200, XM533) or v1 (XM535D1/D2/W1): the default engine;
# xm232: the XM232's engine, its own driver
ifeq ($(HEVC_LAYOUT),xm232)
open_hevc-objs := xm232_hevc.o
else
open_hevc-objs := xm_hevc.o
ifneq ($(HEVC_LAYOUT),)
ccflags-y += -DHEVC_DEFAULT_LAYOUT=\"$(HEVC_LAYOUT)\"
endif
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD HEVC_LAYOUT=$(HEVC_LAYOUT) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
