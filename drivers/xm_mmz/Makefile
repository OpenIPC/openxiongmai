# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_mmz.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_mmz.o
open_mmz-objs := xm_mmz.o
# MMZ_ALLOCATOR=list: the V200 series' allocator by default (XM530V200,
# XM533, XM535), whose loaders pass only mmz_start and mmz_size
ifneq ($(MMZ_ALLOCATOR),)
ccflags-y += -DMMZ_DEFAULT_ALLOCATOR=\"$(MMZ_ALLOCATOR)\"
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD MMZ_ALLOCATOR=$(MMZ_ALLOCATOR) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
