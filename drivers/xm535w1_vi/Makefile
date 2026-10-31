# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_vi.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_vi.o
open_vi-objs := xm535w1_vi.o
# WITH_VPSS=1: on the XM535D1/D2 beside open_vpss (built WITH_VI=1), which
# has the VPSS hooks (the online BPU's picture goes to VI)
ifeq ($(WITH_VPSS),1)
ccflags-y += -DWITH_VPSS
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD WITH_VPSS=$(WITH_VPSS) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
