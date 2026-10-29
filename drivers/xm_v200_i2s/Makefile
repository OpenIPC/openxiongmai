# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_i2s.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_i2s.o
open_i2s-objs := xm_v200_i2s.o
# I2S_CHIP=v200 (XM530V200, XM533; default), xm535 (D1/D2) or xm535w1
ifeq ($(I2S_CHIP),xm535)
ccflags-y += -DI2S_XM535
endif
ifeq ($(I2S_CHIP),xm535w1)
ccflags-y += -DI2S_XM535 -DI2S_XM535W1
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD I2S_CHIP=$(I2S_CHIP) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
