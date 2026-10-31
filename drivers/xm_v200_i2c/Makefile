# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_i2c.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_i2c.o
open_i2c-objs := xm_v200_i2c.o
# I2C_CHIP=xm530v200 (default), xm533 or xm535 (XM535D1/D2/W1)
ifeq ($(I2C_CHIP),xm533)
ccflags-y += -DI2C_CHIP_XM533
endif
ifeq ($(I2C_CHIP),xm535)
ccflags-y += -DI2C_CHIP_XM535
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD I2C_CHIP=$(I2C_CHIP) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
