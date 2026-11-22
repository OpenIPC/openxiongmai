# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_motor.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_motor.o
open_motor-objs := xm_v200_motor.o
# MOTOR_CHIP=xm535d2: the coils through the shift register by default
ifeq ($(MOTOR_CHIP),xm535d2)
ccflags-y += -DMOTOR_IIC
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD MOTOR_CHIP=$(MOTOR_CHIP) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
