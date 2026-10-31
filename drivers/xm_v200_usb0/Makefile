# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_xm_usb0.ko: OpenIPC firmware names the open modules that replace
# vendor blobs open_*, and the module's name is the file's.
obj-m := open_xm_usb0.o
open_xm_usb0-objs := xm_v200_usb0.o
# USB0_CHIP=xm535 for the XM535D1/D2/W1; the XM530V200's otherwise
ifeq ($(USB0_CHIP),xm535)
ccflags-y += -DUSB0_XM535
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD USB0_CHIP=$(USB0_CHIP) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
