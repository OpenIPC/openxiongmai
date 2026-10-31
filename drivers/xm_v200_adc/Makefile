# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_adc.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_adc.o
open_adc-objs := xm_v200_adc.o
# ADC_CHIP=xm535 for the XM535D1/D2/W1; v200 (XM530V200, XM533) otherwise
ifeq ($(ADC_CHIP),xm535)
ccflags-y += -DADC_XM535
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD ADC_CHIP=$(ADC_CHIP) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
