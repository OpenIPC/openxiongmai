# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_acodec.ko: OpenIPC firmware names the open modules that replace
# vendor blobs open_*, and the module's name is the file's.
obj-m := open_acodec.o
open_acodec-objs := xm_v200_acodec.o
# ACODEC_CHIP=v200 (XM530V200; default), xm533, xm535 (D1/D2) or xm535w1
ifeq ($(ACODEC_CHIP),xm533)
ccflags-y += -DACODEC_XM533
endif
ifeq ($(ACODEC_CHIP),xm535)
ccflags-y += -DACODEC_XM535
endif
ifeq ($(ACODEC_CHIP),xm535w1)
ccflags-y += -DACODEC_XM535W1
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD ACODEC_CHIP=$(ACODEC_CHIP) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
