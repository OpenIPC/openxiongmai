# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_nn.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and the module's name is the file's.
obj-m := open_nn.o
open_nn-objs := xm_v200_nn.o
# NN_CHIP=xm533 or xm535 (the XM535D1/D2/W1); the XM530V200 otherwise
ifeq ($(NN_CHIP),xm533)
ccflags-y += -DNN_XM533
endif
ifeq ($(NN_CHIP),xm535)
ccflags-y += -DNN_XM535
endif
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD NN_CHIP=$(NN_CHIP) modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
