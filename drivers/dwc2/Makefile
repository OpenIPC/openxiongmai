# SPDX-License-Identifier: GPL-2.0
ifneq ($(KERNELRELEASE),)
# open_dwc2.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*. Linux 4.19's DWC2 driver, host-only, core and platform glue
# in one module, as the vendor dwc2.ko.
obj-m := open_dwc2.o
open_dwc2-objs := platform.o core.o core_intr.o hcd.o hcd_intr.o \
	hcd_queue.o hcd_ddma.o params.o
ccflags-y := -DCONFIG_USB_DWC2_HOST=1 -I$(src)/include -include $(src)/compat.h
else
KDIR ?= $(HOME)/projects/cameras/sdk/XM530/os/kernel/linux-xm530

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	-rm -f *.ko *.o modules.order Module.symvers *.mod.c .*.cmd
	-rm -rf .tmp_versions

endif
