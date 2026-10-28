PATH=/opt/vtcs_toolchain/arm-eabi-uclibc/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

ifneq ($(KERNELRELEASE),)
# open_h264.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and this one replaces xm510_h264.ko/xm530_h264.ko.
obj-m := open_h264.o
open_h264-objs := hx280enc.o
else
KDIR := $(HOME)/projects/cameras/sdk/XM510/os/kernel/linux-xm510

all:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	-rm *.ko *.o modules.order Module.symvers *.mod.c

endif
