PATH=/opt/vtcs_toolchain/arm-eabi-uclibc/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

ifneq ($(KERNELRELEASE),)
# open_h264.ko: OpenIPC firmware names the open modules that replace vendor
# blobs open_*, and this one replaces xm510_h264.ko/xm530_h264.ko.
obj-m := open_h264.o
open_h264-objs := hx280enc.o
# H264_CHIP=v200: the XM533's and XM535s' h264.ko (0x21100000, IRQ 78, the
# encoder's clock turned on); the XM510's/XM530's otherwise
ifeq ($(H264_CHIP),v200)
ccflags-y += -DH264_V200
endif
else
KDIR := $(HOME)/projects/cameras/sdk/XM510/os/kernel/linux-xm510

all:
	$(MAKE) -C $(KDIR) M=$$PWD H264_CHIP=$(H264_CHIP)

clean:
	-rm *.ko *.o modules.order Module.symvers *.mod.c

endif
