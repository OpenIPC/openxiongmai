PATH=/opt/vtcs_toolchain/arm-eabi-uclibc/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

ifneq ($(KERNELRELEASE),)
obj-m := xm_i2c.o
else
KDIR := $(HOME)/projects/cameras/sdk/XM510/os/kernel/linux-xm510

all:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	-rm *.ko *.o modules.order Module.symvers *.mod.c

endif
