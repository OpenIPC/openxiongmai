PATH=/opt/vtcs_toolchain/arm-eabi-uclibc/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

ifneq ($(KERNELRELEASE),)
obj-m := xm_i2c.o
xm_i2c-objs := proc.o int.o
else
KDIR := $(HOME)/projects/cameras/sdk/XM510/os/kernel/linux-xm510

all:
	$(MAKE) -C $(KDIR) M=$$PWD
	sudo cp xm_i2c.ko /mnt/noc/sdk/majestic.xm510

clean:
	-rm *.ko *.o modules.order Module.symvers *.mod.c

endif
