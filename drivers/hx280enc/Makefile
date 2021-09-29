ifneq ($(KERNELRELEASE),)
obj-m := hx280enc.o
else
KDIR := $(HOME)/projects/cameras/sdk/XM510/os/kernel/linux-xm510

all:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	-rm *.ko *.o modules.order Module.symvers hx280enc.mod.c

endif
