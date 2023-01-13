PATH=/opt/vtcs_toolchain/arm-eabi-uclibc/usr/bin:/usr/bin
CROSS_COMPILE=arm-linux-
CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
STRIP=$(CROSS_COMPILE)strip
CFLAGS=-I../isp/include/isp \
	   -I$(HOME)/git/majestic/thirdparty/nda/xm530/include/xm \
	   -I$(HOME)/git/majestic/thirdparty/nda/xm530/include/mpi \
	   -D_GNU_SOURCE \
	   -Werror -Wall -std=c99

##############################################
# CHIP_ID:	XM530
##############################################
CFLAGS += -DCHIPID_XM530
CFLAGS += -DSOC_SYSTEM -DSOC_XMSDK

OBJECTS = \
	com.o \
	extCfg.o \
	i2c.o \
	isp_sample.o \
	mcu.o \
	sample_common.o \
	uart.o \
	vda_sample.o

libispapp_XM530.so: $(OBJECTS)
	$(CC) -shared -o $@ $^
	sudo cp $@ /mnt/noc/sdk/majestic.xm530
	#$(STRIP) $@

clean:
	-rm *.o *.so
