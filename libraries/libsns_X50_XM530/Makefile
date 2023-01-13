PATH=/opt/vtcs_toolchain/arm-eabi-uclibc/usr/bin:/usr/bin
CROSS_COMPILE=arm-linux-
CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
STRIP=$(CROSS_COMPILE)strip
CFLAGS=-I../isp/include/isp \
	   -I../isp/include/xm \
	   -Werror -Wall -std=c99

##############################################
# CHIP_ID:	XM530
##############################################
CFLAGS += -DCHIPID_XM530
CFLAGS += -DSOC_SYSTEM
CFLAGS += -DAWB_ALGO_V2

OBJECTS = \
	IMX335_cmos.o \
	IMX335_sensor_ctl.o \
	k03_cmos.o \
	k03_sensor_ctl.o \
	SC335E_cmos.o \
	SC335E_sensor_ctl.o \
	SC5235_cmos.o \
	SC5235_sensor_ctl.o \
	SC5239_cmos.o \
	SC5239_sensor_ctl.o \
	SC5332_cmos.o \
	SC5332_sensor_ctl.o \
	X50_cmos.o \
	X50_sensor_ctl.o

libsns_X50_XM530.so: $(OBJECTS)
	$(CC) -shared -o $@ $^
	$(STRIP) $@

clean:
	-rm *.o *.so
