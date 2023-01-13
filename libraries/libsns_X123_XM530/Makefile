CROSS_COMPILE=arm-xm-linux-
CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
STRIP=$(CROSS_COMPILE)strip
CFLAGS=-I../isp/include/isp \
	   -I../isp/include/xm \
	   -Werror -Wall -std=c99 -fPIC

##############################################
# CHIP_ID:	XM530
##############################################
CFLAGS += -DCHIPID_XM530
CFLAGS += -DSOC_SYSTEM
CFLAGS += -DAWB_ALGO_V2

OBJECTS = \
	auge_cmos.o \
	ext_cmos.o \
	ext_sensor_ctl.o \
	imx307_cmos.o \
	imx323_cmos.o \
	mis2003_cmos.o \
	ov9732_cmos.o \
	sc1235_cmos.o \
	sc2145_cmos.o \
	sc2145H_cmos.o \
	sc2235_cmos.o \
	sc2235E_cmos.o \
	sc2235P_cmos.o \
	sc2335_cmos.o \
	sc3035_cmos.o \
	sc3035_sensor_ctl.o \
	sc307E_cmos.o \
	sc4236_cmos.o \
	soif37_cmos.o \
	soih62_cmos.o \
	soih65_cmos.o \
	sp140A_cmos.o \
	sp2305_cmos.o \
	XAx_cmos.o \
	XAx_sensor_ctl.o

libsns_X123_XM530.so: $(OBJECTS)
	$(CC) -shared -o $@ $^
	$(STRIP) $@

clean:
	-rm *.o *.so
