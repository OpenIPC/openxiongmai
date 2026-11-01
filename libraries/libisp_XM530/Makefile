CROSS_COMPILE=arm-openipc-linux-musleabi-
CC=$(CROSS_COMPILE)gcc
STRIP=$(CROSS_COMPILE)strip
override CFLAGS += -I../isp/include/isp \
	   -I../isp/include/mpi \
	   -I../isp/include/xm \
	   -Iinclude \
	   -Iarch/xm530 \
	   -D_GNU_SOURCE -DCHIPID_XM530 -DSOC_SYSTEM \
	   -Werror -Wall -std=c99 -fPIC -O2

OBJECTS = \
	src/main/isp_3a.o \
	src/main/isp_reg.o \
	src/main/isp_stats.o \
	src/main/isp_tune.o \
	src/main/mpi_isp.o \
	src/main/mpi_vda.o \
	arch/xm530/xm530_imx335.o \
	arch/xm530/xm530_modes.o

all: libisp_XM530.so

libisp_XM530.so: $(OBJECTS)
	$(CC) -shared -o $@ $^ -lpthread -lm
	$(STRIP) $@

clean:
	-rm -f $(OBJECTS) *.so
