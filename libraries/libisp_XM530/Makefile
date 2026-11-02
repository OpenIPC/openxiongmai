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
	src/main/isp_color.o \
	src/main/isp_reg.o \
	src/main/isp_stats.o \
	src/main/isp_tune.o \
	src/main/mpi_isp.o \
	src/main/mpi_vda.o \
	arch/xm530/xm530_chroma.o \
	arch/xm530/xm530_geometry.o \
	arch/xm530/xm530_imx335_nr3d.o \
	arch/xm530/xm530_nr3d.o \
	arch/xm530/xm530_start.o

all: libisp_XM530.so

libisp_XM530.so: $(OBJECTS)
	$(CC) -shared -o $@ $^ -lpthread -lm
	$(STRIP) $@

# xm530_geometry() against the vendor library, on the host
HOSTCC ?= cc
test: test/geometry_test.c arch/xm530/xm530_geometry.c arch/xm530/xm530_start.h
	$(HOSTCC) -Werror -Wall -std=c99 -O2 -I../isp/include/isp -Iarch/xm530 \
	    -o test/geometry_test test/geometry_test.c arch/xm530/xm530_geometry.c
	./test/geometry_test

clean:
	-rm -f $(OBJECTS) *.so test/geometry_test

.PHONY: all clean test
