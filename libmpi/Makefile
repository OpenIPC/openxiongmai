PATH=/opt/vtcs_toolchain/arm-eabi-uclibc/usr/bin
CROSS_COMPILE=arm-linux-
CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
CFLAGS=-I../include -Werror -Wall -std=c99

libopenmpi.a: mpi_sys.o mpi_vi.o mpi_venc.o
	$(AR) rcs $@ $^

clean:
	-rm -f *.o *.a
