# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
override CFLAGS += -Werror -Wall -std=c99 -O2 -fPIC

all: isptrace.so 3aprobe.so chromasweep.so

isptrace.so: isptrace.c isptrace.h
	$(CC) $(CFLAGS) -shared -s -o $@ isptrace.c -ldl -lpthread

3aprobe.so: 3aprobe.c
	$(CC) $(CFLAGS) -shared -s -o $@ 3aprobe.c -ldl -lpthread

chromasweep.so: chromasweep.c
	$(CC) $(CFLAGS) -shared -s -o $@ chromasweep.c -ldl -lpthread

clean:
	-rm -f isptrace.so 3aprobe.so chromasweep.so
.PHONY: all clean
