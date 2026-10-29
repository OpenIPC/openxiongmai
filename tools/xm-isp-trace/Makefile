# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
override CFLAGS += -Werror -Wall -std=c99 -O2 -fPIC

isptrace.so: isptrace.c isptrace.h
	$(CC) $(CFLAGS) -shared -s -o $@ isptrace.c -ldl -lpthread

clean:
	-rm -f isptrace.so
.PHONY: clean
