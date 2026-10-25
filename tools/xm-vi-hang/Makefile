# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
override CFLAGS += -Werror -Wall -std=c99 -O2 -D_DEFAULT_SOURCE
LDFLAGS += -static

xm-vi-hang: xm-vi-hang.c isp_writes.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ xm-vi-hang.c

clean:
	-rm -f xm-vi-hang
.PHONY: clean
