# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
override CFLAGS += -Werror -Wall -std=c99 -O2 -D_DEFAULT_SOURCE
LDFLAGS += -static

xm-rgn-test: xm-rgn-test.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ xm-rgn-test.c

clean:
	-rm -f xm-rgn-test
.PHONY: clean
