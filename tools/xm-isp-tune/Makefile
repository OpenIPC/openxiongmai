# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
override CFLAGS += -Werror -Wall -std=c99 -O2
LDFLAGS += -static

ispreg: ispreg.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ ispreg.c

clean:
	-rm -f ispreg
.PHONY: clean
