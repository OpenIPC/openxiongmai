# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
LIBXMISP = ../../libraries/libxmisp
override CFLAGS += -Werror -Wall -std=c99 -O2 -I$(LIBXMISP)
LDFLAGS += -static

xmispd: xmispd.c $(LIBXMISP)/libxmisp.a
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ xmispd.c $(LIBXMISP)/libxmisp.a $(LDLIBS)

$(LIBXMISP)/libxmisp.a: FORCE
	$(MAKE) -C $(LIBXMISP) CROSS_COMPILE=$(CROSS_COMPILE)

FORCE:

clean:
	-rm -f xmispd
	$(MAKE) -C $(LIBXMISP) clean
.PHONY: FORCE clean
