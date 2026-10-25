# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
LIBXMVENC = ../../libraries/libxmvenc
override CFLAGS += -Werror -Wall -std=c99 -O2 -I$(LIBXMVENC)
LDFLAGS += -static
LDLIBS += -lm -lpthread

xmhevc-test: xmhevc-test.c $(LIBXMVENC)/mvb.h $(LIBXMVENC)/libxmvenc.a
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ xmhevc-test.c $(LIBXMVENC)/libxmvenc.a $(LDLIBS)

$(LIBXMVENC)/libxmvenc.a: FORCE
	$(MAKE) -C $(LIBXMVENC) CROSS_COMPILE=$(CROSS_COMPILE)

FORCE:

clean:
	-rm -f xmhevc-test
	$(MAKE) -C $(LIBXMVENC) clean
.PHONY: FORCE clean
