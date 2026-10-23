# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
LIBXMVENC = ../../libraries/libxmvenc
override CFLAGS += -Werror -Wall -std=c99 -O2 -I$(LIBXMVENC)
LDFLAGS += -static
LDLIBS += -lm -lpthread

xmvenc-test: xmvenc-test.c $(LIBXMVENC)/libxmvenc.a
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ xmvenc-test.c $(LIBXMVENC)/libxmvenc.a $(LDLIBS)

$(LIBXMVENC)/libxmvenc.a: FORCE
	$(MAKE) -C $(LIBXMVENC) CROSS_COMPILE=$(CROSS_COMPILE)

FORCE:

clean:
	-rm -f xmvenc-test
	$(MAKE) -C $(LIBXMVENC) clean
.PHONY: FORCE clean
