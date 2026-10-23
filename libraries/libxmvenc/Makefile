# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
override CFLAGS += -Werror -Wall -std=c99 -O2

OBJECTS = xmvenc.o cabac_tbl.o

libxmvenc.a: $(OBJECTS)
	$(AR) rcs $@ $^

$(OBJECTS): xmvenc.h cabac_tbl.h

clean:
	-rm -f *.o *.a
