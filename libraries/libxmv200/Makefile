# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
override CFLAGS += -Werror -Wall -std=gnu99 -O2

OBJECTS = sys.o venc.o

libxmv200.a: $(OBJECTS)
	$(AR) rcs $@ $^

$(OBJECTS): xmv200.h

clean:
	-rm -f *.o *.a
