# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
override CFLAGS += -Werror -Wall -std=c99 -O2

OBJECTS = xm510.o h62_720p.o

libxmisp.a: $(OBJECTS)
	$(AR) rcs $@ $^

$(OBJECTS): xmisp.h script.h

clean:
	-rm -f *.o *.a
