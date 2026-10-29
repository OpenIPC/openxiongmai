# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
override CFLAGS += -Werror -Wall -std=gnu99 -O2

# the sensor tunings and their registry (tools/xmv200-capture/gen_all.sh)
SENSORS = $(patsubst %.c,%.o,$(wildcard sns_*_tuning.c)) sns_registry.o
OBJECTS = sys.o venc.o isp.o ae.o chip.o board.o layout.o $(SENSORS)

libxmv200.a: $(OBJECTS)
	$(AR) rcs $@ $^

$(OBJECTS): xmv200.h xmv200_priv.h sns_tuning.h

clean:
	-rm -f *.o *.a
