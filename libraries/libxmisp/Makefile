# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
override CFLAGS += -Werror -Wall -std=c99 -O2

OBJECTS = xm510.o ae.o awb.o tune.o sensor_h62.o h62_720p.o

libxmisp.a: $(OBJECTS)
	$(AR) rcs $@ $^

$(OBJECTS): xmisp.h xmisp_priv.h script.h ae.h awb.h tune.h

clean:
	-rm -f *.o *.a test/ae_test test/awb_test test/tune_test

# The 3A layers on the host: make test
HOSTCC ?= cc
test: test/ae_test.c ae.c ae.h test/awb_test.c awb.c awb.h test/tune_test.c tune.c tune.h
	$(HOSTCC) -Werror -Wall -std=c99 -O2 -o test/ae_test test/ae_test.c ae.c -lm
	$(HOSTCC) -Werror -Wall -std=c99 -O2 -o test/awb_test test/awb_test.c awb.c -lm
	$(HOSTCC) -Werror -Wall -std=c99 -O2 -o test/tune_test test/tune_test.c tune.c -lm
	./test/ae_test
	./test/awb_test
	./test/tune_test
.PHONY: test
