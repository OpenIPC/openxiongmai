# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
override CFLAGS += -Werror -Wall -std=c99 -O2 -I.

# A sensor's directory: its 3A driver (sensor.c) if it has one, and a
# start-up part list per mode (tools/xm-isp-trace/extract.py).
SENSORS = sensors/soi_jxh62/sensor.o sensors/soi_jxh62/jxh62_720p.o \
          sensors/soi_jxh42/h42_720p.o sensors/soi_jxh65/h65_720p.o \
          sensors/smartsens_sc1235/sc1235_720p.o \
          sensors/smartsens_sc1245/sc1245_720p.o \
          sensors/smartsens_sc2235/sc2235_1080p.o \
          sensors/onsemi_ar0130/ar0130_720p.o \
          sensors/omnivision_ov9750/ov9750_720p.o
SCRIPTS = script.o scripts.o xm510_start.o $(SENSORS)
OBJECTS = xm510.o ae.o awb.o tune.o $(SCRIPTS)

libxmisp.a: $(OBJECTS)
	$(AR) rcs $@ $^

$(OBJECTS): xmisp.h xmisp_priv.h script.h ae.h awb.h tune.h

clean:
	-rm -f *.o sensors/*/*.o *.a test/ae_test test/awb_test test/tune_test \
	    test/script_test

# The 3A layers on the host: make test
HOSTCC ?= cc
SCRIPT_SRC = $(SCRIPTS:.o=.c)
test: test/ae_test.c ae.c ae.h test/awb_test.c awb.c awb.h test/tune_test.c \
      tune.c tune.h test/script_test.c test/h62_720p_flat.c script.h \
      $(SCRIPT_SRC)
	$(HOSTCC) -Werror -Wall -std=c99 -O2 -o test/ae_test test/ae_test.c ae.c -lm
	$(HOSTCC) -Werror -Wall -std=c99 -O2 -o test/awb_test test/awb_test.c awb.c -lm
	$(HOSTCC) -Werror -Wall -std=c99 -O2 -o test/tune_test test/tune_test.c tune.c -lm
	$(HOSTCC) -Werror -Wall -std=c99 -O2 -I. -o test/script_test \
	    test/script_test.c test/h62_720p_flat.c $(SCRIPT_SRC) -lm
	./test/ae_test
	./test/awb_test
	./test/tune_test
	./test/script_test
.PHONY: test
