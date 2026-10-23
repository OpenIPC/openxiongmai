CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
override CFLAGS += -Werror -Wall -std=c99 -O2
LDFLAGS += -static
LDLIBS += -lm

xmvenc-test: xmvenc-test.c cabac_tbl.c cabac_tbl.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ xmvenc-test.c cabac_tbl.c $(LDLIBS)

clean:
	-rm -f xmvenc-test
