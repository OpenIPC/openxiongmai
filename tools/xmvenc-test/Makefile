CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
override CFLAGS += -Werror -Wall -std=c99 -O2
LDFLAGS += -static

xmvenc-test: xmvenc-test.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

clean:
	-rm -f xmvenc-test
