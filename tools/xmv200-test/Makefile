# SPDX-License-Identifier: MIT
CROSS_COMPILE ?= arm-openipc-linux-musleabi-
CC = $(CROSS_COMPILE)gcc
LIB = ../../libraries/libxmv200
override CFLAGS += -Werror -Wall -std=gnu99 -O2 -I$(LIB)

# REPLAY_DATA=<file>: build in the bring-up replay (replay.c)
ifdef REPLAY_DATA
override CFLAGS += -DREPLAY_DATA='"$(REPLAY_DATA)"'
endif

xmv200-test: main.c replay.c $(LIB)/libxmv200.a
	$(CC) $(CFLAGS) -static -o $@ main.c replay.c $(LIB)/libxmv200.a -lpthread -lm

$(LIB)/libxmv200.a: FORCE
	$(MAKE) -C $(LIB) CROSS_COMPILE=$(CROSS_COMPILE)

FORCE:

clean:
	-rm -f xmv200-test

.PHONY: clean FORCE
