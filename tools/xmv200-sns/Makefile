# SPDX-License-Identifier: MIT
# make check [FAMILY=xm530v200]: the sensor drivers against the tunings, on
# the build machine
L = ../../libraries/libxmv200
HOSTCC ?= cc
SRC = check.c $(L)/sns.c $(wildcard $(L)/sns_*.c)

check: sns-check
	./sns-check $(FAMILY)

sns-check: $(SRC) $(L)/sns.h $(L)/sns_tuning.h
	$(HOSTCC) -O1 -Wall -Werror -I$(L) -o $@ $(SRC)

clean:
	-rm -f sns-check

.PHONY: check clean
