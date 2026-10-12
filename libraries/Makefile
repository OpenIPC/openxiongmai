# Builds the userspace libraries for one Xiongmai SoC family, the way
# OpenIPC/firmware's xiongmai-opensdk package calls it:
#
#   make $(TARGET_CONFIGURE_OPTS) CHIPARCH=xm530 -C libraries all
#
# CC, STRIP and CFLAGS come from the caller; each library's Makefile appends
# its own include paths and defines to CFLAGS rather than replacing them.

MAKEFILES = $(shell find . -mindepth 2 -maxdepth 2 -type f -name Makefile)
SUBDIRS   = $(sort $(dir $(MAKEFILES)))

# XM530 (and XM550, which runs the same firmware): the sensor libraries and
# the ISP application layer. libmpi is left out: firmware does not ship it,
# and its Makefile sets PATH to a vendor toolchain.
ifeq ($(CHIPARCH),xm530)
    SUBDIRS := $(filter ./libsns_X50_XM530/ ./libsns_X123_XM530/ ./libispapp_XM530/,$(SUBDIRS))
else
    $(error CHIPARCH=$(CHIPARCH): only xm530 is supported)
endif

all:
	@failed=""; \
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir all || failed="$$failed $$dir"; \
	done; \
	if [ -n "$$failed" ]; then \
		echo; \
		echo "Build failed for:" >&2; \
		for d in $$failed; do echo "  $$d" >&2; done; \
		exit 1; \
	fi

clean:
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean; \
	done

.PHONY: all clean
