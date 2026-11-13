# SPDX-License-Identifier: GPL-3.0-only
#
# PC-based host tests for CAN2040-Lite.
#
# Production protocol sources from ../../src are compiled directly into the
# test binary, so the host tests exercise the same code that runs on-target.
# Test-only support (the frame builder) comes from ../common.

CC         ?= cc
CFLAGS     ?= -O2 -std=c11 -Wall -Wextra -Werror
CPPFLAGS   ?=
LDFLAGS    ?=
SRC_DIR    ?= ../../../src
COMMON_DIR ?= ../common

CPPFLAGS += -I$(SRC_DIR) -I$(COMMON_DIR)

# The host suite builds with the opt-in edge-pair capture log enabled so its
# recording and replay paths are covered.  Production builds default it off.
CPPFLAGS += -DCAN_EDGE_PAIR_TIME_RECORDING=1

PROTO_SRCS   := $(wildcard $(SRC_DIR)/*.c)
PIO_SRCS     := $(SRC_DIR)/pico/can_pio_program.c
SUPPORT_SRCS := $(wildcard $(COMMON_DIR)/*.c)

BIN := can_test_units

.PHONY: all test clean

all: $(BIN)

$(BIN): can_test_units.c $(PROTO_SRCS) $(PIO_SRCS) $(SUPPORT_SRCS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ can_test_units.c \
		$(PROTO_SRCS) $(PIO_SRCS) $(SUPPORT_SRCS) $(LDFLAGS)

test: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
