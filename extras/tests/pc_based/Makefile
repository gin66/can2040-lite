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

PROTO_SRCS   := $(wildcard $(SRC_DIR)/*.c)
SUPPORT_SRCS := $(wildcard $(COMMON_DIR)/*.c)

BIN := can_test_units

.PHONY: all test clean

all: $(BIN)

$(BIN): can_test_units.c $(PROTO_SRCS) $(SUPPORT_SRCS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ can_test_units.c \
		$(PROTO_SRCS) $(SUPPORT_SRCS) $(LDFLAGS)

test: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
