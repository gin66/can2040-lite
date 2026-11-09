# SPDX-License-Identifier: GPL-3.0-only
#
# Build the can_gen CLI frame generator.

CC         ?= cc
CFLAGS     ?= -O2 -std=c11 -Wall -Wextra -Werror
CPPFLAGS   ?=
LDFLAGS    ?=
SRC_DIR    ?= ../../../src
COMMON_DIR ?= ../../tests/common

CPPFLAGS += -I$(SRC_DIR) -I$(COMMON_DIR)

PROTO_SRCS   := $(wildcard $(SRC_DIR)/*.c)
SUPPORT_SRCS := $(wildcard $(COMMON_DIR)/*.c)

BIN := can_gen

.PHONY: all clean

all: $(BIN)

$(BIN): can_gen.c $(PROTO_SRCS) $(SUPPORT_SRCS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ can_gen.c \
		$(PROTO_SRCS) $(SUPPORT_SRCS) $(LDFLAGS)

clean:
	rm -f $(BIN)
