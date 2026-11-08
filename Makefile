# SPDX-License-Identifier: GPL-3.0-only
#
# Convenience top-level Makefile for the host-only parts of CAN2040-Lite.

.PHONY: all test tool clean

all: tool test

tool:
	$(MAKE) -C extras/tools/can_gen

test:
	$(MAKE) -C tests/pc_based test

clean:
	$(MAKE) -C extras/tools/can_gen clean
	$(MAKE) -C tests/pc_based clean
