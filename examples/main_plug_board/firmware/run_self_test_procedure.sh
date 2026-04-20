#!/bin/bash

# Run GDB
gdb-multiarch build/stm32f103c8tx_chademo.elf --command=.gdbinit \
	-ex "set var dbg_self_test_enabled = 1"
