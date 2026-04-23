#!/bin/bash

# Run GDB
gdb-multiarch build/stm32f103c8tx_chademo.elf --command=.gdbinit \
	-ex "tbreak main" \
	-ex "continue" \
	-ex "set var dbg_self_test_enabled = 1" \
	-ex "continue&" \
	-ex "shell telnet localhost 53663"
