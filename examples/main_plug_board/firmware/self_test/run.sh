#!/bin/bash

# Run GDB
gdb-multiarch build/stm32f103c8tx_chademo.elf --command=self_test/.gdbinit \
	-ex "tbreak main" \
	-ex "continue" \
	-ex "set var dbg_self_test_enabled = 1" \
	-ex "continue&" &

bg_proc=$!

telnet localhost 53663

kill "$bg_proc"
