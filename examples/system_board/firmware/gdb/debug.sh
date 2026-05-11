#!/bin/bash

# Run GDB
gdb-multiarch build/stm32f103c8tx_chademo.elf --command=gdb/debug.gdbinit \
	-ex "continue"
