#!/bin/bash

# Run GDB
gdb-multiarch build/stm32f103c8tx_chademo.elf --command=gdb/debug.gdbinit \
    -ex "continue" &

bg_proc=$!

telnet localhost 53663

kill "$bg_proc"
