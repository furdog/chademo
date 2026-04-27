#!/bin/bash

# Run GDB
gdb-multiarch build/stm32f103c8tx_chademo.elf --command=gdb/self_test.gdbinit \
    -ex "set dbg_self_test.interactive_mode = ${interactive:-0}" \
    -ex "continue" &

bg_proc=$!

telnet localhost 53663

kill "$bg_proc"
