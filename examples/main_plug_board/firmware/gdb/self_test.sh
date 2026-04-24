#!/bin/bash

# Run GDB
gdb-multiarch build/stm32f103c8tx_chademo.elf --command=gdb/self_test.gdbinit &

bg_proc=$!

telnet localhost 53663

kill "$bg_proc"
