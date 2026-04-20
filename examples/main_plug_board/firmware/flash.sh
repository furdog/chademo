#!/bin/bash

# Configuration
PROJECT_NAME="stm32f103c8tx_chademo" # Change this to match your .elf name
ELF_FILE="build/${PROJECT_NAME}.elf"
INTERFACE="interface/stlink.cfg"
TARGET="target/stm32f1x.cfg"
# The ID for your specific clone chip
CLONE_ID="0x2ba01477"

if [ ! -f "$ELF_FILE" ]; then
    echo "Error: $ELF_FILE not found. Did you run 'make'?"
    exit 1
fi

echo "Flashing $ELF_FILE to Blue Pill..."

# Run OpenOCD command and exit immediately after flashing
openocd -f "$INTERFACE" \
        -c "set CPUTAPID $CLONE_ID" \
        -f "$TARGET" \
        -c "program $ELF_FILE verify reset exit"
