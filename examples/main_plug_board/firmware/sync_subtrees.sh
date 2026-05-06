#!/bin/bash

cd "$(git rev-parse --show-toplevel)"

# Segger RTT
#git subtree add --prefix=examples/main_plug_board/firmware/Drivers/SEGGER/ https://github.com/SEGGERMicro/RTT.git ad6970d813bb12b8a5e34aa94b3a1999f7bd0b6b --squash
git subtree pull --prefix=examples/main_plug_board/firmware/Drivers/SEGGER/ https://github.com/SEGGERMicro/RTT.git ad6970d813bb12b8a5e34aa94b3a1999f7bd0b6b --squash

# Modbus
#git subtree add --prefix=examples/main_plug_board/firmware/agnostic/liblightmodbus/ https://github.com/Jacajack/liblightmodbus.git v3.0 --squash
git subtree pull --prefix=examples/main_plug_board/firmware/agnostic/liblightmodbus/ https://github.com/Jacajack/liblightmodbus.git v3.0 --squash
