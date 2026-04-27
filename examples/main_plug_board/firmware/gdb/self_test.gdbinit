set pagination off

target extended-remote | openocd -d1 -f interface/stlink.cfg \
	-c "set CPUTAPID 0x2ba01477" -f target/stm32f1x.cfg -c "gdb_port pipe"

monitor reset halt
monitor verify_image build/stm32f103c8tx_chademo.elf

monitor rtt setup 0x20000000 0x5000 "SEGGER RTT"
monitor rtt start
monitor rtt server start 53663 0

tbreak self_test_stm32_run
continue
set var dbg_self_test_enabled = 1
