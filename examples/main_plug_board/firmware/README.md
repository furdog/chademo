# **CHAdeMO** controller inside charging plug
This page describes **CHAdeMO** controller implemented inside charger plug.
The software utilises [Segger RTT](#debug-segger) for now. Though other debug methods are available.

> This software is on early stage of development (WIP)

see [Implementation notes](#implementation-notes) for more info.

# Debug (basics)
> [!NOTE]
>
> this section explains basics of working with gdb. This does not covers logging.
>
> Logging capabilities are explained in other debug sections.

#setup:
Assumes windows, msys2, gdb-none-eabi-multiarch, openocd, two terminal instances, at least ST-LINK-V2.
Linux setup migh be similar.

---
#terminal1 example (use flag `-c "set CPUTAPID ***"` for stm32 clones):
```
openocd -f interface/stlink.cfg -c "set CPUTAPID 0x2ba01477" -f target/stm32f1x.cfg
```
---
#terminal2 example:
```
gdb-multiarch.exe build/stm32f103c8tx_chademo.elf
(gdb) target remote :3333          # Use :4242 if using st-util
(gdb) monitor reset halt           # Resets the chip and freezes it
(gdb) continue                     # Starts the program

#CTRL+C to return from the program to gdb shell
(gdb) backtrace or bt              # for backtrace
(gdb) info registers               # print registers
(gdb) list                         # to see the C code around the crash point.
(gdb) break #func_name             # to set breakpoint at #func_name
(gdb) print #variable_name         # to print variable name
```

### Example steps to debug `MX_CAN_Init`
* **Reset the chip:** `monitor reset halt`
* **Set a breakpoint at the start of CAN init:** `break MX_CAN_Init`
* **Run:** `continue`
* **Step through the code:** Type `n` (next) line by line.
* **Watch the return value:** Look for a line like `if (HAL_CAN_Init(&hcan) != HAL_OK)`.
* When you hit the `Error_Handler()`, type `print hcan.ErrorCode`.

### What the ErrorCode bits mean:
* **0x01:** `HAL_CAN_ERROR_EWG` (Protocol Error)
* **0x02:** `HAL_CAN_ERROR_EPV` (Passive Error)
* **0x04:** `HAL_CAN_ERROR_BOF` (Bus Off)
* **0x20:** `HAL_CAN_ERROR_TIMEOUT` (**Most Likely**)

# Debug (semihosting)
> [!NOTE]
> This section explains how to log stuff from firmware into gdb session.
> 
> This method requires GDB. Without it, your program will step on the breakpoint and freezes until gdb tells to continue.
> 
> This debugging method is based on breakpoints. Thus very slow and is not recomended for truly real time debugging.

To proceed, modify makefile:
Remove `Core/Src/syscalls.c \` from build files list.

Use `-lrdimon` instead of `-lnosys`
```makefile
#LIBS = -lc -lm -lnosys 
LIBS = -lc -lm -lrdimon
```

Use `--specs=rdimon.specs` instead `-specs=nano.specs`
```makefile
LIBDIR = 
#LDFLAGS = $(MCU) -specs=nano.specs ...
LDFLAGS = $(MCU) --specs=rdimon.specs ...
```

Edit `main.c`:
```c
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>

extern void initialise_monitor_handles(void);

#define GDB_PRINTF(fmt, ...) \
    printf(fmt, ##__VA_ARGS__);

/* USER CODE END Includes */
```

```C
/* USER CODE BEGIN 1 */
	void initialise_monitor_handles();
/* USER CODE END 1 */
```

Call `GDB_PRINTF("fmt", ...)` inside code wenever you like.

Run debugger.
`.gdbinit` script may look like this:
```bash
target extended-remote | openocd -d1 -f interface/stlink.cfg \
	-c "set CPUTAPID 0x2ba01477" -f target/stm32f1x.cfg -c "gdb_port pipe"
monitor arm semihosting enable
monitor reset halt

continue
```

Take note that `set CPUTAPID 0x2ba01477` is only for stm32f103 clones

To run gdb session:
`gdb-multiarch build/stm32f103c8tx_chademo.elf --command=.gdbinit`
(may differ under various OS)

# Debug (Alternative to semihosting)
> [!NOTE]
> This section explains how to log stuff from firmware into gdb session.
>
> This method has simpler setup than semihosting.
>
> It also doesn't freezes program without gdb running.
>
> This debugging method is based on breakpoints. Thus very slow and is not recomended for truly real time debugging.

Edit `main.c`:
```c
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>

char gdb_log_buffer[128]; // Global buffer to avoid stack issues

__attribute__((noinline)) void gdb_log_trigger(void) {
	asm volatile("nop"); // Just a place for the breakpoint to land
}

#define GDB_PRINTF(fmt, ...) \
	snprintf(gdb_log_buffer, sizeof(gdb_log_buffer), fmt, ##__VA_ARGS__); \
	gdb_log_trigger();

/* USER CODE END Includes */
```

Call `GDB_PRINTF("fmt", ...)` inside code wenever you like.

Run debugger.
`.gdbinit` script may look like this:
```bash
target extended-remote | openocd -d1 -f interface/stlink.cfg \
	-c "set CPUTAPID 0x2ba01477" -f target/stm32f1x.cfg -c "gdb_port pipe"
monitor reset halt

break gdb_log_trigger
commands
    silent
    printf "%s", gdb_log_buffer
    continue
end

continue
```
Take note that `set CPUTAPID 0x2ba01477` is only for stm32f103 clones

To run gdb session:
`gdb-multiarch build/stm32f103c8tx_chademo.elf --command=.gdbinit`
(may differ under various OS)

# Debug (ITM)
Not implemented yet… (Requires flashing hardware modifications or other flashing tools)

# Debug (MCUViewer)
Use the MCUViewer software to debug memory in real time.
available https://github.com/klonyyy/MCUViewer

> [!NOTE]
>
> This debugging method is only good for monitoring variables and is not suitable for logging

# Debug (SEGGER)
> [!NOTE]
>
> This method is complicated to setup, but is superior for real time logging.
>
> This method assumes running of separate telnet terminal

download https://github.com/SEGGERMicro/RTT, unpack into SEGGER directory.

To proceed, modify makefile:

Remove from sources: `#Core/Src/syscalls.c \`

Add to sources:
`SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c \
SEGGER/RTT/SEGGER_RTT.c \
SEGGER/RTT/SEGGER_RTT_printf.c`

Add to includes:
`-ISEGGER/Config \
-ISEGGER/RTT`

add to ASSM sources (optional):
`# ASMM sources
ASMM_SOURCES =  \
SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.S`

Edit `main.c`:
```c
  /* USER CODE BEGIN 1 */
  SEGGER_RTT_Init();
  /* USER CODE END 1 */
```
Now you can log by using printf-like function:
```c
SEGGER_RTT_printf(0, "Hello world! %i\n", some_int_number);

/* Normal printf will work too! */
printf("Hello world! %i\n", some_int_number);
```

Run debugger.
`.gdbinit` script may look like this:
```bash
target extended-remote | openocd -d1 -f interface/stlink.cfg \
	-c "set CPUTAPID 0x2ba01477" -f target/stm32f1x.cfg -c "gdb_port pipe"

monitor reset halt

monitor rtt setup 0x20000000 0x5000 "SEGGER RTT"
monitor rtt start
monitor rtt server start 53663 0

continue
```
Take note that `set CPUTAPID 0x2ba01477` is only for stm32f103 clones

To run gdb session:
`gdb-multiarch build/stm32f103c8tx_chademo.elf --command=.gdbinit`
(may differ under various OS)

Run in separate terminal `telnet localhost 53663` (53663 is a leetspeak for (SEGGE)r. Easy to memorize)

# Implementation notes
(20.04.2026)
First step was to use [Segger RTT](#debug-segger) and other tools for debugging.

For now there is a need to manage self test.
In order to do that, i define volatile variables:
```C
volatile bool dbg_self_test_enabled = false;
```
This variable should be flagged by debugger at start to:
- enable all relays, communication channels, etc one after another
- read and validate inputs, so everything is in valid state
- log status into a telnet session (`telnet localhost 53663`)

Self-test procedure should be interactive and implemented as simple automata
that guides user about further steps (shorting specific pins, sending messages, etc).

I have created `run_self_test_procedure.sh` script to provide this functionality fully automated.

(22.04.2026)
I have implemented `self_test.h` automata to self test hardware. Then i defined in `main.c`:
```C
volatile struct dbg_self_test dbg_self_test;
```
This automata should be only ran if `dbg_self_test_enabled` is true

(23.04.2026)
So, for now my automata just automatically enables and disables certain outputs
5 times with certain intervals. LED's and relays are tested

I have wrote script:
```bash
gdb-multiarch build/stm32f103c8tx_chademo.elf --command=.gdbinit \
	-ex "tbreak main" \
	-ex "continue" \
	-ex "set var dbg_self_test_enabled = 1" \
	-ex "continue&" \
	-ex "shell telnet localhost 53663"
```
Cool stuff: some commands can run in background by providing & after

> [!WARNING]
> Ok. I have found out that there is no safe way to run `-ex "shell telnet localhost 53663"`.
> It causes keep_alive() error, while openocd runs in pipe mode
> gdb either should run as separate process through tcp socket, or telnet must be used in separate window...

(24.04.2026)
Resolved gdb+openocd in the same window, by just moving gdb onto backgound.

I have added `monitor verify_image build/stm32f103c8tx_chademo.elf` so, now debugging wont start until there firmware matches

I have decided to add custom targets to simplify build process 
```makefile
#######################################
# CUSTOM TARGETS
#######################################
```
Everything works as intended. The next step is to make rtt more interactive.
So my plan is to send and parse telnet input. For example: `press enter to
go to the next testing phase.`

I have added a copy of RTT knowledge base wiki: `RTT - SEGGER Knowledge Base.mhtml`

