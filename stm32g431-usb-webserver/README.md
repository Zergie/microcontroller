# STM32G431CBU6 USB-NCM webserver

GPIO controller for the STM32G431CBU6 (UFQFPN48): USB CDC-NCM, IPv4, DHCP,
HTTP and the Graphite GPIO UI adapted from `rp2040-usb-webserver`.
No RTOS, mDNS, DNS server, captive portal or persistent GPIO settings.
`POST /api/system/dfu` enters the factory bootloader for USB firmware updates.

## GPIO UI

The responsive UI retains pin selection, applied-state markers, per-pin drafts,
Apply, Refresh and pin information. Names are STM32 ports, not Pico GP numbers.
The drawing follows the **WeAct QFN48 board top view**, with USB at the top
(the supplied landscape reference rotated 90 degrees clockwise). Both rows of
each 2×12 header are shown; power/reference/reset contacts are information-only.
The SWD header is at the bottom: 3V3, DIO/PA13, CLK/PA14, GND, left to right.
BOOT0 and NRST sit beside USB; KEY C13 is at the lower right.
This is the board layout, not the MCU package pin numbering.
Pin placement follows the supplied reference and
[WeAct hardware files](https://github.com/WeActStudio/WeActStudio.STM32G431CoreBoard/tree/master/Hardware/QFN48).
Check your board revision before connecting peripherals.

Each line below is one horizontal pair, listed from USB toward the bottom:

| Row | Left outer | Left inner | Right inner | Right outer |
| --- | --- | --- | --- | --- |
| 1 | VCC | VCC | 3V3 | 3V3 |
| 2 | GND | GND | GND | GND |
| 3 | PB12 | PB13 | PB10 | PB11 |
| 4 | PB14 | PB15 | PB2 | V+ (VREF+) |
| 5 | PC6 | PA8 | PB0 | PB1 |
| 6 | PA9 | PA10 | PA7 | PC4 |
| 7 | PA11 | PA12 | PA5 | PA6 |
| 8 | PA15 | PC10 | PA3 | PA4 |
| 9 | PC11 | PB3 | PA1 | PA2 |
| 10 | PB4 | PB5 | NRST | PA0 |
| 11 | PB6 | PB7 | PC14 | PC15 |
| 12 | PB8 | PB9 | VB (VBAT) | PC13 |

- Input, pull-up, pull-down, output and open-drain with Slow/Fast slew settings.
- Live levels update every 250 ms while the page is visible, without clicking
  Refresh, changing editor values or discarding drafts. Polls do not overlap.
- On the WeAct Core Board, C13 is active HIGH and needs a pull-down: released
  is L, pressed is H. PC13 now starts in Input – Pull-down; other editable pins
  start without pulls ([board definition](https://raw.githubusercontent.com/zephyrproject-rtos/zephyr/main/boards/weact/stm32g431_core/weact_stm32g431_core.dts)).
- Hardware PWM, 10–100,000 Hz and 0–100% duty: PA0–3 (TIM2), PA6/7 and
  PB0/1 (TIM3), PB6/7/9 (TIM4). Timer channels share frequency; conflicts are
  rejected without changing the pin. Actual integer-timer frequency is reported.
- PA11/12 (USB), PA13/14 (SWD), PB8 (BOOT0), PC14/15 and PF0/1 (oscillators)
  remain reserved. Other exposed GPIOs start as inputs (PC13 with pull-down).
- ADC is not implemented. Pico bus keeper, selectable drive current and
  selectable input hysteresis are omitted. No onboard LED mapping is assumed.
- Settings are volatile. An empty-body POST applies a command; GET only reads.
  `/api/gpio.json` returns 41 entries: PA0–15, PB0–15, PC4/6/13/14/15, PF0/1,
  then PC10/PC11. Existing indices 0–38 are unchanged; PC10/PC11 are 39/40.
  PF0/PF1 belong to the oscillator and are not header contacts in this drawing.
  `/api/gpio/levels.json` returns a compact 41-element `levels` array for live
  polling. Two connection-owned buffers keep it independent of the full
  snapshot. TCP copies the bytes before these buffers are released.

The board uses **192.168.7.1/24**. DHCP offers the connected host an address
starting at **192.168.7.16**, without advertising an internet gateway or DNS
server. Open <http://192.168.7.1/>. Connect only one board using this subnet.
The host needs a CDC-NCM driver; Windows 11, Linux and Apple hosts are the
intended targets. The initial minimal firmware was confirmed reachable on Windows;
the new GPIO hardware controls and Apple hosts still require device validation.

## Hardware

- USB D− to **PA11**, USB D+ to **PA12**, with correct USB power, grounding,
  decoupling and connector wiring for the actual board.
- USB-C device connectors need the appropriate CC resistors. Do not connect
  USB VBUS directly to the MCU's 3.3 V supply.
- The application uses internal **HSI16** for the CPU and **HSI48**, trimmed
  by CRS from USB SOF, for USB. No external crystal is assumed.
- No LED, button, UART or board-specific peripheral is assumed.
- SWD on PA13/PA14 remains available. This firmware has no Pico BOOTSEL/UF2 flow.

This is an initial port: successful compilation and host tests do not establish
that the USB electrical interface, enumeration, oscillator synchronization or
iPhone connection works on a particular board.

## Build

Requirements: PowerShell 7, Git, Docker Desktop with Linux containers, and
network access for initial dependency/toolchain downloads.

```powershell
./Build-Project.ps1
./Test-Native.ps1
# Explicitly refresh/check dependency pins:
./Build-Project.ps1 -FetchDependencies
```

Dependencies are checked out at exact revisions under ignored `deps/`:

| Dependency | Revision |
| --- | --- |
| TinyUSB | `72649b6234968813ca3e53420c7ac4526b6a6eb9` |
| lwIP 2.2.1 | `77dcd25a72509eb83f72b033d219b1d40cd8eb95` |
| STM32G4 CMSIS device | `626ee412334a5ed2e5b320af5a8d77d69f03a558` |
| CMSIS 5.9.0 | `2b7495b8535bdcb306dac29b9ded4cfb679d7e5c` |

The build reuses the repository's existing Docker **compiler image**. It does
not compile or link Pico SDK code. CMake can also run with a local ARM GCC,
newlib, Ninja and Python installation:

```sh
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=MinSizeRel
cmake --build build
python3 tools/report_memory.py build/stm32g431_usb_webserver.elf
```

Outputs under `build/`: `.elf`, `.bin`, `.hex`, `.map`, per-function `.su` stack
reports and `memory-usage.json`. Missing dependencies are fetched automatically.

## Flash from PowerShell

Install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
on Windows, including its USB DFU driver. The default method uses the chip's
factory USB DFU bootloader and requires no ST-LINK. Keep the native USB data
cable connected (PA11/PA12). By default the script sends an empty POST to
`http://192.168.7.1/api/system/dfu`, requires HTTP 202, waits up to 30 seconds for
DFU enumeration through CubeProgrammer, then writes and verifies the firmware.
It validates the firmware file and programmer before requesting the transition.

**One-time installation:** older firmware has no DFU API. Hold BOOT0 high while
resetting, release it, then flash this version using `-SkipEnterDfu`. The boot
option bytes must permit manual system-memory boot. Afterwards BOOT0 can stay
LOW for API-triggered DFU updates. No option-byte writes are performed.

```powershell
# From this project directory, build then program and verify via USB DFU:
./Build-Project.ps1
./Flash-Project.ps1
# Or combine both:
./Flash-Project.ps1 -Build
# One-time installation or manual recovery, already in DFU:
./Flash-Project.ps1 -SkipEnterDfu
# Different device address or enumeration timeout:
./Flash-Project.ps1 -BaseUri http://192.168.7.1 -DfuTimeoutSeconds 45
# Preview without writing to the board:
./Flash-Project.ps1 -WhatIf
# Select another DFU device if necessary:
./Flash-Project.ps1 -UsbPort USB2
# Optional ST-LINK/SWD method:
./Flash-Project.ps1 -Mode SWD -SerialNumber 'YOUR_STLINK_SERIAL' -UnderReset
# Custom programmer location:
./Flash-Project.ps1 -ProgrammerPath 'C:/Tools/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe'
```

The flash script finds CubeProgrammer on PATH or in its standard Program Files
installation. It defaults to the generated HEX and **USB DFU on USB1**; override
with `-Firmware` or `-UsbPort`. ELF and HEX contain their load addresses;
BIN files are written at **0x08000000**. After DFU verification, **return BOOT0
low and reset or power-cycle the board** to start the application. The script
does not use the SWD-only reset command in DFU mode. It stops on
programmer errors, and does not request mass erase or option-byte changes.
For optional SWD mode, connect ST-LINK SWDIO to PA13, SWCLK to PA14, GND and
the target voltage reference to the board's 3.3 V rail. Power the board and
connect NRST when using `-UnderReset`. SWD defaults to 1000 kHz
(`-FrequencyKHz` overrides it) and resets automatically after verification.
CLI arguments follow [ST's CubeProgrammer documentation](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_Command_Lines.html).

### DFU API behavior

`POST /api/system/dfu` accepts an **empty body only** and returns HTTP 202:

```json
{"status":"rebooting","mode":"dfu","delay_ms":1500}
```

GET does not change the boot mode. After 1500 ms the application disconnects USB,
waits 250 ms, records a one-shot marker in an 8-byte NOLOAD SRAM section and
requests a software reset. Startup's wrapped `SystemInit` checks the marker and
software-reset cause before data/BSS or peripherals are initialized. It consumes
the marker, remaps system memory, switches vectors/stack and enters ROM at
`0x1FFF0000` ([AN2606, STM32G431xx/441xx](https://www.st.com/resource/en/application_note/an2606-introduction-to-system-memory-boot-mode-on-stm32-mcus-stmicroelectronics.pdf)).
Repeated requests do not extend the delay. A subsequent reset boots the normal
application with BOOT0 LOW. API failure, malformed acceptance, enumeration
timeout or an unexpected existing DFU device stops the script before any write.
Use `-SkipEnterDfu` for an intentionally selected device already in DFU.
`-WhatIf` performs neither API calls nor USB/programmer operations.

The ROM handoff and physical USB re-enumeration still need board validation;
host tests cover API dispatch, timing and flash-script orchestration.

## Memory layout

The MCU's RAM is split: **22 KiB SRAM** at `0x20000000` and **10 KiB CCM** at
`0x10000000`. The linker puts data/BSS, USB buffers and lwIP pools in SRAM,
and reserves a **4 KiB CPU stack** plus a **6000-byte GPIO snapshot buffer** in
CCM. There is no libc heap; lwIP owns a static 4 KiB heap. USB uses its dedicated
packet memory. One snapshot is served at a time; concurrent snapshots get 503.
Snapshot ownership lasts until file close and TCP copies its data before reuse.

The initial configuration uses four 1536-byte receive buffers, one 2048-byte
NCM buffer per direction, two TCP PCBs, a 536-byte TCP MSS and 1072-byte
send/receive windows. This favors a small, low-concurrency page over throughput.
The linker rejects SRAM/flash overflow. `memory-usage.json` reports flash by
load address, including the initialized data image and alignment gaps.

Allocated RAM includes the entire static lwIP heap and pools, even when empty.
The stack reservation is a budget, **not a measured runtime high-water mark**.
Remaining CCM cannot be treated as additional contiguous regular SRAM.

Measured GPIO UI build (ARM GCC 13.2.1, `-Os`, 2026-10-31):

| Allocation | Used | Capacity |
| --- | ---: | ---: |
| Flash, including initialized data and alignment | 94,320 bytes (71.96%) | 131,072 bytes |
| Regular SRAM, including lwIP heap/pools and DFU marker | 18,500 bytes (82.12%) | 22,528 bytes |
| CCM snapshot + stack reservation | 10,096 bytes (98.59%) | 10,240 bytes |
| Total RAM allocated/reserved | 28,596 bytes (87.27%) | 32,768 bytes |

This leaves 4,028 bytes of regular SRAM and 144 bytes of CCM unallocated.
The ARM firmware build and sanitized native packet test passed; hardware USB
validation and runtime stack measurements remain outstanding.

## Validation

`Test-Native.ps1` compiles the real application loop, lwIP, DHCP and HTTP with
ASan/UBSan. A fake USB transport injects Ethernet frames and verifies DHCP
offer/ACK (without gateway/DNS), ARP, TCP handshake, the HTTP page and immediate
USB transmit backpressure. It does not emulate the USB device controller or NCM
control transfers. It also exercises GPIO register writes, reserved-pin rejection,
timer conflicts, the maximum PWM snapshot, snapshot ownership and POST validation.
PC13 tests verify the input bit mapping, pull-down startup and independent
pressed/released live-level buffers. Browser tests verify live transitions and
preservation of edited output settings while polling.
DFU tests cover GET/nonempty-body rejection, connection-owned response URIs,
HTTP 202, the 1500 ms delay, timer wraparound and idempotent scheduling.

```powershell
# Browser tests with a fake API, no board outputs changed:
npm ci
npm test
# After flashing: verify UI assets and GPIO API, without driving pins:
./Test-Device.ps1
# Offline flash-script tests with fake API/programmer; no board changes:
./tests/flash-script.test.ps1
```

Browser tests use Edge on Windows (Chromium on other systems) and check mobile
layout, drafts, Apply, reserved pins, PWM eligibility, errors and the info dialog.
Tests assert every header position, SWD ordering and PC10/PC11 selection. Full
board, mobile and desktop screenshots are written to `build/ui-board.png`,
`build/ui-mobile.png` and `build/ui-desktop.png`.

Hardware acceptance: flash via SWD, confirm the USB-NCM adapter enumerates,
confirm DHCP assignment, load the page repeatedly, unplug/replug and retest.
Repeat with each intended host, including iPhone if required. A host can use a
manual address such as `192.168.7.2/24` to isolate DHCP failures.

## Sources and licensing

STM32 startup and register definitions are provided by ST's CMSIS device
package; TinyUSB and lwIP retain their upstream licenses in `deps/`.
`src/dhcpserver/` is adapted from `../shared/rp2040-usb-web/dhcpserver/`, with
its original MIT copyright notices retained. Its Pico timer is replaced by
lwIP's `sys_now()` and serial lease logging is omitted.
Frontend code and styling derive from `../rp2040-usb-webserver` under the
retained MIT license in `LICENSE`. The existing shared web asset generator is
reused without modifying the RP2040 projects.
USB VID/PID `cafe:4310` is for development; products need an owned VID/PID.
