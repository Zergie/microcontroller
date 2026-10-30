# RP2040 USB Webserver

USB webserver for the standard Raspberry Pi Pico (RP2040, board `pico`).
USB CDC-NCM connects the board to a PC or iPhone. The board uses `192.168.7.1`;
DHCP assigns the host an address such as `192.168.7.16`. Open
<http://192.168.7.1/> or the mDNS name <http://rp2040.local/>.

## Setup

Use a USB data cable and flash the firmware once; see Building and flashing
firmware below. Then connect normally without holding BOOTSEL. The onboard LED
starts off and turns on after network/webserver initialization. Other available
pins start as inputs without pull resistors. GPIO settings do not survive a reset.

The pin mapping targets the standard Pico, not arbitrary RP2040 boards or the
Pico W CYW43 LED. UART and USB serial logging are disabled to keep GPIO 0 and 1 free.

## Using the UI

The mobile UI uses Graphite colors and light gray controls, optimized for
375 × 667 pixels (iPhone 8). The schematic Pico shows its left edge on the left,
right edge on the right, and the Micro-USB port at the top in a top view.
All 40 header pins can be selected for information. Power, ground and system
pins are information-only. GP numbers are not physical header pin numbers.

Select a GPIO or **Onboard LED**, choose a mode and values, then press **Apply**.
Only this button sends the configuration; selecting and editing do not change
hardware. The header scrolls with the board. The editor stays visible at the
bottom when screen height permits; on short screens it scrolls with the page.

| Mode | Settings and behavior |
| --- | --- |
| Input | Digital level without internal pulls; undefined without an external signal. |
| Input – Pull-up | Weakly pulls an unconnected input HIGH. |
| Input – Pull-down | Weakly pulls an unconnected input LOW. |
| Input – Bus Keeper | Weakly holds the last digital level. |
| Input – Analog | ADC on GP26–28 only; raw value and voltage using a nominal 3.3 V reference. |
| Output | Actively drives LOW (Off) or HIGH (On). |
| Output – Open Drain | Drives LOW or releases to Hi-Z; HIGH requires an external pull-up. |
| Output – PWM | Frequency in Hz and duty cycle in percent. |

Applicable pad settings appear below the mode fields: **Drive strength**
(2/4/8/12 mA) and **Slew rate** (Slow/Fast) for output, Open Drain and PWM;
**Hysteresis (Schmitt trigger)** (On/Off) for digital inputs. These fields are
hidden in analog mode. All visible settings are applied together with the mode.

**Info (i)** explains the selected pin functions, current mode selection and
visible settings. UART, SPI, I2C and PIO show alternative pin assignments and
are marked as not supported. Digital GPIO, PWM, Open Drain and ADC are supported
by the API.

**Refresh (circular arrow)** reads the hardware state again. Only fields you
edited remain as drafts, including when switching pins. Successful application
clears the acknowledged pin draft; errors retain your inputs for correction or
retry. Each status request, such as Refresh or a command, takes a new ADC reading.
The voltage measurement is not calibrated.

Board markers show the applied state: `IN`, `IN↑`, `IN↓`, `IN↔`, `OUT`, `PWM`,
`OD` or `ADC`, followed by `H`/`L`, PWM duty cycle or ADC voltage. Open Drain
shows `L` while actively driving LOW and `Z · H`/`Z · L` when released, with
the current input level. Drafts do not change board markers until applied.

Available pins are **GPIO 0–22 and 25–28**; GP25 is the onboard LED. GP23 (SMPS),
GP24 (VBUS) and GP29 (VSYS) are reserved for Pico hardware and return HTTP 403.
The API reports all 30 GPIOs. QSPI flash pins are outside this set.

PWM runs in hardware. A slice has two channels, A/B, sharing a counter and
frequency. A different frequency on an occupied slice, or assigning the same
channel twice, returns HTTP 409. GP0/GP16 and GP9/GP25 are examples of channel
aliases. Switch the previous pin to input or ordinary output to release PWM.
Clock dividers can cause a small frequency difference; status reports the
rounded actual frequency.

## Local web development

Requirements: PowerShell 7 and Node.js 22 or later. The development server needs
no npm packages or local SDK. GPIO firmware must already be running on the board.
Run commands from the project directory:

```powershell
./Start-DevServer.ps1
# Alternative port / target
./Start-DevServer.ps1 -Port 5174 -Target http://rp2040.local
# Optional background process
./Start-DevServer.ps1 -Background
./Stop-DevServer.ps1
```

<http://127.0.0.1:5173/> reads files directly from `web/`: `index.html`,
`style.css` and the JavaScript modules listed under Architecture below. Saving triggers a full
browser reload without building or flashing. Reload discards drafts; hardware
keeps its state.

The server listens only on `127.0.0.1` and forwards `/api/...` to
`http://192.168.7.1`. Relative frontend URLs avoid the need for CORS permissions.
Stop the foreground process with Ctrl+C. Background logs and process metadata
are in `build/dev-server*.log` and `build/dev-server-process.json`.
Changes to `tools/dev-server.mjs` require restarting the server.

**The board serves embedded files.** Changes in `web/` appear at `rp2040.local`
only after a firmware build and flash. There is no API for uploading HTML, CSS
or JavaScript into flash.

## Building and flashing firmware

Requirements: PowerShell 7, Docker Desktop with Linux containers, a USB data
cable, and internet access for the initial toolchain/firmware build.

```powershell
./Build-Project.ps1
# Hold BOOTSEL while connecting until RPI-RP2 appears:
./Flash-Project.ps1
# Select a specific board when multiple boards are connected:
./Flash-Project.ps1 -Drive F:\
# Running firmware with the BOOTSEL endpoint:
./Flash-Project.ps1 -EnterBootsel
# Also available through the local proxy:
./Flash-Project.ps1 -EnterBootsel -BaseUri http://127.0.0.1:5173
# Enter BOOTSEL and detect the drive without flashing:
./Enter-Bootsel.ps1
```

`POST /api/system/bootsel` with an empty body returns HTTP 202 and reboots into
the ROM bootloader after about 1.5 seconds. The endpoint must already exist in
the running firmware. Enter-Bootsel waits for a newly appeared drive with an
RP2040 board ID. Existing BOOTSEL drives do not count as a newly rebooted target.
Manual BOOTSEL remains available if the firmware is unreachable.

The toolchain contains ARM GCC, CMake, Ninja and Pico SDK 2.3.1 pinned to a
specific commit, including lwIP/TinyUSB. Image:
`rp2040-usb-webserver-toolchain:sdk-2.3.1-ncm-ios-v1`.
Rebuild it with `./Build-Project.ps1 -RebuildToolchain`. Ubuntu packages come
from repositories without a frozen package snapshot, so builds are not
bit-for-bit reproducible.

Outputs: `build/pico/rp2040_usb_webserver.uf2`, plus ELF, BIN and
`compile_commands.json`. `tools/embed_web.py` embeds HTML, CSS and all six
JavaScript modules during the build. Adding embedded web files requires
updating both the generator and CMake dependencies. The flash script checks
`INFO_UF2.TXT` and copies from Windows; Docker USB passthrough is unnecessary.

## Tests and troubleshooting

```powershell
# Offline: Node/browser/PowerShell regressions using local HTTP fakes
npm ci
npm test
# Offline: C regressions in Docker with ASan/UBSan
./Test-Native.ps1
# On a connected board
./Test-Device.ps1
./Test-Device.ps1 -TestLed
./Test-Gpio.ps1
./Test-InputModes.ps1
./Test-PadSettings.ps1
# Through the local proxy
./Test-Gpio.ps1 -BaseUri http://127.0.0.1:5173
```

Hardware tests use only GP25 as a test output. Test-InputModes tests GP26–28 as
ADC/digital inputs. Scripts restore original modes, pulls and pad settings,
including hidden settings. Cleanup failures do not prevent attempts to restore
remaining pins, and diagnostics retain an earlier test failure. Tests do not
physically measure waveforms, current or switching thresholds.

Browser tests start their own servers and simulate API responses. On Windows
they use installed Microsoft Edge. On other systems run
`npx playwright install chromium` first; `PLAYWRIGHT_CHANNEL` can select an
installed browser channel. The version is pinned in `package-lock.json`.
A missing browser or failed assertion fails the run. Offline PowerShell tests
require PowerShell 7. Native tests use real production modules and installed
TinyUSB functions with hardware fakes, covering JSON boundaries, DHCP, HTTP,
PWM conflicts, USB backpressure and watchdog recovery.

If connection fails, first check the cable and Ethernet IP address. To isolate
name resolution, explicitly try <http://192.168.7.1/>. An absent board produces
HTTP 502 at the local proxy. DHCP advertises neither a DNS server nor an internet
default gateway, preserving the host internet connection. Known local HTTP
connectivity probes redirect to the UI; automatic portal opening is not
guaranteed. There is no HTTPS redirect, CAPPORT option 114 or DNS responder.

`GET /api/system/network.json` reports local portal redirects,
`dns_advertised:false` and watchdog recovery. A blocked network loop resets
after eight seconds; portal routes are disabled in the recovery boot, while
the regular UI/API remain available. A normal boot enables them again.
DHCP supports initial assignment, cached addresses and renewal/rebinding with
`ciaddr`. Replies to configured clients go to `ciaddr`; initial assignment uses
broadcast.

The NCM packet-filter and NTB input-size fix resolved the previously empty IPv4
configuration on the affected iPhone. On 2026-10-30 the user confirmed that
iOS 26.6 received an IP and opened the website with automatic configuration.
This does not establish compatibility with every adapter or iOS version.

## Architecture and provenance

`gpio_api.c` parses HTTP paths, maps errors to HTTP and emits bounded JSON.
`gpio_control.c` validates configurations before hardware changes and supplies
readings. `web_fs.c` connects the API and static files to lwIP. Each dynamic
GPIO response owns its snapshot; the TCP copy policy preserves its lifetime.
`web/pico-pins.js` owns the standard Pico's physical pin order, GPIO identities,
pin legends, firmware-support labels and ADC eligibility. Board rendering and
mode eligibility use the same descriptions; no generic board-profile layer is involved.
`web/gpio-model.js` defines modes, fields, explanations, presentation and commands.
`web/pin-session.js` owns snapshots, drafts, busy exclusion, connection/error state,
refresh and submitted-pin acknowledgement. `web/pin-store.js` is its internal
draft implementation. `web/gpio-transport.js` supplies the HTTP adapter; tests
use a deterministic request adapter without a browser. `web/app.js` handles DOM
selection, input validity and rendering; request start/end does not repaint the
editor or board unless a new snapshot was accepted.
See [CONTEXT.md](CONTEXT.md) for domain terms and ownership. Session behavior is
tested through its public interface. Browser tests check wiring, accessibility,
layout and visible states; native tests verify actual embedded module payloads
and imports as well as the firmware behavior described above.
API reference: [API.md](API.md).

Based on [mattmyne/usbnet](https://github.com/mattmyne/usbnet), commit
`458f148778727a92f235e3b915f2cfd8c56ae93b`. The upstream USB networking example
uses TinyUSB and lwIP from the Pico SDK, with DHCP code based on Pico examples.
It was itself based on the TinyUSB `net_lwip_webserver` example, including
TinyUSB PR #2829. See [LICENSE](LICENSE) and the source-file copyright notices
for the MIT license and attribution.
Pico SDK 2.3.1: `079c6f39023649b154152db30f1d781e884879bc`.
The Windows 11 NCM divisor fix from TinyUSB PR #3914 and the iOS backport from
PR #3630 are checked during image build; see [patch provenance](patches/README.md).

Hardware and artwork references:

- [Pico datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- [PWM SDK](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#hardware_pwm)
- [Header pinout](https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf)
- [GPIO functions, section 2.19](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [Official USB vector paths](https://www.raspberrypi.com/documentation/microcontrollers/images/pico-pinout.svg),
  group `USB_...`, embedded in `web/index.html` with original fill colors and
  no runtime dependency on an external image.
