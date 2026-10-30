# HTTP API

Direct base URL: `http://192.168.7.1` or `http://rp2040.local`.
Development base URL: `http://127.0.0.1:5173`.
All state-changing commands use POST with an empty body and Content-Length: 0.
GET does not change outputs. Responses are not cached.

| Method | Path | Effect / response |
| --- | --- | --- |
| GET | `/api/gpio.json` | Status of all GPIOs 0–29 |
| POST | `/api/gpio/{pin}/on` | Digital output HIGH; stop existing PWM |
| POST | `/api/gpio/{pin}/off` | Digital output LOW; stop existing PWM |
| POST | `/api/gpio/{pin}/input` | Input without pulls; stop PWM |
| POST | `/api/gpio/{pin}/input-pullup` | Input with internal pull-up; stop PWM |
| POST | `/api/gpio/{pin}/input-pulldown` | Input with internal pull-down |
| POST | `/api/gpio/{pin}/input-buskeeper` | Input with bus keeper (both pull bits set) |
| POST | `/api/gpio/{pin}/analog` | ADC input, GP26–28 only; digital input buffer off |
| POST | `/api/gpio/{pin}/open-drain/low` | Open Drain: actively drive LOW |
| POST | `/api/gpio/{pin}/open-drain/release` | Open Drain: release to high impedance; never actively drive HIGH |
| POST | `/api/gpio/{pin}/pwm/{hz}/{duty}` | Set PWM; 10–100000 Hz, integer duty percentage 0–100 |
| POST | `/api/led/on` | Compatibility route for GPIO 25 HIGH |
| POST | `/api/led/off` | Compatibility route for GPIO 25 LOW |
| GET | `/api/led.json` | Legacy `{"on":true}` / `{"on":false}` status of the SIO output latch |

Successful GPIO POSTs return the complete GPIO status. LED POSTs retain the
legacy response. Use the GPIO status route for PWM and input modes; the SIO
output latch does not represent the PWM signal.

Example GPIO 25 entry, shown without other pins and pad fields for brevity:

```json
{
  "pins": [
    {
      "pin": 25,
      "available": true,
      "reason": "",
      "mode": "pwm",
      "pull": "none",
      "value": true,
      "frequency_hz": 1000,
      "actual_frequency_hz": 1000,
      "duty_percent": 50,
      "slice": 4,
      "channel": "B"
    }
  ]
}
```

The actual response includes all 30 pins in ascending order. `mode` is `input`,
`output`, `pwm`, `analog` or `open-drain`. `pull` reads the hardware configuration:
`none`, `up`, `down` or `keeper` (both pulls). Pull-up remains `mode: "input"`
with `pull: "up"`. Output, PWM and ordinary input modes disable pull resistors.
For digital outputs, `value` reads the output latch; for input and PWM it reads
the instantaneous digital input level. A single level does not describe PWM
duty cycle or measured frequency.

In analog mode, `value` is null. Additional fields are `adc_raw` (0–4095) and
`millivolts`, calculated using integer arithmetic: `(adc_raw * 3300 + 2048) / 4096`.
This assumes a 3.3 V reference, is not calibrated, and is sampled again on every
status request. GP29 remains reserved for VSYS. The nominal external Pico ADC
input range is 0–3.3 V.

Open Drain reports the instantaneous input level as `value`, plus
`output_enabled` (actively driving LOW) and `output_latch` (always false).
Release does not actively drive HIGH; HIGH requires an external pull-up.
All internal pulls stay off in this mode. Switching to an input mode stops
digital output or PWM.

`actual_frequency_hz` is calculated from the system clock, PWM divider and
period length, rounded to whole Hz. Frequency/duty fields are zero outside PWM
mode. Reserved pins report a placeholder `value` of false.

Error responses have the form `{"error":"Description"}`:

| HTTP status | Meaning |
| --- | --- |
| 400 | Invalid pin number, number format, PWM values, or non-empty POST body |
| 403 | Pico board pin 23, 24 or 29 is reserved |
| 404 | Unknown path/command; GET on command paths returns the ordinary HTML 404 response |
| 409 | PWM slice frequency differs, or the same A/B channel is already assigned to another pin |
| 503 | No free POST connection or memory for a status snapshot |
| 502 | Local proxy only: board unreachable / timeout |

Rejected commands do not change outputs. Pins are reconfigured only on command,
except for boot initialization of available pins as inputs without pulls and
the LED turning on after startup. Switching to digital output/input releases
the affected PWM channel; another occupied channel on the same slice stays active.

PowerShell examples:

```powershell
Invoke-RestMethod http://127.0.0.1:5173/api/gpio.json -NoProxy
Invoke-RestMethod http://127.0.0.1:5173/api/gpio/25/on -Method Post -Body '' -NoProxy
Invoke-RestMethod http://127.0.0.1:5173/api/gpio/25/pwm/1000/50 -Method Post -Body '' -NoProxy
Invoke-RestMethod http://127.0.0.1:5173/api/gpio/25/input -Method Post -Body '' -NoProxy
```

The API targets a local USB connection and has no authentication. The development
server binds only to loopback and does not forward POSTs from foreign browser origins.

## Advanced pad settings

The following suffixes can be appended to ordinary GPIO mode commands:

| Suffix | Allowed modes | Values |
| --- | --- | --- |
| `/drive/{ma}/{slew}` | Output (`on`/`off`), PWM and Open Drain | `ma`: 2, 4, 8, 12; `slew`: slow, fast |
| `/hysteresis/{state}` | Input, Pull-up, Pull-down and Bus Keeper | `state`: on, off |

Examples:

```text
POST /api/gpio/25/on/drive/8/fast
POST /api/gpio/25/pwm/1000/50/drive/4/slow
POST /api/gpio/25/input-pullup/hysteresis/on
```

All parameters are validated before hardware changes. Invalid combinations
return HTTP 400, reserved pins 403 and PWM conflicts 409. Mode and pad settings
are applied in one command, rather than separate requests. Existing commands
without suffixes remain valid and preserve pad settings.

GPIO status also includes `drive_strength_ma`, `slew_rate` and `hysteresis`, read
directly from pad registers. Drive strengths are nominal output driver settings,
not current regulation or current limits. Hysteresis enables/disables the
digital input Schmitt trigger; it does not implement software debouncing.
The analog UI exposes none of these extra options.

## Reboot into BOOTSEL

```text
POST /api/system/bootsel
Content-Length: 0
```

Response: HTTP 202 Accepted, JSON:

```json
{"status":"rebooting","mode":"bootsel","delay_ms":1500}
```

The reboot is scheduled about 1.5 seconds later while USB and lwIP continue to
be serviced. The RP2040 then calls `reset_usb_boot(0,0)` to enter the ROM USB
bootloader with mass storage and PICOBOOT. Windows should show an RPI-RP2 drive.
The webserver/GPIO API are unavailable in this mode; firmware runs again after
flashing or a normal restart. Repeated accepted requests do not extend the
original reboot deadline.

GET on this URL returns 404 without rebooting. Non-empty POST bodies return 400.
The website has no BOOTSEL button.

```powershell
Invoke-RestMethod http://192.168.7.1/api/system/bootsel -Method Post -Body '' -NoProxy
```

The same route is available through the local proxy at 127.0.0.1:5173.
