# Name Tag API

All routes are same-origin HTTP. JSON responses use `Cache-Control: no-store`.
There is no authentication; the USB host controls the device.

## Read status

`GET /api/animation.json` returns:

```json
{
  "boot_id": "0123456789abcdef",
  "selection": "random",
  "current": "rainbow",
  "brightness_percent": 20,
  "speed_percent": 100,
  "interval_seconds": 300,
  "color": "#00ff28",
  "original_colors": true,
  "color_mode": "original",
  "current_color": "#000000",
  "settings_saved": true,
  "settings_saving": false,
  "settings_error": false,
  "pixel_count": 10,
  "pin": 0,
  "pads_supported": true,
  "control_view": "animations",
  "default_colors": ["#00ff28", "#ff0014", "#b400ff", "#ffffff", "#ff6400", "#b400ff", "#00ff28", "#000000", "#00ff28", "#000000", "#000000", "#000000", "#000000"],
  "effects": ["blink", "color-cycle", "comet", "chase", "pulse", "sparkle", "solid", "rainbow", "sparkle-pulse", "rainbow-comet", "rainbow-chase", "rainbow-sparkle", "custom-color-chase"]
}
```

`current` varies while `selection` remains `random`. For a single effect, both
identify that effect. `default_colors` is ordered like `effects`.
`boot_id` is a random 64-bit identity encoded as 16 hexadecimal digits. It is
generated once per boot and is never persisted. The controller reloads when
this value changes, including after reflashing or connecting another board.
`color_mode` is `original`, `random` or `custom`. `original_colors` remains a
legacy compatibility flag (`true` only for original mode). `color` is the stored
custom color; `current_color` is the effective base color for the current effect.
Random mode samples once per effect start, including shuffle-loop transitions;
parameter edits and GET requests do not resample. Multicolor palettes are
always those from the original classes.
`settings_saved` reports whether the current configuration matches a verified
flash record. `settings_saving` is true only while a commit is executing, not during the
ten-second debounce. Synchronous flash operations pause HTTP handling, so a
polling client may not observe this short state.
`settings_error` reports a failed save; live control remains
available and the main loop retries. Successful POSTs acknowledge applied RAM
state; wait for `settings_saved:true` before disconnecting power.

## Apply settings

```text
POST /api/animation/set/{selection}/{brightness}/{speed}/{interval}/{color}
POST /api/animation/set/{selection}/{brightness}/{speed}/{interval}/{color}/original
POST /api/animation/set/{selection}/{brightness}/{speed}/{interval}/{color}/random
```

The body must be empty. Selection is `random` or an ID from `effects`.
Brightness is an integer 1–100 (%), speed 5–800 (%), interval 5–3600 (seconds),
color exactly six hexadecimal digits without `#`. The interval is required for
all commands but only affects random loop. The optional `/original` suffix uses
the original per-effect colors; `/random` chooses one random base color per
animation start. Without a suffix, `color` overrides the base color
(backward compatible with previous clients). Speed 100% uses the original
`default.py` timing with CircuitPython 9.2.4's RP2040 float rounding, except Comet
which uses the nominal 10 ms interval instead of 9 ms for a slightly slower pace. Brightness
uses PixelBuf's 256-step scale. Color edits redraw the current frame immediately
without advancing its phase or random stream. Changing parameters preserves
animation progress and fractional elapsed time; identical requests do not redraw.
Choosing a different effect clears the pixels and resumes that effect's saved
progress with an immediately due first draw, even on a quick revisit.
Entering random loop starts a fresh shuffled round. Every field is validated
before any configuration changes, and success returns the status snapshot.

```powershell
Invoke-RestMethod http://192.168.7.1/api/animation/set/random/20/100/300/00ff28/original -Method Post -Body '' -NoProxy
Invoke-RestMethod http://192.168.7.1/api/animation/set/solid/20/100/300/ff8000 -Method Post -Body '' -NoProxy
```

Responses: 200 applied, 400 invalid command or nonempty body, 404 unknown
command, 503 no free HTTP slot or response memory. Applying always starts from
the validated complete configuration; an invalid command leaves it unchanged.
Changes are saved ten seconds after the last configuration change, then
restored on reboot. Every additional change restarts the full ten-second timer;
there is no forced intermediate save during continuous editing. Identical
configurations do not cause another write. Animation progress and the sampled runtime color are not persisted; the color
mode and custom color are persisted. Records use one fixed layout without a version field or migrations. Versioned
records from earlier firmware and invalid values are ignored; if no valid record
remains, the board uses boot defaults.

## LED pads

`GET /api/pads.json` returns `active`, `pressed_mask`, `fade_ms`, `lease_ms`,
`settings_saved`, `settings_error` and `colors` (eight `#rrggbb` strings).
Animation status advertises `pads_supported:true` and `control_view`, either
`animations` or `pads`.
New firmware also advertises `pads_transport:"websocket-v1"`.

```text
POST /api/pads/set/{mask}/{fade_ms}/{colors}
POST /api/pads/config/{fade_ms}/{colors}
POST /api/pads/pulse/{mask}/{duration_ms}
POST /api/pads/release
POST /api/view/animations
```

Bodies must be empty. `mask` is an integer 0–255: bit 0 holds LED 1, bit 7 holds
LED 8. `fade_ms` is an integer 0–2000. `colors` contains exactly 48 hexadecimal
digits: eight consecutive RGB colors, without separators or `#`. All fields
are validated before any changes. Successful commands return pad status.

`set` enters pad mode and atomically applies the complete mask and configuration.
Newly held LEDs illuminate immediately at the configured global brightness.
Released LEDs fade independently; their duration is captured at release.
Send the current mask at least every 1200 ms while holding LEDs; the web client
uses 250 ms heartbeats. An expired lease releases all held LEDs and starts their
fades. `release` does the same immediately and stays in pad mode.
`view/animations` clears pad state and resumes animations. Extra LEDs remain off
in pad mode. Colors, fade duration and selected view use the shared ten-second
persistence debounce; touches and heartbeats do not create flash writes.

`pulse` starts an immediately released note envelope for a nonzero mask (1–255)
with an integer duration of 0–2000 ms. It requires an already active pad view;
it never changes colors, manual fade, view or persisted settings. Each selected
LED captures its own duration, so overlapping notes fade independently. Repeating
a pulse retriggers its selected LEDs. Zero duration leaves them dark. A pulse
does not renew held-pad leases. It returns 400 if validation or the active-view
precondition fails, with no partial changes.

### Fast pad transport

The current frontend uses `config` to apply colors/fade and enter pad mode,
without changing held LEDs or renewing their lease. This request is sent only
on entry or settings changes. The legacy `set` route remains available for
existing clients and older boards; capability detection selects the transport.

Open `ws://192.168.7.1:81/api/pads/ws` (or `rp2040.local:81`). The development
server proxies `/api/pads/ws` on its own port to board port 81. The endpoint
accepts one controller and checks Origin against the device Host; the proxy
checks its own origin and rewrites the upstream origin. Frames follow
[RFC 6455](https://www.rfc-editor.org/rfc/rfc6455), with masked client frames,
fragmented binary messages, ping/pong and bounded receive storage.

A held-pad message is two bytes: `[1, mask]`, where `mask` is the eight-bit
held-pad state. A transient pulse is four bytes: `[2, mask, duration_lo,
duration_hi]`, with duration in unsigned little-endian milliseconds (0–2000).
The nonzero mask and active-view precondition are the same as HTTP `pulse`.
Both messages are acknowledged as `[command, mask]` after application. Pulse
support is additive to `websocket-v1`; older firmware understands only command 1.
Use matching firmware for Songs from the updated frontend.
The browser sends subsequent changes immediately, without waiting for replies.
It sends held masks every 250 ms and idle masks every second; missing replies
for over one second stop the controller. The 1200 ms board lease still applies.
Closing or losing the socket releases held LEDs with their configured fade.
Masks never change colors, fade, brightness, saved settings or reactivate an
animation view. Close the stream before `POST /api/view/animations`.

TCP Nagle buffering is disabled for the device connection and proxy. The browser
bounds pending transport data rather than building an obsolete gesture queue.
The device closes stalled handshakes after 2 seconds and idle peers after 5 seconds.

## Shared system routes

- `GET /api/system/network.json`: USB-network watchdog recovery and portal status.
- `POST /api/system/bootsel`, empty body: 202 acknowledgement followed by a ROM
  BOOTSEL reboot after 1500 ms. USB networking disappears until firmware boots.
