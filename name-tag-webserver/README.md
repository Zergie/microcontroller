# Name Tag USB Webserver

Native C firmware rebuilding the LED animations in
[`../name_tag_rp2040/default.py`](../name_tag_rp2040/default.py), with an English
mobile web controller. The original CircuitPython project remains unchanged.
MIDI and the experimental neural animation from `ai.py` are not included.

## Hardware

[`board_config.h`](board_config.h) configures **10 RGB NeoPixels on GP0**,
using GRB order at 800 kHz. On 2026-10-30 the user confirmed that all LEDs
illuminated red at 20% brightness after changing the data pin from GP24 to GP0.
UART logging is disabled so it does not take ownership of GP0.

The archived CircuitPython code used GP24, but its saved boot log identifies a
different physical board. That pin produced no visible LED output on the
currently connected board. On a standard Pico, GP24 senses VBUS. Keep the data
pin configurable and confirm it against the actual LED wiring.
The default SDK board is `pico`. `PIXEL_COUNT`, `PIXEL_PIN` and `PIXEL_RGBW` are compile-time hardware settings. RGBW output keeps white at
zero and uses RGB channels. Supply the LEDs appropriately and share ground.

## Build and development

Requirements: Docker Desktop with Linux containers, PowerShell 7; Node.js 22+
for local web development and tests. Run from this directory:

```powershell
./Build-Project.ps1
./Start-DevServer.ps1 -Background
# Open http://127.0.0.1:5174/
./Stop-DevServer.ps1
```

The shared Docker toolchain pins Pico SDK 2.3.1 and includes the tested iPhone
NCM negotiation fix. The UF2 is `build/pico/name_tag_webserver.uf2`.
The local server reloads HTML/CSS/JS edits and proxies `/api/` to `192.168.7.1`;
use `-Target http://192.168.7.1` or `-Port` to override. The GPIO project's local
server can remain on port 5173. API control requires the Name Tag firmware.

To install, first verify the wiring above, then connect the target board in
BOOTSEL mode and run `./Flash-Project.ps1`. If its current firmware already
supports the BOOTSEL endpoint, use:

```powershell
./Flash-Project.ps1 -EnterBootsel -BaseUri http://192.168.7.1
```

Only the explicitly selected firmware is copied; a project build never flashes
a board. Connect normally afterward and open <http://192.168.7.1/> or
<http://rp2040.local/>. Firmware-hosted web changes require a build and flash;
local web development does not. There is no runtime web-asset upload endpoint.

An already open page detects a board reboot through the `boot_id` in animation
status and reloads once the board is reachable again. Status is polled every
two seconds while visible and checked on returning from background or browser
page restoration. Short connection failures with the same boot identity do not
reload the editor. Web assets use `Cache-Control: no-store` so the reload fetches
the installed interface. Queued edits, held pads and sound are stopped before
navigation. This requires firmware and a page containing the restart detector;
a page opened from older firmware needs one manual reload after installation.

## Animations

The controller selects **Blink, Color Cycle, Comet, Chase, Pulse, Sparkle,
Solid, Rainbow, Sparkle Pulse, Rainbow Comet, Rainbow Chase, Rainbow Sparkle,
Custom Color Chase**, plus **Aurora, Tidal, Fireflies, Molten, Ripple, Heartbeat, Prism Weave,
Twin Orbit**, or **Random loop · all effects**.

Random loop is the boot default: it shuffles all 21 effects, plays each exactly
once per round, then reshuffles. The effect at the round boundary never repeats
immediately. The default interval is **300 seconds** per effect, as in the old
sequence. The current effect is displayed separately from the selected loop.

Brightness ranges from **1% to 100%** and defaults to **20%**. Speed **100%** uses the original timing below;
5–800% scales both frame intervals and animation time. The unified **Color** control offers **Original Color**, **Random Color** and
**Color** (a custom RGB picker). Original Color is the boot default. Random Color
chooses a saturated wheel color when an effect starts (also at each shuffle-loop
transition), or when entering this color mode. It stays fixed during that effect;
changing brightness, speed or interval does not reroll it. Native hardware entropy
seeds the PRNG at boot. Color uses a custom base color for single-color effects
and Color Cycle's first color. The picker appears inside the same control only
in custom mode.
Rainbow and Custom Color Chase retain their original multicolor palettes.
Settings are sent automatically: continuous input is coalesced for 200 ms,
committed changes send immediately. Offline controls stay disabled; queued
changes resume after reconnection. Settings are stored in flash and restored on boot.

Color edits redraw the current frame immediately without advancing the effect's
phase or random stream. Quick effect revisits draw immediately after the clear
frame, preserving progress without waiting on the previous visit's deadline.
Fractional elapsed time survives parameter changes; identical requests do not
insert extra output frames.

### Animation Lab additions

Eight selected concepts from `mockups/animation-lab.html` run natively on all
**ten LEDs**: Aurora (mint/violet curtains), Tidal (blue filling tide), Fireflies
(sparse gold/green pulses), Molten (embers), Ripple (centre-out waves), Heartbeat
(double beat), Prism Weave (counter-moving ribbons), and Twin Orbit (amber/blue
orbits). Meteor Shower (08) and Raindrops (09) are not included in firmware.

Original Color preserves the mockup palettes; Color and Random Color recolor
individual light envelopes with the selected base color. Brightness, 5–800%
speed, ten-second persistence and the no-repeat shuffle apply as before. These
IDs are appended to the catalogue, preserving existing IDs and saved choices.
Native order is LED 10 to LED 1 when read visually left-to-right.

The pure renderer runs at a nominal 33 ms cadence (about 30 FPS at 100%). A
772-byte pair of linearly interpolated lookup tables supplies sine and Gaussian
curves; no heap allocation, per-frame transcendental math or blocking output is
needed. Integer phase reduction preserves smooth movement after long uptime.
PIO/DMA output remains unchanged. Host tests compare 80 RGB frames against the
mockup (at most 2/255 channel tolerance), plus palette edits, black/custom colors,
large clocks, frame pacing, all 21 shuffle entries and settings restoration.
Actual appearance and rendering time at maximum speed still need hardware checks.

## LED Pads

Open the menu at the right of the compact **Now playing** card and select
**LED Pads**. Eight multitouch pads control the first eight LEDs; the remaining
two LEDs stay off in this view. Hold a pad to light its LED and release it to
fade out. Multiple fingers can hold different pads or the same pad.
Slide a held finger across pads to play them like piano keys. Moving into a gap
or outside the pads releases that finger's LED; sliding back onto a pad lights
it again. Each finger is tracked independently.

Pads also play local browser sine tones: from left to right **C4, D4, E4, F4,
G4, A4, B4 (German H4), C5**, while LED numbering and colors stay unchanged.
Audio follows the same held-pad mask as LEDs, including multitouch and slides.
The shared fade-out controls release; a minimum 10 ms release and 10 ms attack
avoid abrupt gain steps. Eight reusable voices are limited to a combined peak
of 0.25 full scale, independently of LED brightness. Leaving the view, going
offline, hiding/leaving the page or losing focus silences held notes and tails.
Audio is optional: failure to create or resume it does not prevent LED control.
There is no extra audio UI and no audio request to the board.

Following [Web Audio gesture guidance](https://developer.mozilla.org/en-US/docs/Web/API/Web_Audio_API/Best_practices),
the context is created/resumed synchronously during pad pointerdown or keyboard
activation; loading or navigating to the view does not start audio.
Sessions that expose `navigator.audioSession` are requested as `playback` during
the same gesture so iPhone Safari can play through the speaker in silent mode.
Unsupported or rejected session settings leave ordinary Web Audio available.
The operating system still chooses the output route and media volume. On older
iOS versions without this API, turn silent mode off to hear the built-in speaker.
See the [WebKit silent-mode report](https://bugs.webkit.org/show_bug.cgi?id=237322).
Browser
tests check integration and real OfflineAudioContext-rendered samples. Actual
iPhone Safari still needs checks for first-touch audio permission, interruption
and background recovery, speaker/Bluetooth latency, volume and audible clicks
during rapid multitouch slides, including zero fade.

Choose a color below each pad. Pads are displayed in reverse LED order, from 8 to 1.
In portrait, the **Colors** menu above the pads applies **Original**, **Rainbow**,
**Sunset**, **Ocean** or **Pastel** to all eight LEDs. Individual edits show
**Custom**. Palette colors use the existing save mechanism; the menu is hidden
in landscape to preserve the full pad area.
**Fade out** sets a shared release duration from 0 to 2 seconds;
changes affect subsequent releases. **Brightness** ranges from 1% to 100% and is
shared with the animation view. In landscape, the Now playing card is hidden,
the pads fill the available height and the menu stays accessible beside the
sliders. Both views adapt to iPhone portrait and landscape layouts.

The selected view, eight colors and fade duration are saved after ten seconds
without configuration changes. Held pads and fade progress are never restored.
The browser refreshes held pads every 250 ms; after 1200 ms without a refresh,
the board releases them and starts the configured fade. Leaving the view resumes
animations. With older firmware the pad layout can be previewed, but its
controls remain disabled until firmware is updated.

Current firmware provides a persistent WebSocket on port 81 for pad masks and transient song pulses.
The frontend warms the connection on entering the view and sends each changed
mask without waiting for an HTTP round trip; colors and fade use a separate
HTTP settings queue. The local server proxies the WebSocket as well as HTTP.
Boards without this capability keep the original HTTP pad control. Socket
failure releases held LEDs, silences browser audio and locks controls until
recovery. No held notes are replayed when reconnecting.

Manual pads retain PIO/DMA output with a **300 us reset interval after actual
PIO completion**. Pending unsent pad frames are replaced by the latest state;
animation frames keep their original ordered output and CircuitPython delay.
300 us exceeds the WS2812B-V5's 280 us minimum reset
([manufacturer datasheet](https://www.world-semi.co.kr/_files/ugd/89cd03_1023b0e9d135431aa1e6491bfc318112.pdf)).
The exact LED variant on this board has not been identified; verify physical
output and end-to-end touch latency after installing the new firmware. Host
tests verify timing boundaries and transport behavior, not actual iPhone latency.

## Songs

Choose **Songs** from the menu for a view with the shared **Colors** palette,
eight color pickers, shared **Brightness**, a song selector and a large
**Play / Stop** button. There are no manual pads in this view. Colors and
brightness apply immediately and use the existing board settings. Portrait uses
a stacked layout; landscape puts song selection and playback beside the colors.

The browser plays single-voice C-major arrangements of **Alle meine Entchen**
(112 BPM), **Bruder Jakob** (104 BPM), **Hänschen klein** (108 BPM) and
**Ode an die Freude** (112 BPM). Quarter-note beats, eighth/dotted notes, held
notes and explicit rests are stored in one file per song under `web/songs/`.
`web/songs.js` contains the catalog and shared player; `web/songs/score.js` parses
the common notation. The lower G3 needed in
Bruder Jakob and Ode an die Freude uses the G LED with a lower browser oscillator
pitch; manual LED Pads keep their C4–C5 mapping.

**Mahna Mahna · MIDI** plays the entire user-provided `mahnamahna.mid`: one
melody track, 17 notes and about 4.5 seconds at approximately 180 BPM. The file's
exact tempo is 333333 microseconds per quarter note, with 384 ticks per quarter.
Its note-on/off timing and explicit gaps are retained without adding articulation
gaps. E5, F5 and G5 play at their actual octave while sharing the E/F/G LEDs.
This replaces the previous short publisher-preview excerpt. The source MIDI is
kept as `tests/fixtures/mahnamahna.mid` for an independent timing comparison;
it is not served or decoded by the RP2040.

Playback is anchored to a monotonic start time, including articulation gaps;
late callbacks skip expired events rather than accumulating tempo drift or
playing a burst of stale notes. LEDs use the existing WebSocket mask stream
when supported; the older HTTP fallback remains available but its timing also
depends on HTTP latency. Audio is local and starts only from the Play gesture.
Each LED starts at full brightness and immediately releases into a fade lasting
the note's complete beat-derived duration (rounded to the API's integer
milliseconds). Half notes therefore fade for twice as long as quarter notes.
The audio holds the note with a separate short 20 ms release and articulation
gap. Each note uses a transient pulse command with its own duration. Manual pad
fade and colors remain the persistent preferences throughout playback, including
long songs, interruptions and power loss. No browser-side restoration is needed.
The pulse commands require this firmware build; install matching firmware before
using Songs from the updated development frontend.

Stop, song changes, view changes, loss of connection/focus and hiding or leaving
the page cancel future notes and silence audio; already triggered LED fades
finish on the board. Returning or reconnecting never
starts a song automatically. The selected song and playback position are local
to the page and are not persisted on the RP2040. No additional firmware API is
required.

## Persistent settings

The board restores the selected effect or random loop, brightness, speed, loop
interval, color mode and custom RGB color after reboot. Animation
progress, the random shuffle and the sampled runtime color are rebuilt; they are not written every frame.

The latest configuration is saved **ten seconds after the last change**. Every
additional change restarts the full ten-second timer. Continuous editing never
forces an intermediate save. **Saving…** is reserved for an actual write, not
the ten-second waiting period. Writes are synchronous and briefly pause HTTP,
so polling may not observe this short state at all. Wait ten seconds after the
last change before removing power; `settings_saved:true` confirms persistence.
The absence of Saving is not a persistence confirmation. Removing power earlier
may restore the previous saved settings.
A storage error is shown without disabling live animation control, and writes
are retried after five seconds. Unchanged settings do not create new records.

The final 8192 bytes of the SDK-configured flash hold a two-sector, append-only
journal (16 records per sector). Records use a fixed layout with a signature, stable effect IDs,
a generation counter and CRC32. There is no format version or migration. Sector rotation keeps the previous valid record
until the replacement has been programmed and verified. A partial/corrupt or
invalid record is ignored; if none remain, boot defaults are used. A linker
assertion keeps firmware out of the reserved region, which is absent from the
UF2 payload. Normal UF2 updates retain it; a full flash erase removes settings.

Flash erase/program runs from the main loop through the Pico SDK's
`flash_safe_execute`, not from a USB/HTTP callback. This single-core firmware
marks core 1 unused; the callback and programming buffer are in RAM. USB polling
pauses briefly for the flash operation while PIO and DMA retain their RAM buffers.
`settings/` owns the record format, batching, recovery and flash adapter.

On 2026-10-30 persistence was also verified on the connected GP0 board: after
saving, a BOOTSEL reboot and reinstall of the same UF2 restored every setting
without sending another configuration command. No watchdog recovery occurred.

## CircuitPython port

The renderer ports the bundled **Adafruit LED Animation 2.11.1** classes in
`../name_tag_rp2040/lib/adafruit_led_animation`, using the constructor values in
`default.py`. The archived Python files remain unchanged and are also the
executable test reference. The MIT attribution is in
[animations/LICENSE.txt](animations/LICENSE.txt).

| Effect | Frame interval at 100% | Original parameters |
| --- | --- | --- |
| Blink | 500 ms | Jade `(0,255,40)`, alternating with black |
| Color Cycle | 399 ms | Magenta `(255,0,20)`, Orange `(255,40,0)` |
| Comet | 10 ms | Purple `(180,0,255)`, tail 10, bounce |
| Chase | 99 ms | White, size 3, spacing 6 |
| Pulse | 99 ms | Amber `(255,100,0)`, linear triangular period 3 s |
| Sparkle | 99 ms | Purple, 10 random selections with replacement |
| Solid | 1000 ms | Jade |
| Rainbow | 99 ms | Period 2 s, wheel step 1, adjacent colors per LED |
| Sparkle Pulse | 99 ms | Jade, period 3 s, one accumulating sparkle |
| Rainbow Comet | 99 ms | Tail 7, bounce, default wheel step `256//7` |
| Rainbow Chase | 99 ms | Size 3, spacing 2, wheel step 8 |
| Rainbow Sparkle | 99 ms | Default period 5 s, 15 selections, background brightness 0.2 |
| Custom Color Chase | 99 ms | Size 2, spacing 3, Orange / White / Jade |

These intervals include CircuitPython 9.2.4's RP2040 numeric behavior:
binary32 float results lose their bottom two mantissa bits when boxed as
`MICROPY_OBJ_REPR_C`. Thus `int(speed * 1000)` turns 0.01, 0.1 and 0.4 into
9, 99 and 399 ms. Comet deliberately uses its nominal **10 ms** interval rather
than 9 ms, reducing its native step rate from about 111 to 100 steps/s at 100%
speed. This is a pacing adjustment, not a hardware-calibrated reproduction of
the Python loop's execution overhead. Other effects retain their existing timing.
`animations/circuitpython_math.h` applies the same rounding
to gradients, pulse intensity and rainbow position. Global brightness uses
PixelBuf's integer scale `floor(percent * 256 / 100) / 256`; for example Jade
at 20% produces `(0,50,7)`. Sources:
[numeric configuration](https://github.com/adafruit/circuitpython/blob/9.2.4/py/circuitpy_mpconfig.h),
[float boxing](https://github.com/adafruit/circuitpython/blob/9.2.4/py/obj.h),
[PixelBuf](https://github.com/adafruit/circuitpython/blob/9.2.4/shared-module/adafruit_pixelbuf/PixelBuf.c).

The port keeps persistent pixel/cycle state, tail clipping and reversal,
color truncation, chase palette advancement, and the intermediate `show()` calls
in Sparkle, Sparkle Pulse and Rainbow Sparkle. Two-frame bursts are drained in order through
PIO/DMA, including the transient bright frame before Sparkle's quarter-color
frame. Sequence changes clear the pixels while retaining each effect's progress,
matching `auto_clear=True, auto_reset=False`. Editing settings does not restart
or reshuffle the running effect. Each due update draws once; late calls do not
run a burst of catch-up frames, matching `Animation.animate`.

There is no fixed 16 ms cap: Comet runs at its 10 ms base update interval.
PIO serializes the bits; paced DMA transfers fixed buffers without blocking USB.
No heap allocation per frame or second core is required.

The serializer polls DMA completion and then clears/checks PIO TXSTALL before
starting the inter-frame timer. DMA completion alone does not mean the last LED
bit has left the shift register. The following frame waits for two raw ticks,
using CircuitPython 9.2.4's RP2040 conversion (1024 ticks/s), rather than the
previous 300 us approximation. Completion polling continues even between
animation updates, so this pause is not added again when a later frame arrives.
Sources: [NeoPixel output](https://github.com/adafruit/circuitpython/blob/9.2.4/ports/raspberrypi/common-hal/neopixel_write/__init__.c)
and [raw tick conversion](https://github.com/adafruit/circuitpython/blob/9.2.4/ports/raspberrypi/supervisor/port.c).

Sparkle intentionally retains a quarter-color background: the original selects
10 pixels with replacement on a 10-pixel strip, briefly shows the bright image,
then sets those pixels to `color // 4`. All ten background pixels consequently
become lit after a few updates. This behavior is in the Python source, not a C
port error. Frame values alone do not establish visual fidelity: the delay
between bright and dim images also matters. The timing correction is host-tested
against DMA/PIO completion and clock boundaries; its visible result still needs
a comparison on hardware after flashing.
Changing Sparkle's color also converts existing quarter/dim background pixels,
as its original color setter does, instead of leaving old-colored pixels behind.


The random loop intentionally remains the requested shuffle of all 21 effects,
rather than the archived fixed eight-effect sequence. Random pixel selection has
the same with-replacement behavior, but uses a native PRNG; an actual run does
not reproduce CircuitPython's random seed/sequence. Automated tests exercise the native renderer directly; the archived Python
classes are no longer executed as a differential oracle. Visual fidelity still
requires comparison on hardware.

## Tests

```powershell
./Test-Native.ps1
npm ci
npm test
```

The browser suites run up to four independent cases concurrently per suite.
Each case owns its browser context, device mock and cleanup; audio module unit
tests remain serial because they replace process globals.

Native programs are cached under `build/native-test-cache`. Each run still
executes all assertions and sanitizers. Source/preprocessed-header changes,
compiler flags or a different Docker image invalidate the corresponding binary.
Generated web assets participate in the cache key. Use `./Test-Native.ps1 -NoCache`
to compile everything without reading or updating the cache.

Native tests exercise the real renderer, shuffle sequencing, interval boundaries,
brightness, atomic command rejection and every JSON capacity under ASan/UBSan.
The production-engine tests cover immediate recoloring and phase/RNG preservation
for every effect, rapid revisits, fractional clocks, no-op requests and all 100
brightness percentages. Persistence tests cover reboot restoration, coalescing/no-op writes, sector
rotation, CRC failures, 512 torn writes and retry recovery. They also verify PIO/DMA configuration, GRB packing, transmit-buffer ownership,
PIO TXSTALL completion and CircuitPython inter-frame timing (including the
second boundary), actual embedded assets and the Name Tag adapter through shared
HTTP, including simultaneous response ownership and BOOTSEL capacity limits.
Browser tests cover portrait and landscape iPhone layouts, simultaneous touches,
independent releases, pad colors, fade settings, view restoration and queued requests,
selection, random loop, automatic updates, edits during transfers,
offline recovery, all three color modes and rejected requests. Windows uses installed Microsoft Edge; elsewhere
install Chromium with `npx playwright install chromium`.

Both firmware projects have been built and host-tested. The GP0 firmware was
flashed through the BOOTSEL endpoint and the user confirmed the solid-red LED
test on physical name-tag hardware. Other effects have automated renderer tests;
they have not all been visually checked on this hardware.

## Ownership and API

`animations/` owns effects and sequencing; `pixels/` owns PIO/DMA hardware;
`api/` owns command parsing and snapshots; `web/` owns the controller. `web/pads.js` is the DOM adapter;
`web/pad-session.js` owns holds, local audio, songs and lifecycle;
`web/pad-transport.js` orders configuration, held masks and transient pulses.
It avoids redundant configuration requests and keeps WebSocket masks independent
of HTTP work.
Rendering reads session snapshots and never sends commands or starts sound.
See [CONTEXT.md](CONTEXT.md) for ownership and invariants. Shared
transport and tools live in [`../shared/rp2040-usb-web`](../shared/rp2040-usb-web).
See [API.md](API.md).

The PIO serializer derives from Raspberry Pi's
[WS2812 example](https://github.com/raspberrypi/pico-examples/tree/master/pio/ws2812)
under [BSD-3-Clause](pixels/LICENSE.txt). Shared networking retains its
[MIT license](../shared/rp2040-usb-web/LICENSE) and source copyright notices.

Random Color requires the updated firmware. When a development server is used
with older firmware, the option is shown disabled; original/custom colors still
work. Random Color stores the mode, not the sampled color, so new animation
starts do not generate flash writes. Saved configurations use the current record format.

Versioned records from earlier firmware and out-of-range settings are rejected
without conversion. The latest valid record is restored; if none exists, boot
defaults are used. There are no settings migrations.
