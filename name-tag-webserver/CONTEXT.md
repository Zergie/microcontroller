# Name Tag vocabulary and module ownership

- **Pad preferences**: eight colors and the manual fade-out duration. These are
  persistent settings; touches, notes and their envelopes cannot modify them.
- **Pad session**: one browser controller's pointer holds, selected view, songs,
  local audio and connection lifecycle. `web/pad-session.js` owns this state.
  `web/pads.js` is its DOM adapter and renders read-only snapshots. Pointer
  capture/hit testing stays in that adapter; mask ownership stays in the session.
- **Held mask**: the union of fingers and keyboard holds, in physical LED order.
  Multiple fingers on one pad hold it until the last finger leaves. Visual order
  is LED 8 to LED 1; audio pitches from left to right are C4 through C5.
- **Pulse**: a transient immediately released LED envelope with its own duration.
  Songs emit pulses; they never overwrite or restore the manual fade preference.
  Firmware `pads/pads.c` owns independent envelopes and validates active view,
  mask and duration before changing anything.
- **Pad transport**: `web/pad-transport.js` owns serialization, configuration
  acknowledgements, stale connection rejection and note expiry. Configuration,
  held masks (including lease renewal) and one-shot pulses are separate work.
  Confirmed configuration is not retransmitted before each note. The optional
  WebSocket mask stream bypasses HTTP work; pulses wait for configuration and
  readiness and are dropped once expired. Pending work is derived from the
  desired and acknowledged state, not maintained as a separate queue flag.

A view change is one session operation. Hidden pages, lost focus and disconnects
clear holds, pending notes and sound. Resume/reconnect cannot replay held notes.
Audio unlock remains synchronous with a user gesture. Rendering has no audio or
network side effects. Session tests supply deterministic adapters; browser tests
exercise the existing pointer gestures, accessibility and iPhone layouts.

`settings/` owns the existing fixed-layout CRC journal and flash adapter. Save
only preferences and selected board view after ten quiet seconds. Do not add
migration or record-version machinery. The PIO/DMA pixel driver and animation
pacing remain independent of browser audio and network waits.

`../shared/rp2040-usb-web` owns USB-NCM, DHCP, mDNS, HTTP response lifetime,
watchdog recovery and BOOTSEL. Shared network regressions belong there and run
from both application test runners. Changes to shared code require native and
browser tests and firmware builds for both projects. Preserve the tested iOS NCM
behavior, no-DNS/no-default-gateway policy and lwIP response-copy rules.

The animation catalogue contains 13 classic effects plus eight Animation Lab
concepts: Aurora, Tidal, Fireflies, Molten, Ripple, Heartbeat, Prism Weave and
Twin Orbit. `animations/lab_effects.c` owns their bounded ten-pixel renderer;
`lab_curves.h` holds interpolated sine/Gaussian tables. Original mode preserves
concept palettes; custom/random mode recolors the light envelopes. Append IDs;
do not renumber existing effects or introduce settings migrations. All 21 effects
participate in the existing shuffle. Mockup concepts 08/09 remain preview-only.
