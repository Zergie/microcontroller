# Project vocabulary and module ownership

This project targets the standard Raspberry Pi Pico (RP2040), not a generic
board family. Preserve its English UI, Graphite colors and PCB artwork.

- **Pico pin**: a physical header position (1–40), a GPIO or a passive power,
  ground or system pin. GP25 is the internal onboard LED, without a header
  position. `web/pico-pins.js` owns frontend pin identity, header order,
  peripheral descriptions, firmware-support labels and ADC eligibility.
- **GPIO snapshot**: the last accepted device status for all 30 GPIOs, in
  numerical order. It describes applied hardware state, not an editor preview.
- **Draft**: only fields explicitly edited for a GPIO. Refresh merges a new
  snapshot with these fields. Returning a field to its applied value removes
  that draft field. Drafts do not alter board markers or hardware.
- **Pin session**: one device's snapshots, drafts and request lifecycle.
  `web/pin-session.js` owns busy exclusion, response acceptance, connection/error
  state and submitted-pin acknowledgement. `PinStore` is its internal draft
  implementation; callers use the session interface.
- **Apply**: send the chosen GPIO configuration and acknowledge only the
  submitted pin after accepting a valid complete snapshot. Rejection or a failed
  follow-up status request preserves drafts. Selecting another pin during an
  outstanding operation does not change which pin is acknowledged.
- **Refresh**: read hardware state without changing outputs or acknowledging
  drafts. Only one Apply/Refresh operation runs at a time.

`web/app.js` is the DOM adapter: selection, browser input validity, rendering,
layout and accessibility. `web/gpio-transport.js` is the HTTP adapter with the
existing timeout and error mapping. A deterministic request adapter tests the
session interface without a browser. No general event bus or board-profile
framework is required.

Firmware remains authoritative for available GPIOs and electrical validation.
Frontend Pico descriptions do not replace firmware checks. Keep the working
USB-NCM iOS fixes, watchdog recovery, no-DNS/no-default-gateway policy and
lwIP dynamic-response lifetime/copy rules intact.

The application adapter `web_routes.c` connects GPIO commands and snapshots to
`../shared/rp2040-usb-web`. That shared module owns USB-NCM, DHCP, mDNS, HTTP
connections and dynamic response lifetime, watchdog recovery and BOOTSEL.
`main.c` owns GPIO initialization and polls the runtime. Changes to shared code
must be tested and built against both this application and `name-tag-webserver`.
