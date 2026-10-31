# Animation Lab

Run the mockup through the existing local API proxy:

```powershell
node shared/rp2040-usb-web/tools/dev-server.mjs --root name-tag-webserver/mockups --port 5186 --target http://192.168.7.1
```

Open `http://127.0.0.1:5186/animation-lab.html`. Gallery previews stay local.
Explore starts live output; Previous/Next change the sampled animation. Speed,
Glow and Pause affect both the browser preview and the transmitted frames.

The existing API addresses eight LEDs. Ten simulated pixels are interpolated
into eight pads in the controller's visual order (LED 8 to LED 1); LED 9 and 10
stay off. The board's current global brightness also applies. Frames use
`POST /api/pads/set/255/0/{colors}`, with one request at a time and a 70 ms
minimum delay between frames. Slow responses reduce the frame rate instead of
building a queue.

Normal closing or hiding the page stops output, restores the captured pad
colors/fade with no held fingers, and resumes animations if they were active.
No command is sent merely by loading the gallery. Connection errors stop the
stream and attempt restoration. Page exit uses a release/restoration beacon;
restoring the prior mode cannot be guaranteed if the browser terminates or USB
is unplugged. The firmware's held-pad lease prevents stuck lights. These are
ordinary configuration commands, so an unchanged preview held for ten seconds
can be persisted by existing firmware; no flash-persistence behavior is changed.

Tests: `node --test name-tag-webserver/tests/live-preview.test.mjs`.
No firmware build or flash is needed.
