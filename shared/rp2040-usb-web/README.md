# Shared RP2040 USB web runtime

Both firmware projects compile these sources directly through `target_usb_web`.
There is one source of truth for USB-NCM, DHCP, mDNS, HTTP connection handling,
watchdog recovery and the BOOTSEL endpoint. No Git submodule or copied firmware
tree is required. The pinned SDK and NCM backport live in
[`../../toolchains/rp2040`](../../toolchains/rp2040).

## Application interface

Initialize application hardware, call `network_runtime_init(hostname)` once,
then call `network_runtime_poll()` frequently. Never block in an application
callback or frame renderer. The runtime owns the lwIP/TinyUSB calls and watchdog;
applications own their hardware and main loop. Both use `192.168.7.1/24`; DHCP
advertises neither DNS nor an internet gateway. Connect one board at a time with
this default subnet configuration.

Each application implements the two functions in `web_http.h`:

- `app_http_open(file, uri)` handles its GET and internal response routes.
- `app_http_command(uri, response_uri)` validates and executes an empty-body POST,
  returning an HTTP status and a stable response URI on success.

The common HTTP implementation owns pending POST slots, static web assets,
connection-owned JSON buffers and their release. Applications write JSON through
`web_http_json` and return ROM responses through `web_http_text`. Shared routes
are `/api/system/network.json` and `POST /api/system/bootsel`; applications must
not replace them. Internal `/_api/` URIs select responses without executing commands.
An application may override the default 400/404 response text. The shared 503
response is used for connection or memory exhaustion.

Keep `LWIP_HTTPD_FILE_EXTENSION` and the `HTTP_IS_DATA_VOLATILE` copy policy:
lwIP may close the file before TCP acknowledges its bytes. Also preserve the
tested iOS NCM negotiation and immediate USB backpressure behavior.

## Assets and tools

`target_usb_web(target, absolute_web_directory)` discovers all web assets with
CMake `CONFIGURE_DEPENDS`. The generator embeds HTML, CSS and JavaScript with
their HTTP headers. Asset additions, deletions and content edits participate in
incremental builds. Other asset types fail explicitly rather than silently
disappearing from the firmware.

Project-local PowerShell entry points pass their project directory to shared
implementations in `tools/`. Docker mounts the repository at `/workspace` so
both application and shared sources are available. The Node development server
accepts `--root`, `--port` and `--target` and provides live reload and API proxying.
For the Name Tag pad controller it also forwards same-origin WebSocket upgrades
at `/api/pads/ws` to target port 81 for masks and transient song pulses, rewriting
Host/Origin for the device. Other
upgrade paths and foreign origins are rejected. The firmware WebSocket service
belongs to Name Tag; the shared HTTP runtime and GPIO firmware are unchanged.

Run both projects' `Test-Native.ps1` and `npm test` after shared changes, then
build both firmware projects. Shared network/HTTP fakes live in `tests/fakes`;
network regression suites live in `tests/` and are run by both projects through
`tests/run_network.py`. That runner owns DHCP packets, captive redirects, NCM
negotiation, USB backpressure and watchdog recovery, with shared SDK fakes and
no dependency on either application directory. It can also run directly in the
toolchain container. The
Name Tag suite also exercises shared HTTP with its own real application adapter.

`Test-Native.ps1` caches compiled host programs in each project's ignored
`build/native-test-cache` directory. The repository mount remains read-only;
only the cache is mounted writable. Cache keys include the Docker image ID,
compiler options, source contents and preprocessed source with transitive and
generated headers. Binaries are checksummed, published under a process lock and
copied into the current test directory before execution. Test results are never
cached. `-NoCache` bypasses the cache entirely. Linker boundary checks and asset
generation still run on every invocation.

The cache regression suite is `tests/native_runner_test.py`; run it with Python
inside the Linux toolchain container after changes to `tools/native_runner.py`.
It checks reuse, invalidation, corruption, concurrent publication and execution
of failing tests from cached programs.

USB networking derives from mattmyne/usbnet commit
`458f148778727a92f235e3b915f2cfd8c56ae93b`, TinyUSB and Pico examples.
See [LICENSE](LICENSE), source notices and the
[TinyUSB patch provenance](../../toolchains/rp2040/patches/README.md).
