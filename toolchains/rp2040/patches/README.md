# TinyUSB NCM compatibility backport

`tinyusb-ncm-ios.patch` adapts the relevant parts of TinyUSB PR
https://github.com/hathach/tinyusb/pull/3630 to Pico SDK 2.3.1's pinned
TinyUSB revision. Upstream issue: https://github.com/hathach/tinyusb/issues/3505.
The original implementation is part of TinyUSB and retains its MIT license.

The backport adds Ethernet packet-filter acknowledgement and GET/SET NTB
input-size negotiation, including DATA-stage processing and host-selected
transmit size/datagram limits. It advertises capabilities 0x21. Unlike the
new upstream descriptor macro, it preserves this SDK's nine-argument macro
signature and fixed notification interval. The older descriptor API reads
the checked subtype byte directly. Control requests validate direction and
length; GET responses use persistent interface storage.

The existing Windows NCM divisor fix (TinyUSB PR #3914) remains applied.
No DNS server or default gateway is introduced by this patch.

`rp2040-usb-webserver/tests/run_ncm.py` extracts the actual control callback from the installed
SDK for host tests under ASan/UBSan. The original callback fails the packet
filter test; the patched callback must pass packet filter, 4/8-byte input
sizes, DATA stages, negotiated limits and rejection tests. The firmware
build separately compiles the complete actual driver and descriptor macro.

Docker build context includes only Dockerfile and this patch. After changing
the patch run `./Build-Project.ps1 -RebuildToolchain` from either application
directory before native tests. Both applications use the same image.
