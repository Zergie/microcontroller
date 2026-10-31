#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H
// GP0 verified on the connected name tag with a solid-red test (2026-10-30).
// The archived CircuitPython GP24 setting does not match this board.
#define PIXEL_PIN 0
#define PIXEL_COUNT 10
#define PIXEL_RGBW 0
// WS2812B-V5 needs >280 us. Increase for a different LED variant if required.
#define PIXEL_PAD_RESET_US 300u
#endif
