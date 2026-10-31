#include "pixels/pixels.h"
#include "pixel_sdk.h"
#include "board_config.h"
#include <stdio.h>
#include <string.h>

pio_hw_t fake_pio;
pixel_sdk_t fake_pixels;
static uint64_t now;
uint64_t time_us_64(void) { return now; }

int main(void) {
    pixels_init();
    assert(fake_pixels.enabled && fake_pixels.pio.pin == PIXEL_PIN);
    assert(fake_pixels.pio.shift == 24 && fake_pixels.pio.join == PIO_FIFO_JOIN_TX);
    assert(fake_pixels.pio.divider == 15.625f);
    assert(fake_pixels.dma.size == DMA_SIZE_32 && fake_pixels.dma.dreq == 7);
    assert(fake_pixels.dma.read_increment && !fake_pixels.dma.write_increment);
    rgb_t frame[PIXEL_COUNT];
    for (unsigned i = 0; i < PIXEL_COUNT; ++i) frame[i] = (rgb_t){i + 1, i + 2, i + 3};
    now = 1000;
    assert(pixels_submit(frame) && fake_pixels.starts == 1 && fake_pixels.count == PIXEL_COUNT);
    uint32_t transmitted[PIXEL_COUNT];
    memcpy(transmitted, fake_pixels.source, sizeof(transmitted));
    for (unsigned i = 0; i < PIXEL_COUNT; ++i)
        assert(transmitted[i] == ((i + 2) << 24 | (i + 1) << 16 | (i + 3) << 8));
    memset(frame, 0, sizeof(frame));
    now = 1300;
    assert(!pixels_submit(frame)); // DMA still owns the packed frame.
    assert(!memcmp(transmitted, fake_pixels.source, sizeof(transmitted)));
    fake_pixels.busy = false;
    pixels_poll();
    const uint32_t stalled=1u<<PIO_FDEBUG_TXSTALL_LSB;
    assert(fake_pio.fdebug==stalled); // Driver writes the W1C bit.
    fake_pio.fdebug=0; // Model the peripheral clearing that bit on write.
    now=1400;
    assert(!pixels_submit(frame)); // DMA done, but PIO has not stalled yet.
    assert(!memcmp(transmitted,fake_pixels.source,sizeof(transmitted)));
    now=1500;
    fake_pio.fdebug=stalled;
    pixels_poll(); // Exact PIO completion; raw tick 1 + 2 => 2931 us.
    now = 1870;
    assert(!pixels_submit(frame)); // Previous implementation sent too early.
    now = 2930;
    assert(!pixels_submit(frame));
    now = 2931;
    assert(pixels_submit(frame) && fake_pixels.starts == 2);
    for (unsigned i = 0; i < PIXEL_COUNT; ++i) assert(fake_pixels.source[i] == 0);

    // Polling completes a frame even if the next image is not available yet.
    now=3000; fake_pixels.busy=false; pixels_poll(); fake_pio.fdebug=0;
    now=3231; fake_pio.fdebug=stalled; pixels_poll();
    now=100000;
    assert(pixels_submit(frame) && fake_pixels.starts==3); // No extra on-demand delay.

    // CircuitPython's raw-tick conversion resets its fractional part each second.
    now=999800; fake_pixels.busy=false; pixels_poll(); fake_pio.fdebug=0;
    now=999900; fake_pio.fdebug=stalled; pixels_poll();
    now=1000976; assert(!pixels_submit(frame));
    now=1000977; assert(pixels_submit(frame) && fake_pixels.starts==4);
    pixels_set_pad_mode(true);
    now=1001100; fake_pixels.busy=false; pixels_poll(); fake_pio.fdebug=0;
    now=1001300; fake_pio.fdebug=stalled; pixels_poll();
    now=1001599; assert(!pixels_submit(frame));
    now=1001600; assert(pixels_submit(frame) && fake_pixels.starts==5);
    now=1001800; fake_pixels.busy=false; pixels_poll(); fake_pio.fdebug=0;
    now=1002000; fake_pio.fdebug=stalled; pixels_poll();
    pixels_set_pad_mode(false); // Restore original animation timing, even while idle.
    now=1002300; assert(!pixels_submit(frame));
    now=1003908; assert(pixels_submit(frame) && fake_pixels.starts==6);
    puts("Pixels: 800 kHz PIO config, paced DMA, GRB order, buffer ownership and TXSTALL, CircuitPython inter-frame timing and nonblocking polling OK");
}
