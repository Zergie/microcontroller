#ifndef PIXELS_H
#define PIXELS_H
#include <stdbool.h>
#include "animations/effects.h"
void pixels_init(void);
// Poll each main-loop iteration, even without a queued frame, to timestamp the
// actual PIO end and keep the original inter-frame delay out of the next tick.
void pixels_poll(void);
// WS2812-compatible 300 us reset for manual pads; animation timing stays original.
void pixels_set_pad_mode(bool enabled);
// Copies one complete frame and starts DMA; false means the previous frame is
// still being transmitted/reset. Never waits for FIFO space or completion.
bool pixels_submit(const rgb_t *frame);
#endif
