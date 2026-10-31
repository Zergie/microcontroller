#ifndef NETWORK_RUNTIME_H
#define NETWORK_RUNTIME_H
#include <stdbool.h>
// One USB network per firmware. Call poll frequently from the main loop;
// application work must not block USB, lwIP timers or the watchdog.
bool network_runtime_init(const char *hostname);
void network_runtime_poll(void);
#endif
