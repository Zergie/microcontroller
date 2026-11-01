#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H
#include <stdint.h>
void platform_panic(void) __attribute__((noreturn));
#define LWIP_PLATFORM_ASSERT(message) do { (void)(message); platform_panic(); } while (0)
#define LWIP_PLATFORM_DIAG(message) do { } while (0)
#endif
