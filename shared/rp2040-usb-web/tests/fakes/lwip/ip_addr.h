#ifndef TEST_IP_ADDR_H
#define TEST_IP_ADDR_H
#include <stdbool.h>
#include <stdint.h>
typedef uint16_t u16_t;
typedef struct { uint32_t addr; } ip4_addr_t;
typedef ip4_addr_t ip_addr_t;
#define ip_addr_copy(dest,src) ((dest)=(src))
#define ip_2_ip4(ip) (ip)
#define ip4_addr_get_u32(ip) ((ip)->addr)
#define IP_ANY_TYPE NULL
#define IP4_ADDR(ip,a,b,c,d) ((ip)->addr=(uint32_t)(a)|((uint32_t)(b)<<8)|((uint32_t)(c)<<16)|((uint32_t)(d)<<24))
#endif
