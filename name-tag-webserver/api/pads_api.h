#ifndef PADS_API_H
#define PADS_API_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
int pads_api_command(const char *uri,uint64_t now_ms);
bool pads_api_snapshot(char *buffer,size_t capacity,uint64_t now_ms);
#endif
