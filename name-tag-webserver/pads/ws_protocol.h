#ifndef PAD_WS_PROTOCOL_H
#define PAD_WS_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// A bounded RFC 6455 endpoint for the binary mask and transient pulse commands.
typedef struct {
    uint8_t input[1024], message[5];
    size_t used, message_used;
    bool upgraded, fragmented;
    void *user;
    bool (*write)(void *user,const uint8_t *data,size_t length);
    bool (*press)(void *user,unsigned mask);
    bool (*pulse)(void *user,unsigned mask,unsigned duration_ms);
} pad_ws_protocol_t;
// False means the peer must be disconnected. No payload references survive feed.
bool pad_ws_feed(pad_ws_protocol_t *ws,const uint8_t *data,size_t length);
#endif
