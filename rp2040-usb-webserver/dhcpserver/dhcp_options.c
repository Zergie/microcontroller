#include "dhcp_options.h"
#include <string.h>

bool dhcp_parse_options(const uint8_t *data, size_t length, struct dhcp_request_options *result) {
    static const uint8_t cookie[] = {99, 130, 83, 99};
    struct dhcp_request_options parsed = {0};
    bool has_type = false;
    if (length < sizeof(cookie) || memcmp(data, cookie, sizeof(cookie)) != 0) return false;
    size_t offset = sizeof(cookie);
    while (offset < length) {
        uint8_t code = data[offset++];
        if (code == 0) continue; // PAD has no length byte.
        if (code == 255) {
            if (!has_type) return false;
            *result = parsed;
            return true;
        }
        if (offset == length) return false;
        size_t size = data[offset++];
        if (size > length - offset) return false;
        if (code == 53) {
            if (size != 1 || has_type) return false;
            parsed.message_type = data[offset];
            has_type = true;
        } else if (code == 50) {
            if (size != 4 || parsed.has_requested_ip) return false;
            memcpy(parsed.requested_ip, data + offset, 4);
            parsed.has_requested_ip = true;
        }
        offset += size;
    }
    return false; // END is required; never scan beyond the received payload.
}
