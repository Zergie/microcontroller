#ifndef DHCP_OPTIONS_H
#define DHCP_OPTIONS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct dhcp_request_options {
    uint8_t message_type;
    bool has_requested_ip;
    uint8_t requested_ip[4];
};

// Input includes the four-byte DHCP cookie. Reject truncated/duplicate options.
bool dhcp_parse_options(const uint8_t *data, size_t length, struct dhcp_request_options *result);

#endif
