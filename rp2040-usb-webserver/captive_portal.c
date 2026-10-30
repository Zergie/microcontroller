#include "captive_portal.h"
#include <string.h>

const char *captive_portal_redirect(const char *path) {
    // Keep probe handling separate from the GPIO API. Missing assets and unknown
    // API paths must remain errors, rather than masquerading as HTML/redirects.
    static const char *probes[] = {
        "/hotspot-detect.html", "/library/test/success.html", // Apple
        "/generate_204", "/gen_204",                       // Android/Chrome
        "/connecttest.txt", "/ncsi.txt", "/redirect",        // Windows
        "/canonical.html", "/success.txt"                   // Firefox
    };
    size_t length = strcspn(path, "?");
    for (unsigned i = 0; i < sizeof(probes) / sizeof(probes[0]); ++i) {
        if (strlen(probes[i]) == length && strncmp(path, probes[i], length) == 0) {
            return "HTTP/1.0 302 Found\r\nLocation: " CAPTIVE_PORTAL_URL
                "\r\nCache-Control: no-store\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
        }
    }
    return NULL;
}
