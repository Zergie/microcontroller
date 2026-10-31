#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "captive_portal.h"

static void http_tests(void) {
    const char *probes[]={"/hotspot-detect.html","/library/test/success.html","/generate_204","/gen_204",
        "/connecttest.txt","/ncsi.txt","/redirect","/canonical.html","/success.txt","/generate_204?test=1"};
    for(unsigned i=0;i<sizeof(probes)/sizeof(probes[0]);++i) {
        const char *response=captive_portal_redirect(probes[i]);assert(response);
        assert(strstr(response,"302 Found\r\n"));assert(strstr(response,"Location: " CAPTIVE_PORTAL_URL "\r\n"));
        assert(strstr(response,"Content-Length: 0\r\n"));assert(strstr(response,"Cache-Control: no-store\r\n"));
    }
    assert(!captive_portal_redirect("/api/gpio.json"));
    assert(!captive_portal_redirect("/api/system/bootsel"));
    assert(!captive_portal_redirect("/missing.js"));
    assert(!captive_portal_redirect("/hotspot-detect.htmlx"));
    assert(!captive_portal_redirect("/"));
    puts("Captive HTTP: Apple/Android/Windows/Firefox probes redirect; API and assets excluded OK");
}
int main(void) { http_tests(); }
