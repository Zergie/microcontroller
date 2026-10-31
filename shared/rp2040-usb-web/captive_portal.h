#ifndef CAPTIVE_PORTAL_H
#define CAPTIVE_PORTAL_H

// Canonical HTTP origin for captive probes and the local GPIO controller.
#define CAPTIVE_PORTAL_URL "http://192.168.7.1/"

const char *captive_portal_redirect(const char *path);

#endif
