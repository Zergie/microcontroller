#include "pads_api.h"
#include "pads/pads.h"
#include "settings/settings.h"
#include <stdio.h>
#include <string.h>

static bool number(const char **text,unsigned *value) {
    const char *p=*text; unsigned n=0;
    if (*p<'0' || *p>'9') return false;
    while (*p>='0' && *p<='9') {
        if (n>10000) return false;
        n=n*10+(unsigned)(*p++-'0');
    }
    if (*p++!='/') return false;
    *value=n; *text=p; return true;
}
static int hex(char c) {
    if (c>='0' && c<='9') return c-'0';
    if (c>='a' && c<='f') return c-'a'+10;
    if (c>='A' && c<='F') return c-'A'+10;
    return -1;
}
int pads_api_command(const char *uri,uint64_t now) {
    if (!strcmp(uri,"/api/view/animations")) { pads_set_active(false,now); return 200; }
    if (!strcmp(uri,"/api/pads/release")) { pads_release(now); return 200; }
    const char pulse_prefix[]="/api/pads/pulse/";
    if (!strncmp(uri,pulse_prefix,sizeof(pulse_prefix)-1)) {
        const char *p=uri+sizeof(pulse_prefix)-1;
        unsigned mask,duration;
        // Parse the final number with the same bounded parser, without accepting suffixes.
        char command[32];
        if (strlen(p)+2>sizeof(command)) return 400;
        snprintf(command,sizeof(command),"%s/",p); p=command;
        if (!number(&p,&mask) || !number(&p,&duration) || *p) return 400;
        return pads_pulse(mask,duration,now) ? 200 : 400;
    }
    const char prefix[]="/api/pads/set/";
    const char config_prefix[]="/api/pads/config/";
    bool config_only=!strncmp(uri,config_prefix,sizeof(config_prefix)-1);
    if (!config_only && strncmp(uri,prefix,sizeof(prefix)-1)) return 404;
    const char *p=uri+(config_only ? sizeof(config_prefix)-1 : sizeof(prefix)-1);
    unsigned mask=0,fade;
    if ((!config_only && !number(&p,&mask)) || !number(&p,&fade) || strlen(p)!=PAD_COUNT*6) return 400;
    pads_config_t candidate={.fade_ms=fade};
    for (unsigned i=0;i<PAD_COUNT;++i) {
        uint8_t *channels[]={&candidate.colors[i].r,&candidate.colors[i].g,&candidate.colors[i].b};
        for (unsigned c=0;c<3;++c) {
            int high=hex(p[i*6+c*2]),low=hex(p[i*6+c*2+1]);
            if (high<0 || low<0) return 400;
            *channels[c]=(uint8_t)(high*16+low);
        }
    }
    if (!config_only) return pads_apply(mask,&candidate,now) ? 200 : 400;
    if (!pads_configure(&candidate)) return 400;
    pads_set_active(true,now);
    return 200;
}
bool pads_api_snapshot(char *buffer,size_t capacity,uint64_t now) {
    pads_config_t config=pads_config();
    int n=snprintf(buffer,capacity,"{\"active\":%s,\"pressed_mask\":%u,\"fade_ms\":%u,\"lease_ms\":%u,"
        "\"settings_saved\":%s,\"settings_error\":%s,\"colors\":[",
        pads_active() ? "true":"false",pads_pressed(now),config.fade_ms,PAD_LEASE_MS,
        settings_saved() ? "true":"false",settings_error() ? "true":"false");
    if (n<0 || (size_t)n>=capacity) return false;
    size_t used=(size_t)n;
    for (unsigned i=0;i<PAD_COUNT;++i) {
        rgb_t c=config.colors[i];
        n=snprintf(buffer+used,capacity-used,"%s\"#%02x%02x%02x\"",i ? ",":"",c.r,c.g,c.b);
        if (n<0 || (size_t)n>=capacity-used) return false;
        used+=(size_t)n;
    }
    if (capacity-used<3) return false;
    memcpy(buffer+used,"]}",3); return true;
}
