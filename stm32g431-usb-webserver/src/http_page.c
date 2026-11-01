#include <stdio.h>
#include <string.h>
#include "lwip/apps/httpd.h"
#include "lwip/apps/fs.h"
#include "web_assets.h"
#include "gpio_control.h"
#include "system_control.h"

/* A connection owns the snapshot until close. lwIP copies its TCP payload. */
static char snapshot[6000] __attribute__((section(".ccm")));
static bool snapshot_busy;
static char levels[2][256];
static bool levels_busy[2];
static void *posts[2];
static bool post_dfu[2];
#define RESPONSE(code,body) "HTTP/1.0 " code "\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n" body
static const char ok[]=RESPONSE("200 OK","{\"ok\":true}");
static const char dfu[]=RESPONSE("202 Accepted","{\"status\":\"rebooting\",\"mode\":\"dfu\",\"delay_ms\":1500}");
static const char bad[]=RESPONSE("400 Bad Request","{\"error\":\"Invalid GPIO mode or settings.\"}");
static const char forbidden[]=RESPONSE("403 Forbidden","{\"error\":\"Pin reserved for USB, SWD, BOOT0 or oscillators.\"}");
static const char conflict[]=RESPONSE("409 Conflict","{\"error\":\"Timer frequency conflict.\"}");
static const char busy[]=RESPONSE("503 Service Unavailable","{\"error\":\"GPIO snapshot busy; retry Refresh.\"}");
static const char missing[]=RESPONSE("404 Not Found","{\"error\":\"Unknown resource.\"}");
static int serve(struct fs_file *f,const char *data,size_t len) {
    f->data=data; f->len=(int)len; f->index=f->len; f->flags=FS_FILE_FLAGS_HEADER_INCLUDED; return 1;
}
static int text(struct fs_file *f,const char *data) {return serve(f,data,strlen(data));}
static bool number(const char **cursor,unsigned *value) {
    const char *p=*cursor; unsigned n=0;
    if(*p<'0' || *p>'9') return false;
    do {unsigned d=(unsigned)(*p++-'0'); if(n>(100000U-d)/10U) return false; n=n*10+d;} while(*p>='0' && *p<='9');
    *value=n; *cursor=p; return true;
}
int fs_open_custom(struct fs_file *f,const char *uri) {
    f->pextension=NULL;
    if(!strcmp(uri,"/api/gpio/levels.json")) {
        unsigned slot; for(slot=0;slot<2 && levels_busy[slot];++slot) {}
        if(slot==2) return text(f,busy);
        size_t used=(size_t)snprintf(levels[slot],sizeof(levels[slot]),RESPONSE("200 OK","{\"levels\":["));
        for(unsigned i=0;i<GPIO_COUNT;++i) {
            if(i) levels[slot][used++]=',';
            levels[slot][used++]=gpio_level(i)?'1':'0';
        }
        memcpy(levels[slot]+used,"]}",3); used+=2;
        levels_busy[slot]=true; f->pextension=levels[slot];
        return serve(f,levels[slot],used);
    }
    if(!strcmp(uri,"/api/gpio.json")) {
        if(snapshot_busy) return text(f,busy);
        size_t used=(size_t)snprintf(snapshot,sizeof(snapshot),RESPONSE("200 OK","{\"pins\":["));
        const char *modes[]={"input","output","open-drain","pwm"}, *pulls[]={"none","up","down"};
        for(unsigned i=0;i<GPIO_COUNT;++i) {
            gpio_state_t s=gpio_states[i];
            int n=snprintf(snapshot+used,sizeof(snapshot)-used,
                "%s{\"pin\":%u,\"available\":%s,\"mode\":\"%s\",\"pull\":\"%s\",\"value\":%u,\"output_enabled\":%s,\"slew_rate\":\"%s\"",
                i?",":"",i,gpio_available(i)?"true":"false",modes[s.mode],pulls[s.pull],gpio_level(i)?1:0,
                s.mode==2 && !s.value?"true":"false",s.fast?"fast":"slow");
            if(n<0 || (size_t)n>=sizeof(snapshot)-used) return text(f,busy);
            used+=(size_t)n;
            if(s.mode==3) {
                uint32_t divider=(16000000U/s.frequency+65535U)/65536U;
                uint32_t period=16000000U/(divider*s.frequency);
                n=snprintf(snapshot+used,sizeof(snapshot)-used,",\"frequency_hz\":%lu,\"actual_frequency_hz\":%lu,\"duty_percent\":%u",
                    (unsigned long)s.frequency,(unsigned long)(16000000U/(divider*period)),s.duty);
                if(n<0 || (size_t)n>=sizeof(snapshot)-used) return text(f,busy);
                used+=(size_t)n;
            }
            if(used+4>=sizeof(snapshot)) return text(f,busy);
            snapshot[used++]='}';
        }
        memcpy(snapshot+used,"]}",3); used+=2;
        snapshot_busy=true; f->pextension=snapshot; return serve(f,snapshot,used);
    }
    if(!strcmp(uri,"/_api/ok")) return text(f,ok);
    if(!strcmp(uri,"/_api/dfu-accepted")) return text(f,dfu);
    if(!strcmp(uri,"/_api/400")) return text(f,bad);
    if(!strcmp(uri,"/_api/403")) return text(f,forbidden);
    if(!strcmp(uri,"/_api/409")) return text(f,conflict);
    if(!strcmp(uri,"/_api/503")) return text(f,busy);
    if(!strcmp(uri,"/")) uri="/index.html";
    for(size_t i=0;i<sizeof(web_assets)/sizeof(web_assets[0]);++i)
        if(!strcmp(uri,web_assets[i].name)) return serve(f,(const char*)web_assets[i].data,web_assets[i].length);
    /* Let httpd try index.html after index.shtml and generate its 404. */
    if(!strncmp(uri,"/api/",5) || !strncmp(uri,"/_api/",6)) return text(f,missing);
    return 0;
}
void fs_close_custom(struct fs_file *f) {
    if(f->pextension==snapshot) snapshot_busy=false;
    for(unsigned i=0;i<2;++i) if(f->pextension==levels[i]) levels_busy[i]=false;
    f->pextension=NULL;
}
static int command(const char *uri) {
    unsigned pin;
    if(strncmp(uri,"/api/gpio/",10)) return 400;
    const char *part=uri+10;
    if(!number(&part,&pin) || *part++!='/' || pin>=GPIO_COUNT) return 400;
    if(!gpio_available(pin)) return 403;
    gpio_state_t s={0};
    const char *commands[]={"input","input-pullup","input-pulldown","on","off","open-drain/low","open-drain/release"};
    bool found=false;
    for(unsigned i=0;i<7;++i) {
        size_t len=strlen(commands[i]);
        if(!strncmp(part,commands[i],len) && (!part[len] || part[len]=='/')) {
            s.mode=i<3?0:i<5?1:2; s.pull=i<3?i:0; s.value=i==3 || i==6;
            part+=len; found=true; break;
        }
    }
    if(!found) {
        unsigned hz,duty;
        if(strncmp(part,"pwm/",4)) return 400;
        part+=4;
        if(!number(&part,&hz) || *part++!='/' || !number(&part,&duty) || duty>100) return 400;
        s.mode=3; s.frequency=hz; s.duty=(uint8_t)duty;
    }
    if(!strcmp(part,"/slew/fast")) s.fast=1;
    else if(*part && strcmp(part,"/slew/slow")) return 400;
    return gpio_apply(pin,s);
}
err_t httpd_post_begin(void *connection,const char *uri,const char *request,u16_t len,int content_len,char *response,u16_t capacity,u8_t *window) {
    (void)request;(void)len;(void)window;
    int status=400;
    if(content_len==0) {
        unsigned slot; for(slot=0;slot<2 && posts[slot];++slot) {}
        if(slot==2) status=503;
        else {
            bool is_dfu=!strcmp(uri,"/api/system/dfu");
            status=is_dfu?202:command(uri);
            if(status==200 || status==202) {posts[slot]=connection; post_dfu[slot]=is_dfu; return ERR_OK;}
        }
    }
    snprintf(response,capacity,"/_api/%d",status); return ERR_ARG;
}
err_t httpd_post_receive_data(void *connection,struct pbuf *p) {(void)connection; pbuf_free(p); return ERR_ARG;}
void httpd_post_finished(void *connection,char *response,u16_t capacity) {
    for(unsigned i=0;i<2;++i) if(posts[i]==connection) {
        posts[i]=NULL;
        if(post_dfu[i]) {system_request_dfu(); snprintf(response,capacity,"/_api/dfu-accepted");}
        else snprintf(response,capacity,"/_api/ok");
        post_dfu[i]=false; return;
    }
}
