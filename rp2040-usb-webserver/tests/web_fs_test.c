#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lwip/apps/fs.h"
#include "lwip/apps/httpd.h"
#include "gpio_api.h"
#include "network_status.h"
#include "sdk.h"

static bool portal_enabled=true;
static unsigned bootsel_requests;
static bool fail_malloc;
void *__real_malloc(size_t size);
void *__wrap_malloc(size_t size) {
    if (fail_malloc) { fail_malloc=false; return NULL; }
    return __real_malloc(size);
}
int network_status_portal_enabled(void) { return portal_enabled; }
const char *network_status_response(void) {
    return "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n{\"dns_advertised\":false}";
}
void system_request_bootsel(void) { ++bootsel_requests; }

static struct fs_file open_file(const char *uri) {
    struct fs_file file={0};
    assert(fs_open_custom(&file,uri)==1);
    assert(file.data && file.len==(int)strlen(file.data) && file.index==file.len);
    assert(file.flags==FS_FILE_FLAGS_HEADER_INCLUDED);
    return file;
}
static void close_file(struct fs_file *file) { fs_close_custom(file); assert(file->pextension==NULL); }
static err_t begin(void *connection,const char *uri,int length,char *response) {
    u8_t window=1;
    return httpd_post_begin(connection,uri,"",0,length,response,80,&window);
}
static void command(const char *uri) { bool led;assert(gpio_api_command(uri,&led)==200); }
static bool snapshot_level(const struct fs_file *file) {
    const char *pin=strstr(file->data,"\"pin\":25,");assert(pin);
    const char *value=strstr(pin,"\"value\":");assert(value);
    return strncmp(value,"\"value\":true",12)==0;
}

static void route_contracts(void) {
    struct { int status; const char *label; const char *body; } errors[]={
      {400,"Bad Request","{\"error\":\"Invalid command: pin 0-29, PWM 10-100000 Hz and 0-100 percent; body must be empty.\"}"},
      {403,"Forbidden","{\"error\":\"This pin is reserved for Pico board hardware.\"}"},
      {404,"Not Found","{\"error\":\"Unknown API command.\"}"},
      {409,"Conflict","{\"error\":\"PWM conflict: slice frequencies must match; a channel can only be assigned once.\"}"},
      {503,"Service Unavailable","{\"error\":\"No free connections or memory.\"}"}
    };
    for (unsigned i=0;i<sizeof(errors)/sizeof(errors[0]);++i) {
        char uri[32],expected[256];snprintf(uri,sizeof(uri),"/_api/%d",errors[i].status);
        snprintf(expected,sizeof(expected),"HTTP/1.0 %d %s\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n%s",errors[i].status,errors[i].label,errors[i].body);
        struct fs_file file=open_file(uri);assert(strcmp(file.data,expected)==0 && !file.pextension);close_file(&file);
    }
    const char *assets[]={"/","/index.html","/app.js","/gpio-model.js","/pin-store.js",
        "/pico-pins.js","/pin-session.js","/gpio-transport.js","/style.css"};
    const char *root=NULL;
    for (unsigned i=0;i<sizeof(assets)/sizeof(assets[0]);++i) {
        struct fs_file file=open_file(assets[i]);assert(!file.pextension);
        if(i==0) root=file.data;
        if(i==1) assert(file.data==root);
        close_file(&file);
    }
    struct fs_file missing={0};assert(!fs_open_custom(&missing,"/missing.css"));
    assert(!fs_open_custom(&missing,"/api/missing"));assert(!fs_open_custom(&missing,"/api/system/bootsel"));
    struct fs_file file=open_file("/api/system/network.json");assert(strstr(file.data,"\"dns_advertised\":false"));close_file(&file);
    file=open_file("/hotspot-detect.html");
    assert(strcmp(file.data,"HTTP/1.0 302 Found\r\nLocation: http://192.168.7.1/\r\nCache-Control: no-store\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")==0);close_file(&file);
    portal_enabled=false;assert(!fs_open_custom(&missing,"/hotspot-detect.html"));portal_enabled=true;
    file=open_file("/_api/bootsel-accepted");
    assert(strstr(file.data,"HTTP/1.0 202 Accepted") && strstr(file.data,"\"delay_ms\":1500"));
    close_file(&file);assert(bootsel_requests==0);
    for(unsigned i=0;i<2;++i) {
        command(i ? "/api/led/on" : "/api/led/off");file=open_file("/api/led.json");
        assert(strcmp(file.data,i ? "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nContent-Length: 11\r\n\r\n{\"on\":true}" : "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nContent-Length: 12\r\n\r\n{\"on\":false}")==0);close_file(&file);
    }
}

static void snapshot_ownership(void) {
    command("/api/led/on");struct fs_file on=open_file("/api/gpio.json");assert(on.pextension && snapshot_level(&on));
    command("/api/led/off");struct fs_file off=open_file("/api/gpio.json");
    assert(off.pextension && off.pextension!=on.pextension && !snapshot_level(&off) && snapshot_level(&on));
    close_file(&off);assert(snapshot_level(&on));close_file(&on);close_file(&on);
    fail_malloc=true;struct fs_file failed=open_file("/api/gpio.json");
    assert(strstr(failed.data,"HTTP/1.0 503 Service Unavailable") && !failed.pextension);close_file(&failed);
}

static void post_contracts(void) {
    int connections[10]={0};char response[80]="unchanged";
    assert(begin(&connections[0],"/api/led/on",1,response)==ERR_ARG && strcmp(response,"/_api/400")==0);
    assert(begin(&connections[0],"/api/system/bootsel",1,response)==ERR_ARG && bootsel_requests==0);
    assert(begin(&connections[0],"/api/no-such-command",0,response)==ERR_ARG && strcmp(response,"/_api/404")==0);
    assert(begin(&connections[0],"/api/gpio/23/on",0,response)==ERR_ARG && strcmp(response,"/_api/403")==0);
    assert(begin(&connections[0],"/api/gpio/25/pwm/1000/50",0,response)==ERR_OK);
    httpd_post_finished(&connections[0],response,sizeof(response));assert(strcmp(response,"/api/gpio.json")==0);
    assert(begin(&connections[0],"/api/gpio/9/pwm/1000/50",0,response)==ERR_ARG && strcmp(response,"/_api/409")==0);
    assert(begin(&connections[0],"/api/led/on",0,response)==ERR_OK);
    assert(begin(&connections[1],"/api/gpio/25/off",0,response)==ERR_OK);
    httpd_post_finished(&connections[1],response,sizeof(response));assert(strcmp(response,"/api/gpio.json")==0);
    httpd_post_finished(&connections[0],response,sizeof(response));assert(strcmp(response,"/api/led.json")==0);
    assert(begin(&connections[0],"/api/system/bootsel",0,response)==ERR_OK && bootsel_requests==1);
    httpd_post_finished(&connections[0],response,sizeof(response));assert(strcmp(response,"/_api/bootsel-accepted")==0);
    for(unsigned i=0;i<8;++i) assert(begin(&connections[i],i%2 ? "/api/gpio/25/off" : "/api/led/on",0,response)==ERR_OK);
    bool before=gpio_control_led_on();
    assert(begin(&connections[8],"/api/system/bootsel",0,response)==ERR_ARG && strcmp(response,"/_api/503")==0);
    assert(bootsel_requests==1 && gpio_control_led_on()==before);
    for(unsigned i=0;i<8;++i) {
        httpd_post_finished(&connections[i],response,sizeof(response));
        assert(strcmp(response,i%2 ? "/api/gpio.json" : "/api/led.json")==0);
    }
    strcpy(response,"unchanged");httpd_post_finished(&connections[9],response,sizeof(response));assert(strcmp(response,"unchanged")==0);
    assert(httpd_post_receive_data(&connections[0],pbuf_alloc(0,0,0))==ERR_OK);
    assert(httpd_post_receive_data(&connections[0],pbuf_alloc(0,1,0))==ERR_ARG);
}

int main(void) {
    for(unsigned i=0;i<30;++i) gpio_init(i);
    gpio_control_init();
    route_contracts();snapshot_ownership();post_contracts();
    puts("HTTP filesystem: route/header/body contracts, static/dynamic lifetime, connection-local POST, capacity and BOOTSEL boundaries OK");
}
