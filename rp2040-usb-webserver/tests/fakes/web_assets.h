#ifndef TEST_WEB_ASSETS_H
#define TEST_WEB_ASSETS_H
#include <stddef.h>
static const unsigned char html[]="HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nContent-Length: 4\r\n\r\npico";
static const unsigned char js[]="HTTP/1.0 200 OK\r\nContent-Type: text/javascript\r\n\r\n// test";
static const unsigned char css[]="HTTP/1.0 200 OK\r\nContent-Type: text/css\r\n\r\nbody{}";
static const struct { const char *name; const unsigned char *data; size_t length; } web_assets[]={
 {"/index.html",html,sizeof(html)-1},{"/app.js",js,sizeof(js)-1},
 {"/gpio-model.js",js,sizeof(js)-1},{"/pin-store.js",js,sizeof(js)-1},
 {"/pico-pins.js",js,sizeof(js)-1},{"/pin-session.js",js,sizeof(js)-1},
 {"/gpio-transport.js",js,sizeof(js)-1},{"/style.css",css,sizeof(css)-1}
};
#endif
