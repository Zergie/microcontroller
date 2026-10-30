#ifndef TEST_HTTPD_H
#define TEST_HTTPD_H
#include "lwip/udp.h"
typedef uint8_t u8_t;
#define ERR_ARG -16
err_t httpd_post_begin(void *,const char *,const char *,u16_t,int,char *,u16_t,u8_t *);
err_t httpd_post_receive_data(void *,struct pbuf *);
void httpd_post_finished(void *,char *,u16_t);
#endif
