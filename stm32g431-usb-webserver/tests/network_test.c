/* Exercise the real firmware loop, DHCP, lwIP and HTTP using Ethernet frames.
 * Only the USB transport and hardware are replaced. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define main firmware_main
#include "../src/main.c"
#undef main
#include "lwip/apps/fs.h"
GPIO_TypeDef fake_gpio[4];
TIM_TypeDef fake_timer[3];
RCC_TypeDef fake_rcc;

const uint8_t test_uid[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
static jmp_buf done;
static unsigned phase, loops, offers, acks, arp_replies, renews;
static uint32_t server_seq;
static uint8_t frame[1600];
static const uint8_t host_mac[6] = {2,3,4,5,6,7};
static const uint8_t host_ip[4] = {192,168,7,16};
static const uint8_t device_ip[4] = {192,168,7,1};
static bool usb_busy;
static unsigned dfu_resets;
void board_reset_to_dfu(void) { ++dfu_resets; }

static uint16_t get16(const uint8_t *p) { return (p[0] << 8) | p[1]; }
static uint32_t get32(const uint8_t *p) { return ((uint32_t)get16(p) << 16) | get16(p+2); }
static void put16(uint8_t *p, unsigned n) { p[0] = n >> 8; p[1] = n; }
static void put32(uint8_t *p, uint32_t n) { put16(p, n >> 16); put16(p+2, n); }
static uint16_t checksum(const uint8_t *p, unsigned len, uint32_t sum) {
    for (; len > 1; len -= 2, p += 2) sum += get16(p);
    if (len) sum += p[0] << 8;
    while (sum >> 16) sum = (sum & 65535) + (sum >> 16);
    return (uint16_t)~sum;
}
static void ethernet(unsigned type, bool broadcast) {
    memset(frame, 0, sizeof(frame));
    if (broadcast) memset(frame, 255, 6); else memcpy(frame, usb_netif.hwaddr, 6);
    memcpy(frame+6, host_mac, 6); put16(frame+12, type);
}
static void ipv4(unsigned protocol, unsigned len, bool broadcast) {
    uint8_t *ip = frame+14;
    ip[0] = 0x45; put16(ip+2, len); ip[8] = 64; ip[9] = protocol;
    if (!broadcast) memcpy(ip+12, host_ip, 4);
    if (broadcast) memset(ip+16, 255, 4); else memcpy(ip+16, device_ip, 4);
    put16(ip+10, checksum(ip, 20, 0));
}
static void deliver(unsigned len) { assert(tud_network_recv_cb(frame, len)); }
static void send_dhcp(bool request) {
    ethernet(0x0800, true);
    uint8_t *udp = frame+34, *d = udp+8;
    put16(udp, 68); put16(udp+2, 67);
    d[0] = 1; d[1] = 1; d[2] = 6; put32(d+4, 0x12345678);
    memcpy(d+28, host_mac, 6); put32(d+236, 0x63825363);
    uint8_t *o = d+240;
    *o++ = 53; *o++ = 1; *o++ = request ? 3 : 1;
    if (request) {
        *o++ = 50; *o++ = 4; memcpy(o, host_ip, 4); o += 4;
        *o++ = 54; *o++ = 4; memcpy(o, device_ip, 4); o += 4;
    }
    *o++ = 255;
    unsigned len = o-udp;
    put16(udp+4, len); ipv4(17, 20+len, true); deliver(34+len);
}
static void send_arp(void) {
    ethernet(0x0806, true);
    uint8_t *a = frame+14;
    put16(a, 1); put16(a+2, 0x0800); a[4] = 6; a[5] = 4; put16(a+6, 1);
    memcpy(a+8, host_mac, 6); memcpy(a+14, host_ip, 4); memcpy(a+24, device_ip, 4);
    deliver(42);
}
static void send_tcp(bool syn) {
    static const char request[] = "GET / HTTP/1.0\r\nHost: 192.168.7.1\r\n\r\n";
    ethernet(0x0800, false);
    uint8_t *tcp = frame+34;
    unsigned len = 20 + (syn ? 0 : sizeof(request)-1);
    put16(tcp, 49152); put16(tcp+2, 80); put32(tcp+4, syn ? 100 : 101);
    put32(tcp+8, syn ? 0 : server_seq+1);
    tcp[12] = 0x50; tcp[13] = syn ? 2 : 0x18; put16(tcp+14, 4096);
    if (!syn) memcpy(tcp+20, request, sizeof(request)-1);
    ipv4(6, 20+len, false);
    uint32_t sum = 6 + len;
    for (unsigned i = 26; i < 34; i += 2) sum += get16(frame+i);
    put16(tcp+16, checksum(tcp, len, sum));
    deliver(34+len);
}

void board_init(void) {}
uint32_t sys_now(void) { return loops; }
void platform_panic(void) { abort(); }
bool tusb_init(uint8_t port, const tusb_rhport_init_t *init) {
    assert(port == 0 && init->role == TUSB_ROLE_DEVICE);
    tud_mount_cb(); return true;
}
bool tud_ready(void) { return true; }
bool tud_network_can_xmit(uint16_t size) { assert(size <= 1514); return !usb_busy; }
void tud_network_recv_renew(void) { ++renews; }

void tud_network_xmit(void *ref, uint16_t arg) {
    uint8_t response[1600];
    unsigned len = tud_network_xmit_cb(response, ref, arg);
    if (get16(response+12) == 0x0806) {
        if (get16(response+20) == 2) ++arp_replies;
        return;
    }
    assert(len >= 34 && get16(response+12) == 0x0800);
    uint8_t *ip = response+14;
    unsigned ihl = (ip[0] & 15) * 4;
    assert(checksum(ip, ihl, 0) == 0);
    uint8_t *transport = ip+ihl;
    if (ip[9] == 17) {
        assert(get16(transport) == 67 && get16(transport+2) == 68);
        uint8_t *d = transport+8;
        assert(d[0] == 2 && get32(d+4) == 0x12345678);
        assert(!memcmp(d+16, host_ip, 4));
        bool message_type = false;
        for (uint8_t *o = d+240; o < response+len && *o != 255;) {
            unsigned type = *o++; if (!type) continue;
            assert(o < response+len); unsigned count = *o++;
            assert(o+count <= response+len);
            assert(type != 3 && type != 6); /* No internet gateway or DNS advertised. */
            if (type == 53) {
                assert(count == 1);
                if (*o == 2) ++offers; else { assert(*o == 5); ++acks; }
                message_type = true;
            }
            o += count;
        }
        assert(message_type);
    } else if (ip[9] == 6) {
        assert(get16(transport) == 80);
        if ((transport[13] & 0x12) == 0x12) { server_seq = get32(transport+4); phase = 5; }
        unsigned header = (transport[12] >> 4) * 4;
        unsigned payload_len = get16(ip+2) - ihl - header;
        if (payload_len) {
            char payload[1601]; memcpy(payload, transport+header, payload_len); payload[payload_len] = 0;
            assert(strstr(payload, "HTTP/1.0 200 OK"));
            assert(strstr(payload, "Content-Type: text/html"));
            longjmp(done, 1);
        }
    }
}
void tud_task(void) {
    assert(++loops < 1000);
    switch (phase) {
        case 0: phase = 1; send_dhcp(false); break;
        case 1: assert(offers == 1); phase = 2; send_dhcp(true); break;
        case 2: assert(acks == 1); phase = 3; send_arp(); break;
        case 3: {
            assert(arp_replies == 1);
            struct pbuf *p = pbuf_alloc(PBUF_RAW, 64, PBUF_POOL);
            assert(p); usb_busy = true;
            assert(output(&usb_netif, p) == ERR_MEM); /* Must return immediately. */
            usb_busy = false; pbuf_free(p);
            phase = 4; send_tcp(true); break;
        }
        case 5: phase = 6; send_tcp(false); break;
        default: break;
    }
}
int main(void) {
    /* WeAct C13 key: pulled down at rest, HIGH while pressed. */
    gpio_state_t button={.pull=2};
    assert(gpio_apply(34,button)==200);
    assert(((GPIOC->PUPDR>>26)&3)==2);
    GPIOC->IDR=1U<<13;
    struct fs_file pressed={0},released={0},third={0};
    assert(fs_open_custom(&pressed,"/api/gpio/levels.json"));
    const char *pressed_levels=strstr(pressed.data,"\"levels\":["); assert(pressed_levels);
    assert(pressed_levels[10+34*2]=='1');
    GPIOC->IDR=0;
    assert(fs_open_custom(&released,"/api/gpio/levels.json"));
    assert(strstr(released.data,"\"levels\":[")[10+34*2]=='0');
    assert(pressed_levels[10+34*2]=='1'); /* Connection-owned, not overwritten. */
    assert(fs_open_custom(&third,"/api/gpio/levels.json") && strstr(third.data,"503"));
    fs_close_custom(&pressed); fs_close_custom(&released);
    assert(fs_open_custom(&third,"/api/gpio/levels.json") && strstr(third.data,"200 OK"));
    fs_close_custom(&third);
    /* GET and nonempty POST must never schedule a bootloader transition. */
    struct fs_file dfu_file={0};
    assert(fs_open_custom(&dfu_file,"/api/system/dfu") && strstr(dfu_file.data,"404"));
    char dfu_uri[64]; u8_t dfu_window=1;
    assert(httpd_post_begin((void*)7,"/api/system/dfu","",0,1,dfu_uri,sizeof(dfu_uri),&dfu_window)==ERR_ARG);
    system_control_poll(); assert(dfu_resets==0);
    assert(httpd_post_begin((void*)7,"/api/system/dfu","",0,0,dfu_uri,sizeof(dfu_uri),&dfu_window)==ERR_OK);
    /* A concurrent GPIO request must retain its own response URI. */
    assert(httpd_post_begin((void*)8,"/api/gpio/4/off","",0,0,dfu_uri,sizeof(dfu_uri),&dfu_window)==ERR_OK);
    httpd_post_finished((void*)8,dfu_uri,sizeof(dfu_uri)); assert(!strcmp(dfu_uri,"/_api/ok"));
    loops=UINT32_MAX-1000U;
    httpd_post_finished((void*)7,dfu_uri,sizeof(dfu_uri)); assert(!strcmp(dfu_uri,"/_api/dfu-accepted"));
    assert(fs_open_custom(&dfu_file,dfu_uri) && strstr(dfu_file.data,"202 Accepted") && strstr(dfu_file.data,"\"delay_ms\":1500"));
    loops+=1499U; system_control_poll(); assert(dfu_resets==0);
    system_request_dfu(); /* Retry must not postpone the deadline. */
    ++loops; system_control_poll(); assert(dfu_resets==1);
    system_control_poll(); assert(dfu_resets==1);
    loops=0;
    gpio_state_t pwm={.mode=3,.frequency=1000,.duty=50};
    assert(gpio_apply(0,pwm)==200);
    assert(TIM2->ARR==15999 && TIM2->CCR1==8000);
    pwm.frequency=2000; assert(gpio_apply(1,pwm)==409);
    assert(gpio_apply(11,pwm)==403 && gpio_apply(24,pwm)==403);
    gpio_state_t release={.mode=2,.value=1};
    assert(gpio_apply(5,release)==200);
    assert(GPIOA->OTYPER & (1U<<5));
    assert(GPIOA->BSRR==(1U<<5));
    /* Worst-case snapshot: all implemented PWM pins active. */
    unsigned pwm_pins[]={0,1,2,3,6,7,16,17,22,23,25};
    pwm.frequency=100000; pwm.duty=100;
    for(unsigned i=0;i<sizeof(pwm_pins)/sizeof(pwm_pins[0]);++i) assert(gpio_apply(pwm_pins[i],pwm)==200);
    struct fs_file a={0},b={0};
    assert(fs_open_custom(&a,"/api/gpio.json"));
    assert(strstr(a.data,"200 OK") && strstr(a.data,"\"pin\":40"));
    assert(fs_open_custom(&b,"/api/gpio.json") && strstr(b.data,"503"));
    fs_close_custom(&a);
    assert(fs_open_custom(&b,"/api/gpio.json") && strstr(b.data,"200 OK"));
    fs_close_custom(&b);
    gpio_state_t output_pin={.mode=1,.value=1};
    assert(gpio_apply(39,output_pin)==200 && GPIOC->BSRR==(1U<<10));
    assert(gpio_apply(40,output_pin)==200 && GPIOC->BSRR==(1U<<11));
    assert(gpio_apply(41,output_pin)==400);
    char uri[64]; u8_t window=1;
    assert(httpd_post_begin((void*)1,"/api/gpio/4/on/slew/fast","",0,0,uri,sizeof(uri),&window)==ERR_OK);
    assert(gpio_states[4].mode==1 && gpio_states[4].value==1 && gpio_states[4].fast);
    httpd_post_finished((void*)1,uri,sizeof(uri)); assert(!strcmp(uri,"/_api/ok"));
    assert(httpd_post_begin((void*)1,"/api/gpio/11/on","",0,0,uri,sizeof(uri),&window)!=ERR_OK);
    assert(!strcmp(uri,"/_api/403"));
    assert(httpd_post_begin((void*)1,"/api/gpio/4/off/garbage","",0,0,uri,sizeof(uri),&window)!=ERR_OK);
    assert(gpio_states[4].value==1);
    if (!setjmp(done)) firmware_main();
    assert(gpio_states[34].pull==2);
    assert(offers == 1 && acks == 1 && renews >= 4);
    puts("PASS: DFU API and delayed reset, GPIO/PWM, snapshot ownership, DHCP, ARP, TCP, HTTP, USB backpressure");
    return 0;
}
