#include <string.h>
#include "board.h"
#include "gpio_control.h"
#include "system_control.h"
#include "stm32g4xx.h"
#include "tusb.h"
#include "lwip/init.h"
#include "lwip/etharp.h"
#include "lwip/timeouts.h"
#include "lwip/apps/httpd.h"
#include "netif/ethernet.h"
#include "dhcpserver/dhcpserver.h"

uint8_t tud_network_mac_address[6];
static struct netif usb_netif;
static struct pbuf *received_frame;
static dhcp_server_t dhcp;

static err_t output(struct netif *netif, struct pbuf *p) {
    (void)netif;
    if (p->tot_len > CFG_TUD_NET_MTU) return ERR_BUF;
    if (!tud_ready()) return ERR_USE;
    /* Backpressure must never recurse into tud_task() from lwIP. */
    if (!tud_network_can_xmit(p->tot_len)) return ERR_MEM;
    tud_network_xmit(p, 0);
    return ERR_OK;
}

static err_t init_netif(struct netif *netif) {
    netif->name[0] = 'u'; netif->name[1] = 's';
    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
    netif->linkoutput = output;
    netif->output = etharp_output;
    return ERR_OK;
}

bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
    if (received_frame) return false;
    if (!size || size > CFG_TUD_NET_MTU) return true;
    struct pbuf *p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);
    /* Drop on exhaustion; accepting the frame lets NCM continue receiving. */
    if (!p) return true;
    if (pbuf_take(p, src, size) != ERR_OK) { pbuf_free(p); return true; }
    received_frame = p;
    return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
    (void)arg;
    struct pbuf *p = ref;
    return pbuf_copy_partial(p, dst, p->tot_len, 0);
}

static void discard_pending_frame(void) {
    if (received_frame) { pbuf_free(received_frame); received_frame = NULL; }
}
void tud_network_init_cb(void) { discard_pending_frame(); }
void tud_umount_cb(void) {
    discard_pending_frame();
    netif_set_link_down(&usb_netif);
}
void tud_mount_cb(void) { netif_set_link_up(&usb_netif); }

int main(void) {
    board_init();
    for(unsigned i=0;i<GPIO_COUNT;++i) if(gpio_available(i)) {
        /* WeAct C13 user key is active HIGH and needs an internal pull-down. */
        gpio_state_t input={.pull=i==34?2:0}; gpio_apply(i,input);
    }
    /* Fold all 96 UID bits into a stable locally administered host MAC. */
    const uint8_t *uid = (const uint8_t *)UID_BASE;
    for (unsigned i = 0; i < 6; ++i) tud_network_mac_address[i] = uid[i] ^ uid[i + 6];
    tud_network_mac_address[0] = (tud_network_mac_address[0] & 0xfcU) | 0x02U;
    lwip_init();
    usb_netif.hwaddr_len = 6;
    memcpy(usb_netif.hwaddr, tud_network_mac_address, 6);
    usb_netif.hwaddr[5] ^= 1;
    ip4_addr_t ip, mask, gateway;
    IP4_ADDR(&ip, 192, 168, 7, 1);
    IP4_ADDR(&mask, 255, 255, 255, 0);
    IP4_ADDR(&gateway, 0, 0, 0, 0);
    if (!netif_add(&usb_netif, &ip, &mask, &gateway, NULL, init_netif, ethernet_input)) platform_panic();
    netif_set_default(&usb_netif);
    netif_set_up(&usb_netif);
    dhcp_server_init(&dhcp, &ip, &mask, DHCP_ADVERTISE_NONE);
    if (!dhcp.udp) platform_panic();
    httpd_init();
    const tusb_rhport_init_t usb_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL };
    if (!tusb_init(0, &usb_init)) platform_panic();
    for (;;) {
        tud_task();
        if (received_frame) {
            struct pbuf *p = received_frame;
            received_frame = NULL;
            if (usb_netif.input(p, &usb_netif) != ERR_OK) pbuf_free(p);
        }
        /* Also renew after dropped frames, including allocation failures. */
        tud_network_recv_renew();
        sys_check_timeouts();
        system_control_poll();
    }
}
