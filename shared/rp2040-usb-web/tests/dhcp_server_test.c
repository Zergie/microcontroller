// Exercise the production UDP callback with complete BOOTP/DHCP packets.
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lwip/udp.h"
#include "dhcpserver/dhcpserver.h"
#include "dhcpserver/dhcp_options.h"

#include "network.h"

static void receive(dhcp_server_t *server,const uint8_t *options,size_t length,bool renewing) {
    struct pbuf *p=pbuf_alloc(0,240+length,0);uint8_t *bytes=p->payload;
    bytes[0]=1;bytes[1]=1;bytes[2]=6;bytes[4]=0x12;bytes[5]=0x34;
    const uint8_t mac[]={2,3,4,5,6,7};memcpy(bytes+28,mac,6);
    const uint8_t cookie[]={99,130,83,99};memcpy(bytes+236,cookie,4);
    memcpy(bytes+240,options,length);
    if(renewing) { const uint8_t ip[]={192,168,7,16};memcpy(bytes+12,ip,4); }
    server->udp->receive(server->udp->arg,server->udp,p,NULL,68);
}

static void expect_type(uint8_t type) {
    struct dhcp_request_options parsed;
    assert(fake_network.response_size>=244 && fake_network.response[0]==2);
    assert(dhcp_parse_options(fake_network.response+236,fake_network.response_size-236,&parsed));
    assert(parsed.message_type==type);
}

int main(void) {
    dhcp_server_t server;ip_addr_t ip,mask;
    IP4_ADDR(&ip,192,168,7,1);IP4_ADDR(&mask,255,255,255,0);
    dhcp_server_init(&server,&ip,&mask,DHCP_ADVERTISE_NONE);
    const uint8_t discover[]={53,1,1,255};
    receive(&server,discover,sizeof(discover),false);assert(fake_network.responses==1);expect_type(2);
    for(size_t i=240;i<fake_network.response_size && fake_network.response[i]!=255;) {
        uint8_t code=fake_network.response[i++];if(code==0) continue;
        size_t length=fake_network.response[i++];assert(length<=fake_network.response_size-i);
        assert(code!=3 && code!=6); // Default configuration must not hijack DNS or routing.
        i+=length;
    }
    const uint8_t select[]={53,1,3,50,4,192,168,7,16,54,4,192,168,7,1,255};
    receive(&server,select,sizeof(select),false);assert(fake_network.responses==2);expect_type(5);
    // RFC 2131 renewal/rebinding: ciaddr is set, option 50 MUST be absent.
    const uint8_t renew[]={53,1,3,255};
    receive(&server,renew,sizeof(renew),true);
    if(fake_network.responses!=3) { dhcp_server_deinit(&server);fputs("FAIL: valid DHCP renewal ignored (no ACK)\n",stderr);return 1; }
    expect_type(5);
    ip_addr_t client;IP4_ADDR(&client,192,168,7,16);
    assert(fake_network.destination.addr==client.addr);
    // A cached lease can also be verified immediately after a server restart.
    dhcp_server_deinit(&server);
    dhcp_server_init(&server,&ip,&mask,DHCP_ADVERTISE_NONE);
    const uint8_t cached[]={53,1,3,50,4,192,168,7,16,255};
    receive(&server,cached,sizeof(cached),false);assert(fake_network.responses==4);expect_type(5);
    assert(fake_network.destination.addr==0xffffffff);
    // Missing both address forms must not manufacture an ACK.
    receive(&server,renew,sizeof(renew),false);assert(fake_network.responses==4);
    dhcp_server_deinit(&server);
    dhcp_server_init(&server,&ip,&mask,DHCP_ADVERTISE_DNS);
    receive(&server,discover,sizeof(discover),false);expect_type(2);
    bool dns=false,router=false;
    for(size_t i=240;i<fake_network.response_size && fake_network.response[i]!=255;) {
        uint8_t code=fake_network.response[i++];if(code==0) continue;
        size_t length=fake_network.response[i++];assert(length<=fake_network.response_size-i);
        if(code==6) { assert(length==4 && memcmp(fake_network.response+i,&ip.addr,4)==0);dns=true; }
        if(code==3) router=true;
        i+=length;
    }
    assert(dns && !router);
    dhcp_server_deinit(&server);
    puts("DHCP packet callback: discovery, selection, renewal/rebinding, cached lease restart and reply destinations OK");
}
