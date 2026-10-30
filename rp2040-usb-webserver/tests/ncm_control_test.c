// Test the real TinyUSB control callback extracted from the Docker toolchain.
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

enum { CONTROL_STAGE_SETUP, CONTROL_STAGE_DATA, CONTROL_STAGE_ACK };
enum { TUSB_REQ_TYPE_STANDARD, TUSB_REQ_TYPE_CLASS };
enum { TUSB_DIR_OUT, TUSB_DIR_IN };
enum { TUSB_REQ_GET_INTERFACE=10, TUSB_REQ_SET_INTERFACE=11,
    NCM_SET_ETHERNET_PACKET_FILTER=0x43, NCM_GET_NTB_PARAMETERS=0x80,
    NCM_GET_NTB_INPUT_SIZE=0x85, NCM_SET_NTB_INPUT_SIZE=0x86 };
#define CFG_TUD_NCM_IN_NTB_MAX_SIZE 3020
#define CFG_TUD_NCM_OUT_NTB_MAX_SIZE 3020
#define CFG_TUD_NCM_IN_MAX_DATAGRAMS_PER_NTB 8
#define NCM_NETWORK_CAPS_NTB_INPUT_SIZE (1u << 5)
#define TU_VERIFY(condition,result) do { if (!(condition)) return (result); } while(0)
#define tu_memclr(pointer,size) memset(pointer,0,size)
#define tu_min16(a,b) ((a)<(b)?(a):(b))
#define TU_LOG_DRV(...) ((void)0)
#define XMIT_ALIGN_OFFSET(size) ((4-((size)&3))&3)
typedef struct { struct { uint8_t type,direction; } bmRequestType_bit; uint8_t bRequest; uint16_t wIndex,wValue,wLength; } tusb_control_request_t;
typedef struct { uint32_t dwNtbInMaxSize; uint16_t wNtbInMaxDatagrams,wReserved; } ncm_ntb_input_size_t;
static struct {
    uint8_t itf_num,itf_data_alt,bm_capabilities;
    uint16_t xmit_max_ntb_size,xmit_max_datagrams;
    ncm_ntb_input_size_t ntb_input_size;
    struct test_ntb { struct { uint16_t wBlockLength; } nth; } *xmit_glue_ntb;
    void *xmit_tinyusb_ntb;
    unsigned xmit_glue_ntb_datagram_ndx;
} ncm_interface;
static const uint32_t ntb_parameters[7]={0};
static void *transfer_buffer;
static uint16_t transfer_length;
static void tud_control_xfer(uint8_t port,const tusb_control_request_t *request,void *data,uint16_t size) {
    (void)port;(void)request;transfer_buffer=data;transfer_length=size;
}
static void tud_control_status(uint8_t port,const tusb_control_request_t *request) { tud_control_xfer(port,request,NULL,0); }
static void tud_network_recv_renew_r(uint8_t port) { (void)port; }
static void notification_xmit(uint8_t port,bool force) { (void)port;(void)force; }
static void tud_network_set_packet_filter_cb(uint16_t filter) { assert(filter==15); }

#include "ncm_control_under_test.h"

int main(void) {
    ncm_interface.xmit_max_ntb_size=CFG_TUD_NCM_IN_NTB_MAX_SIZE;
    ncm_interface.xmit_max_datagrams=CFG_TUD_NCM_IN_MAX_DATAGRAMS_PER_NTB;
    ncm_interface.bm_capabilities=1|NCM_NETWORK_CAPS_NTB_INPUT_SIZE;
    tusb_control_request_t request={.bmRequestType_bit={TUSB_REQ_TYPE_CLASS,TUSB_DIR_OUT},.bRequest=NCM_SET_ETHERNET_PACKET_FILTER,.wValue=15};
    if(!netd_control_xfer_cb(0,CONTROL_STAGE_SETUP,&request)) {
        fputs("FAIL: NCM SET_ETHERNET_PACKET_FILTER stalls (iOS negotiation)\n",stderr);return 1;
    }
    assert(transfer_length==0);
    request=(tusb_control_request_t){.bmRequestType_bit={TUSB_REQ_TYPE_CLASS,TUSB_DIR_IN},.bRequest=NCM_GET_NTB_INPUT_SIZE,.wLength=4};
    assert(netd_control_xfer_cb(0,CONTROL_STAGE_SETUP,&request) && transfer_length==4);
    assert(*(uint32_t *)transfer_buffer==CFG_TUD_NCM_IN_NTB_MAX_SIZE);
    request.bRequest=NCM_SET_NTB_INPUT_SIZE;request.bmRequestType_bit.direction=TUSB_DIR_OUT;
    assert(netd_control_xfer_cb(0,CONTROL_STAGE_SETUP,&request) && transfer_length==4);
    *(uint32_t *)transfer_buffer=2048;
    assert(netd_control_xfer_cb(0,CONTROL_STAGE_DATA,&request));
    assert(ncm_interface.xmit_max_ntb_size==2048);
    struct test_ntb ntb={.nth={1600}};
    ncm_interface.xmit_glue_ntb=&ntb;
    assert(!xmit_requested_datagram_fits_into_current_ntb(500));
    assert(xmit_requested_datagram_fits_into_current_ntb(100));
    request.wLength=8;
    assert(netd_control_xfer_cb(0,CONTROL_STAGE_SETUP,&request));
    *(ncm_ntb_input_size_t *)transfer_buffer=(ncm_ntb_input_size_t){4096,2,0};
    assert(netd_control_xfer_cb(0,CONTROL_STAGE_DATA,&request));
    assert(ncm_interface.xmit_max_ntb_size==CFG_TUD_NCM_IN_NTB_MAX_SIZE && ncm_interface.xmit_max_datagrams==2);
    ncm_interface.xmit_glue_ntb_datagram_ndx=2;
    assert(!xmit_requested_datagram_fits_into_current_ntb(100));
    request.wLength=3;
    assert(!netd_control_xfer_cb(0,CONTROL_STAGE_SETUP,&request));
    request.wLength=4;
    assert(netd_control_xfer_cb(0,CONTROL_STAGE_SETUP,&request));
    *(uint32_t *)transfer_buffer=1024;
    assert(!netd_control_xfer_cb(0,CONTROL_STAGE_DATA,&request));
    request.bRequest=NCM_GET_NTB_INPUT_SIZE;request.bmRequestType_bit.direction=TUSB_DIR_IN;request.wLength=8;
    assert(netd_control_xfer_cb(0,CONTROL_STAGE_SETUP,&request) && transfer_length==8);
    assert(((ncm_ntb_input_size_t *)transfer_buffer)->wNtbInMaxDatagrams==2);
    request.wIndex=5;
    assert(!netd_control_xfer_cb(0,CONTROL_STAGE_SETUP,&request));
    puts("TinyUSB NCM: packet filter, GET/SET input size, data stages, host limits and malformed requests OK");
}
