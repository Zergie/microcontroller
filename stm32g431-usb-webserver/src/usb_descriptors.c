#include <string.h>
#include "stm32g4xx.h"
#include "tusb.h"
#include "class/net/net_device.h"

enum { STR_LANGUAGE, STR_MANUFACTURER, STR_PRODUCT, STR_SERIAL, STR_INTERFACE, STR_MAC };
/* TinyUSB example VID/PID for development only; assign an owned VID/PID for products. */
static const tusb_desc_device_t device = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = 64, .idVendor = 0xcafe, .idProduct = 0x4310,
    .bcdDevice = 0x0100, .iManufacturer = STR_MANUFACTURER,
    .iProduct = STR_PRODUCT, .iSerialNumber = STR_SERIAL, .bNumConfigurations = 1
};
static const uint8_t configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_CDC_NCM_DESC_LEN, 0, 100),
    TUD_CDC_NCM_DESCRIPTOR(0, STR_INTERFACE, STR_MAC, 0x81, 64, 0x02, 0x82,
        64, CFG_TUD_NET_MTU, 50, NCM_NETWORK_CAPS_ETH_FILTER | NCM_NETWORK_CAPS_NTB_INPUT_SIZE)
};
const uint8_t *tud_descriptor_device_cb(void) { return (const uint8_t *)&device; }
const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    return index == 0 ? configuration : NULL;
}
const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t language) {
    (void)language;
    static uint16_t descriptor[33];
    static const char hex[] = "0123456789ABCDEF";
    const char *text;
    unsigned len = 0;
    if (index == STR_LANGUAGE) { descriptor[1] = 0x0409; len = 1; }
    else if (index == STR_MAC || index == STR_SERIAL) {
        const uint8_t *bytes = index == STR_MAC ? tud_network_mac_address : (const uint8_t *)UID_BASE;
        unsigned count = index == STR_MAC ? 6 : 12;
        for (unsigned i = 0; i < count; ++i) {
            descriptor[++len] = hex[bytes[i] >> 4];
            descriptor[++len] = hex[bytes[i] & 15];
        }
    } else {
        switch (index) {
            case STR_MANUFACTURER: text = "Microcontroller projects"; break;
            case STR_PRODUCT: text = "STM32G431 USB webserver"; break;
            case STR_INTERFACE: text = "USB-NCM"; break;
            default: return NULL;
        }
        len = strlen(text);
        if (len > 32) len = 32;
        for (unsigned i = 0; i < len; ++i) descriptor[i + 1] = (uint8_t)text[i];
    }
    descriptor[0] = (TUSB_DESC_STRING << 8) | (2 * len + 2);
    return descriptor;
}
