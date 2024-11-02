#ifndef _USB_CONFIG_H
#define _USB_CONFIG_H

#include "funconfig.h"
#include "ch32v003fun.h"

#define FUSB_CONFIG_EPS       2 // Include EP0 in this count
#define FUSB_SUPPORTS_SLEEP   0
#define FUSB_HID_INTERFACES   1
#define FUSB_CURSED_TURBO_DMA 0 // Hacky, but seems fine, shaves 2.5us off filling 64-byte buffers.
#define FUSB_HID_USER_REPORTS 1
#define FUSB_IO_PROFILE       1
#define FUSB_USE_HPE          FUNCONF_ENABLE_HPE

#include "usb_defines.h"

//Taken from http://www.usbmadesimple.co.uk/ums_ms_desc_dev.htm
static const uint8_t device_descriptor[] = {
	18, //Length
	1,  //Type (Device)
	0x00, 0x02, //Spec
	0x0, //Device Class
	0x0, //Device Subclass
	0x0, //Device Protocol  (000 = use config descriptor)
	64, //Max packet size for EP0 (This has to be 8 because of the USB Low-Speed Standard)
	0x09, 0x12, //ID Vendor
	0x35, 0xd0, //ID Product
	0x03, 0x00, //ID Rev
	1, //Manufacturer string
	2, //Product string
	3, //Serial string
	1, //Max number of configurations
};

static const uint8_t HIDAPIRepDesc[ ] =
{
	HID_USAGE_PAGE ( 0xff ), // Vendor-defined page.
	HID_USAGE      ( 0x00 ),
	HID_REPORT_SIZE ( 8 ),
	HID_COLLECTION ( HID_COLLECTION_LOGICAL ),
		HID_REPORT_COUNT   ( 254 ),
		HID_REPORT_ID      ( 0xaa )
		HID_USAGE          ( 0x01 ),
		HID_FEATURE        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,
		HID_REPORT_COUNT   ( 63 ), // For use with `hidapitester --vidpid 1209/D003 --open --read-feature 171`
		HID_REPORT_ID      ( 0xab )
		HID_USAGE          ( 0x01 ),	
		HID_FEATURE        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,
		HID_REPORT_COUNT   ( 63 ), // For configuring the setup.
		HID_REPORT_ID      ( 0xac )
		HID_USAGE          ( 0x01 ),	
		HID_FEATURE        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,
		HID_REPORT_COUNT_N   ( 510,2 ), // For receiving IQ data on host.
		HID_REPORT_ID      ( 0xad )
		HID_USAGE          ( 0x01 ),	
		HID_FEATURE        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,
	HID_COLLECTION_END,
};

/* Configuration Descriptor Set */
static const uint8_t config_descriptor[ ] =
{
    /* Configuration Descriptor */
    0x09,                                                   // bLength
    0x02,                                                   // bDescriptorType
    0x22, 0x00,                                             // wTotalLength
    0x01,                                                   // bNumInterfaces (3)
    0x01,                                                   // bConfigurationValue
    0x00,                                                   // iConfiguration
    0xA0,                                                   // bmAttributes: Bus Powered; Remote Wakeup
    0x32,                                                   // MaxPower: 100mA

    /* Interface Descriptor (Special) */
    0x09,                                                   // bLength
    0x04,                                                   // bDescriptorType
    0x00,                                                   // bInterfaceNumber
    0x01,                                                   // bAlternateSetting
    0x01,                                                   // bNumEndpoints
    0x03,                                                   // bInterfaceClass
    0x00,                                                   // bInterfaceSubClass
    0xff,                                                   // bInterfaceProtocol: OTher
    0x00,                                                   // iInterface

    /* HID Descriptor (Special) */
    0x09,                                                   // bLength
    0x21,                                                   // bDescriptorType
    0x10, 0x01,                                             // bcdHID
    0x00,                                                   // bCountryCode
    0x01,                                                   // bNumDescriptors
    0x22,                                                   // bDescriptorType
    sizeof(HIDAPIRepDesc), 0x00,                            // wDescriptorLength

    /* Endpoint Descriptor (Special) */
    0x07,                                                   // bLength
    0x05,                                                   // bDescriptorType
    0x81,                                                   // bEndpointAddress: IN Endpoint 1
    0x03,                                                   // bmAttributes
    0x08, 0x00,                                             // wMaxPacketSize
    0xff,                                                   // bInterval: slow.
};



#define STR_MANUFACTURER u"CNLohr"
#define STR_PRODUCT      u"lolra ch32v203 goertzel test"
#define STR_SERIAL       u"CUSTOMDEVICE000"

struct usb_string_descriptor_struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wString[];
};
const static struct usb_string_descriptor_struct string0 __attribute__((section(".rodata"))) = {
	4,
	3,
	{0x0409}
};
const static struct usb_string_descriptor_struct string1 __attribute__((section(".rodata")))  = {
	sizeof(STR_MANUFACTURER),
	3,
	STR_MANUFACTURER
};
const static struct usb_string_descriptor_struct string2 __attribute__((section(".rodata")))  = {
	sizeof(STR_PRODUCT),
	3,
	STR_PRODUCT
};
const static struct usb_string_descriptor_struct string3 __attribute__((section(".rodata")))  = {
	sizeof(STR_SERIAL),
	3,
	STR_SERIAL
};

// This table defines which descriptor data is sent for each specific
// request from the host (in wValue and wIndex).
const static struct descriptor_list_struct {
	uint32_t	lIndexValue;
	const uint8_t	*addr;
	uint8_t		length;
} descriptor_list[] = {
	{0x00000100, device_descriptor, sizeof(device_descriptor)},
	{0x00000200, config_descriptor, sizeof(config_descriptor)},
	// interface number // 2200 for hid descriptors.
	{0x00002200, HIDAPIRepDesc, sizeof(HIDAPIRepDesc)},

	{0x00002100, config_descriptor + 18, 9 }, // Not sure why, this seems to be useful for Windows + Android.

	{0x00000300, (const uint8_t *)&string0, 4},
	{0x04090301, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
	{0x04090302, (const uint8_t *)&string2, sizeof(STR_PRODUCT)},	
	{0x04090303, (const uint8_t *)&string3, sizeof(STR_SERIAL)}
};
#define DESCRIPTOR_LIST_ENTRIES ((sizeof(descriptor_list))/(sizeof(struct descriptor_list_struct)) )


#endif

