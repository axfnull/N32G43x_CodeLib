/*****************************************************************************
 * Copyright (c) 2022, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file hid_mouse.c
 * @author Nations
 * @version V1.2.1
 *
 * @copyright Copyright (c) 2022, Nations Technologies Inc. All rights reserved.
 */

/* include ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_core.h"
#include "usbd_hid.h"

#define HID_STATE_IDLE 0
#define HID_STATE_BUSY 1

/*!< endpoint address */
#define HID_INT_EP          0x81
#define HID_INT_EP_SIZE     8
#define HID_INT_EP_INTERVAL 10

#define USBD_VID           0x19F5
#define USBD_PID           0x5760
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

/*!< hid state ! Data can be sent only when state is idle  */
uint8_t hid_state = HID_STATE_IDLE;

/*!< config descriptor size */
#define USB_HID_CONFIG_DESC_SIZ 34
/*!< report descriptor size */
#define HID_MOUSE_REPORT_DESC_SIZE 74

/*!< global descriptor */
const uint8_t hid_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0002, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_HID_CONFIG_DESC_SIZ, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),

    /************** Descriptor of Joystick Mouse interface ****************/
    /* 09 */
    0x09,                          /* bLength: Interface Descriptor size */
    USB_DESCRIPTOR_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
    0x00,                          /* bInterfaceNumber: Number of Interface */
    0x00,                          /* bAlternateSetting: Alternate setting */
    0x01,                          /* bNumEndpoints */
    0x03,                          /* bInterfaceClass: HID */
    0x01,                          /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x02,                          /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,                             /* iInterface: Index of string descriptor */
    /******************** Descriptor of Joystick Mouse HID ********************/
    /* 18 */
    0x09,                    /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE_HID, /* bDescriptorType: HID */
    0x11,                    /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,                       /* bCountryCode: Hardware target country */
    0x01,                       /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,                       /* bDescriptorType */
    HID_MOUSE_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
    0x00,
    /******************** Descriptor of Mouse endpoint ********************/
    /* 27 */
    0x07,                         /* bLength: Endpoint Descriptor size */
    USB_DESCRIPTOR_TYPE_ENDPOINT, /* bDescriptorType: */
    HID_INT_EP,                   /* bEndpointAddress: Endpoint Address (IN) */
    0x03,                         /* bmAttributes: Interrupt endpoint */
    HID_INT_EP_SIZE,              /* wMaxPacketSize: 4 Byte max */
    0x00,
    HID_INT_EP_INTERVAL, /* bInterval: Polling Interval */
    /* 34 */
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x10,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'N', 0x00,                  /* wcChar0 */
    'A', 0x00,                  /* wcChar1 */
    'T', 0x00,                  /* wcChar2 */
    'I', 0x00,                  /* wcChar3 */
    'O', 0x00,                  /* wcChar4 */
    'N', 0x00,                  /* wcChar5 */
    'S', 0x00,                  /* wcChar6 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x26,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'N', 0x00,                  /* wcChar0 */
    '3', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    'G', 0x00,                  /* wcChar3 */
    '4', 0x00,                  /* wcChar4 */
    '3', 0x00,                  /* wcChar5 */
    'x', 0x00,                  /* wcChar6 */
    ' ', 0x00,                  /* wcChar7 */
    ' ', 0x00,                  /* wcChar8 */
    'H', 0x00,                  /* wcChar9 */
    'I', 0x00,                  /* wcChar10 */
    'D', 0x00,                  /* wcChar11 */
    ' ', 0x00,                  /* wcChar12 */
    'M', 0x00,                  /* wcChar13 */
    'O', 0x00,                  /* wcChar14 */
    'U', 0x00,                  /* wcChar15 */
    'S', 0x00,                  /* wcChar16 */
    'E', 0x00,                  /* wcChar17 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x10,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'N', 0x00,                  /* wcChar0 */
    '3', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    'G', 0x00,                  /* wcChar3 */
    '4', 0x00,                  /* wcChar4 */
    '3', 0x00,                  /* wcChar5 */
    'X', 0x00,                  /* wcChar6 */

    0x00
};

/*!< hid mouse report descriptor */
static const uint8_t hid_mouse_report_desc[HID_MOUSE_REPORT_DESC_SIZE] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x02, // USAGE (Mouse)
    0xA1, 0x01, // COLLECTION (Application)
    0x09, 0x01, //   USAGE (Pointer)

    0xA1, 0x00, //   COLLECTION (Physical)
    0x05, 0x09, //     USAGE_PAGE (Button)
    0x19, 0x01, //     USAGE_MINIMUM (Button 1)
    0x29, 0x03, //     USAGE_MAXIMUM (Button 3)

    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x25, 0x01, //     LOGICAL_MAXIMUM (1)
    0x95, 0x03, //     REPORT_COUNT (3)
    0x75, 0x01, //     REPORT_SIZE (1)

    0x81, 0x02, //     INPUT (Data,Var,Abs)
    0x95, 0x01, //     REPORT_COUNT (1)
    0x75, 0x05, //     REPORT_SIZE (5)
    0x81, 0x01, //     INPUT (Cnst,Var,Abs)

    0x05, 0x01, //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30, //     USAGE (X)
    0x09, 0x31, //     USAGE (Y)
    0x09, 0x38,

    0x15, 0x81, //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F, //     LOGICAL_MAXIMUM (127)
    0x75, 0x08, //     REPORT_SIZE (8)
    0x95, 0x03, //     REPORT_COUNT (2)

    0x81, 0x06, //     INPUT (Data,Var,Rel)
    0xC0, 0x09,
    0x3c, 0x05,
    0xff, 0x09,

    0x01, 0x15,
    0x00, 0x25,
    0x01, 0x75,
    0x01, 0x95,

    0x02, 0xb1,
    0x22, 0x75,
    0x06, 0x95,
    0x01, 0xb1,

    0x01, 0xc0 //   END_COLLECTION
};

/*!< mouse report struct */
struct hid_mouse {
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheel;
};

/*!< class */
static usbd_class_t hid_class;

/*!< interface */
static usbd_interface_t hid_intf;

/*!< mouse report */
static struct hid_mouse mouse_cfg;

/* function ------------------------------------------------------------------*/
static void usbd_hid_int_callback(uint8_t ep)
{
    /*!< endpoint call back */
    /*!< transfer successfully */
    if (hid_state == HID_STATE_BUSY) {
        /*!< update the state  */
        hid_state = HID_STATE_IDLE;
    }
}

/*!< endpoint call back */
static usbd_endpoint_t hid_in_ep = {
    .ep_cb = usbd_hid_int_callback,
    .ep_addr = 0x81
};

/* function ------------------------------------------------------------------*/
/**
  * @brief            hid mouse init
  * @pre              none
  * @param[in]        none
  * @retval           none
  */
void hid_mouse_init(void)
{
    usbd_desc_register(hid_descriptor);
    /*!< add interface */
    usbd_hid_add_interface(&hid_class, &hid_intf);
    /*!< interface add endpoint */
    usbd_interface_add_endpoint(&hid_intf, &hid_in_ep);
    /*!< register report descriptor */
    usbd_hid_report_descriptor_register(0, hid_mouse_report_desc, HID_MOUSE_REPORT_DESC_SIZE);
    /*!< init mouse report data */
    mouse_cfg.buttons = 0;
    mouse_cfg.wheel = 0;
    mouse_cfg.x = 0;
    mouse_cfg.y = 0;
    
    usb_dc_init();
}

/**
  * @brief            device send report to host
  * @pre              none
  * @param[in]        ep endpoint address
  * @param[in]        data Points to the data buffer waiting to be sent
  * @param[in]        len Length of data to be sent
  * @retval           none
  */
void hid_mouse_send_report(uint8_t ep, uint8_t *data, uint8_t len)
{
    if (usb_device_is_configured()) {
        if (hid_state == HID_STATE_IDLE) {
            /*!< updata the state */
            hid_state = HID_STATE_BUSY;
            /*!< write buffer */
            usbd_ep_write(ep, data, len, NULL);
        }
    }
}

/**
  * @brief            hid mouse test
  * @pre              none
  * @param[in]        none
  * @retval           none
  */
void hid_mouse_test(void)
{
    /*!< remove mouse pointer */
    mouse_cfg.x += 10;
    mouse_cfg.y = 0;
    /*!< send repotr to host */
    hid_mouse_send_report(HID_INT_EP, (uint8_t *)&mouse_cfg, 4);
}
