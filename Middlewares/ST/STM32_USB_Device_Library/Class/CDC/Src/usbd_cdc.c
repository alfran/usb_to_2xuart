/**
******************************************************************************
* @file    usbd_cdc.c
* @author  MCD Application Team
* @version V2.4.2
* @date    11-December-2015
* @brief   This file provides the high layer firmware functions to manage the 
*          following functionalities of the USB CDC Class:
*           - Initialization and Configuration of high and low layer
*           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
*           - OUT/IN data transfer
*           - Command IN transfer (class requests management)
*           - Error management
*           
*  @verbatim
*      
*          ===================================================================      
*                                CDC Class Driver Description
*          =================================================================== 
*           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
*           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
*           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
*           This driver implements the following aspects of the specification:
*             - Device descriptor management
*             - Configuration descriptor management
*             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
*             - Requests management (as described in section 6.2 in specification)
*             - Abstract Control Model compliant
*             - Union Functional collection (using 1 IN endpoint for control)
*             - Data interface class
* 
*           These aspects may be enriched or modified for a specific user application.
*          
*            This driver doesn't implement the following aspects of the specification 
*            (but it is possible to manage these features with some modifications on this driver):
*             - Any class-specific aspect relative to communication classes should be managed by user application.
*             - All communication classes other than PSTN are not managed
*      
*  @endverbatim
*                                  
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
* @{
*/


/** @defgroup USBD_CDC 
* @brief usbd core module
* @{
*/ 

/** @defgroup USBD_CDC_Private_TypesDefinitions
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USBD_CDC_Private_Defines
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USBD_CDC_Private_Macros
* @{
*/ 

/**
* @}
*/ 


/** @defgroup USBD_CDC_Private_FunctionPrototypes
* @{
*/


static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, 
                                  uint8_t epnum);

static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_CDC_GetFSCfgDesc (uint16_t *length);

static uint8_t  *USBD_CDC_GetHSCfgDesc (uint16_t *length);

static uint8_t  *USBD_CDC_GetOtherSpeedCfgDesc (uint16_t *length);

static uint8_t  *USBD_CDC_GetOtherSpeedCfgDesc (uint16_t *length);

uint8_t  *USBD_CDC_GetDeviceQualifierDescriptor (uint16_t *length);

static uint8_t *USBD_CDC_GetUsrStrDescriptor1 (uint16_t *length);
static uint8_t *USBD_CDC_GetUsrStrDescriptor2 (uint16_t *length);

uint8_t UserRxBuffer[2][APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
uint8_t UserTxBuffer[2][APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */

uint32_t CurrentwIndx = 0xff;

//#define INTERFACE1_STRING_DESCRIPTOR "Interface 1"
//#define INTERFACE2_STRING_DESCRIPTOR "Interface 2"

extern uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
* @}
*/ 

/** @defgroup USBD_CDC_Private_Variables
* @{
*/ 


/* CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_CDC = 
{
  USBD_CDC_Init,
  USBD_CDC_DeInit,
  USBD_CDC_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_CDC_EP0_RxReady,
  USBD_CDC_DataIn,
  USBD_CDC_DataOut,
  NULL,
  NULL,
  NULL,     
  USBD_CDC_GetHSCfgDesc,  
  USBD_CDC_GetFSCfgDesc,    
  USBD_CDC_GetOtherSpeedCfgDesc, 
  USBD_CDC_GetDeviceQualifierDescriptor,
	USBD_CDC_GetUsrStrDescriptor1,
	USBD_CDC_GetUsrStrDescriptor2,
};

/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_CDC_CfgHSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */  
  USB_CDC_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x04,   /* bNumInterfaces: 4 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */
  
  /*---------------------------------------------------------------------------*/ 
  
  /* IAD - 1 */
  0x08, /* bLenght */
  0x0B, /* bDescriptorType */
  0x00, /* bFirstInterface */
  0x02, /* bInterfaceCount */
  0x02, /* bFunctionClass */
  0x02, /* bFunctionSubClass */
  0x01, /* bFunctionProtocol */
  0x01, /* iFunction */
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_CMD_INTERFACE_1,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP1,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0x10,                           /* bInterval: */ 
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  CDC_DATA_INTERFACE_1,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP1,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_IN_EP1,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
  0x00,                               /* bInterval: ignore for Bulk transfer */
  
  /* IAD - 2 */
  0x08, /* bLenght */
  0x0B, /* bDescriptorType */
  0x02, /* bFirstInterface */
  0x02, /* bInterfaceCount */
  0x02, /* bFunctionClass */
  0x02, /* bFunctionSubClass */
  0x01, /* bFunctionProtocol */
  0x01, /* iFunction */
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_CMD_INTERFACE_2,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x03,   /* bDataInterface: 3 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x02,   /* bMasterInterface: Communication class interface */
  0x03,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP2,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0x10,                           /* bInterval: */ 
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  CDC_DATA_INTERFACE_2,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP2,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_IN_EP2,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
  0x00,                               /* bInterval: ignore for Bulk transfer */
  
} ;


/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_CDC_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x04,   /* bNumInterfaces: 4 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */
  
  /*---------------------------------------------------------------------------*/
  
  /* IAD - 1 */
  0x08, /* bLenght */
  0x0B, /* bDescriptorType */
  0x00, /* bFirstInterface */
  0x02, /* bInterfaceCount */
  0x02, /* bFunctionClass */
  0x02, /* bFunctionSubClass */
  0x01, /* bFunctionProtocol */
  0x01, /* iFunction */
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_CMD_INTERFACE_1,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  USBD_IDX_INTERFACE_USR_STR1,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP1,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0x10,                           /* bInterval: */ 
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  CDC_DATA_INTERFACE_1,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  USBD_IDX_INTERFACE_USR_STR1,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP1,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_IN_EP1,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                               /* bInterval: ignore for Bulk transfer */
  
  /* IAD - 2 */
  0x08, /* bLenght */
  0x0B, /* bDescriptorType */
  0x02, /* bFirstInterface */
  0x02, /* bInterfaceCount */
  0x02, /* bFunctionClass */
  0x02, /* bFunctionSubClass */
  0x01, /* bFunctionProtocol */
  0x01, /* iFunction */
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_CMD_INTERFACE_2,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  USBD_IDX_INTERFACE_USR_STR2,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x03,   /* bDataInterface: 3 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x02,   /* bMasterInterface: Communication class interface */
  0x03,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP2,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0x10,                           /* bInterval: */ 
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  CDC_DATA_INTERFACE_2,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  USBD_IDX_INTERFACE_USR_STR2,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP2,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_IN_EP2,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
    
} ;

__ALIGN_BEGIN uint8_t USBD_CDC_OtherSpeedCfgDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{ 
  0x09,   /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,   
  USB_CDC_CONFIG_DESC_SIZ,
  0x00,
  0x04,   /* bNumInterfaces: 4 interfaces */
  0x01,   /* bConfigurationValue: */
  0x04,   /* iConfiguration: */
  0xC0,   /* bmAttributes: */
  0x32,   /* MaxPower 100 mA */  
  
  /* IAD - 1 */
  0x08, /* bLenght */
  0x0B, /* bDescriptorType */
  0x00, /* bFirstInterface */
  0x02, /* bInterfaceCount */
  0x02, /* bFunctionClass */
  0x02, /* bFunctionSubClass */
  0x01, /* bFunctionProtocol */
  0x01, /* iFunction */
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_CMD_INTERFACE_1,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT      ,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP1,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0xFF,                           /* bInterval: */
  
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  CDC_DATA_INTERFACE_1,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP1,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  0x40,                              /* wMaxPacketSize: */
  0x00,
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType: Endpoint */
  CDC_IN_EP1,                        /* bEndpointAddress */
  0x02,                             /* bmAttributes: Bulk */
  0x40,                             /* wMaxPacketSize: */
  0x00,
  0x00,                              /* bInterval */
  
  /* IAD - 2 */
  0x08, /* bLenght */
  0x0B, /* bDescriptorType */
  0x02, /* bFirstInterface */
  0x02, /* bInterfaceCount */
  0x02, /* bFunctionClass */
  0x02, /* bFunctionSubClass */
  0x01, /* bFunctionProtocol */
  0x01, /* iFunction */
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_CMD_INTERFACE_2,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x03,   /* bDataInterface: 3 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x02,   /* bMasterInterface: Communication class interface */
  0x03,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT      ,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP2,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0xFF,                           /* bInterval: */
  
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  CDC_DATA_INTERFACE_2,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP2,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  0x40,                              /* wMaxPacketSize: */
  0x00,
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType: Endpoint */
  CDC_IN_EP2,                        /* bEndpointAddress */
  0x02,                             /* bmAttributes: Bulk */
  0x40,                             /* wMaxPacketSize: */
  0x00,
  0x00                              /* bInterval */
};

/**
* @}
*/ 

/** @defgroup USBD_CDC_Private_Functions
* @{
*/ 

/**
* @brief  USBD_CDC_Init
*         Initialize the CDC interface
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_CDC_HandleTypeDef   *hcdc;
  USBD_CDC_ItfTypeDef   **icdc = (USBD_CDC_ItfTypeDef**) pdev->pUserData;
  
  if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
  {  
    /* Open EP1 IN */
    USBD_LL_OpenEP(pdev,
                   CDC_IN_EP1,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_HS_IN_PACKET_SIZE);
    
    /* Open EP1 OUT */
    USBD_LL_OpenEP(pdev,
                   CDC_OUT_EP1,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_HS_OUT_PACKET_SIZE);
    
    /* Open EP2 IN */
    USBD_LL_OpenEP(pdev,
                   CDC_IN_EP2,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_HS_IN_PACKET_SIZE);
    
    /* Open EP2 OUT */
    USBD_LL_OpenEP(pdev,
                   CDC_OUT_EP2,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_HS_OUT_PACKET_SIZE);
    
  }
  else
  {
    /* Open EP1 IN */
    USBD_LL_OpenEP(pdev,
                   CDC_IN_EP1,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_FS_IN_PACKET_SIZE);
    
    /* Open EP1 OUT */
    USBD_LL_OpenEP(pdev,
                   CDC_OUT_EP1,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_FS_OUT_PACKET_SIZE);
    
    /* Open EP2 IN */
    USBD_LL_OpenEP(pdev,
                   CDC_IN_EP2,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_FS_IN_PACKET_SIZE);
    
    /* Open EP2 OUT */
    USBD_LL_OpenEP(pdev,
                   CDC_OUT_EP2,
                   USBD_EP_TYPE_BULK,
                   CDC_DATA_FS_OUT_PACKET_SIZE);
    
  }
  /* Open Command IN EP1 */
  USBD_LL_OpenEP(pdev,
                 CDC_CMD_EP1,
                 USBD_EP_TYPE_INTR,
                 CDC_CMD_PACKET_SIZE);
  
  /* Open Command IN EP2 */
  USBD_LL_OpenEP(pdev,
                 CDC_CMD_EP2,
                 USBD_EP_TYPE_INTR,
                 CDC_CMD_PACKET_SIZE);
  
  pdev->pClassData = USBD_malloc(2 * sizeof (USBD_CDC_HandleTypeDef));
  
  if(pdev->pClassData == NULL)
  {
    ret = 1; 
  }
  else
  {
    hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
    
    /* Init  physical Interface components */
    icdc[0]->Init();
    icdc[1]->Init();
    
    /* Init Xfer states */
    hcdc[0].TxState =0;
    hcdc[0].RxState =0;
    hcdc[1].TxState =0;
    hcdc[1].RxState =0;
    
    if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
    {
      /* Prepare Out endpoint1 to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             CDC_OUT_EP1,
                             hcdc[0].RxBuffer,
                             CDC_DATA_HS_OUT_PACKET_SIZE);
      
      /* Prepare Out endpoint2 to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             CDC_OUT_EP2,
                             hcdc[1].RxBuffer,
                             CDC_DATA_HS_OUT_PACKET_SIZE);
    }
    else
    {
      /* Prepare Out endpoint1 to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             CDC_OUT_EP1,
                             hcdc[0].RxBuffer,
                             CDC_DATA_FS_OUT_PACKET_SIZE);
      
      /* Prepare Out endpoint2 to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             CDC_OUT_EP2,
                             hcdc[1].RxBuffer,
                             CDC_DATA_FS_OUT_PACKET_SIZE);
    }
  }
  return ret;
}

/**
* @brief  USBD_CDC_Init
*         DeInitialize the CDC layer
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_CDC_ItfTypeDef   **icdc = (USBD_CDC_ItfTypeDef**) pdev->pUserData;
  
  /* Close EP1 IN */
  USBD_LL_CloseEP(pdev,
                  CDC_IN_EP1);
  
  /* Close EP1 OUT */
  USBD_LL_CloseEP(pdev,
                  CDC_OUT_EP1);
  
  /* Close Command IN EP1 */
  USBD_LL_CloseEP(pdev,
                  CDC_CMD_EP1);
  
  /* Close EP2 IN */
  USBD_LL_CloseEP(pdev,
                  CDC_IN_EP2);
  
  /* Close EP2 OUT */
  USBD_LL_CloseEP(pdev,
                  CDC_OUT_EP2);
  
  /* Close Command IN EP2 */
  USBD_LL_CloseEP(pdev,
                  CDC_CMD_EP2);
  
  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    icdc[0]->DeInit();
    icdc[1]->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  
  return ret;
}

/**
* @brief  USBD_CDC_Setup
*         Handle the CDC specific requests
* @param  pdev: instance
* @param  req: usb requests
* @retval status
*/
static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  USBD_CDC_ItfTypeDef   **icdc = (USBD_CDC_ItfTypeDef**) pdev->pUserData;
  static uint8_t ifalt = 0;
  
  CurrentwIndx = req->wIndex;
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    if (req->wLength)
    {
      if (req->bmRequest & 0x80)
      {
        if(req->wIndex == CDC_CMD_INTERFACE_1)
        {
          icdc[0]->Control(req->bRequest,
                           (uint8_t *)hcdc[0].data,
                           req->wLength);
          USBD_CtlSendData (pdev, 
                            (uint8_t *)hcdc[0].data,
                            req->wLength);
        }
        else if (req->wIndex == CDC_CMD_INTERFACE_2)
        {
          icdc[1]->Control(req->bRequest,
                           (uint8_t *)hcdc[1].data,
                           req->wLength);
          USBD_CtlSendData (pdev, 
                            (uint8_t *)hcdc[1].data,
                            req->wLength);
        }
        else
        {
          //ERROR//
        }
      }
      else
      {
        if(req->wIndex == CDC_CMD_INTERFACE_1)
        {
          hcdc[0].CmdOpCode = req->bRequest;
          hcdc[0].CmdLength = req->wLength;
          USBD_CtlPrepareRx (pdev, 
                             (uint8_t *)hcdc[0].data,
                             req->wLength);
        }
        else if(req->wIndex == CDC_CMD_INTERFACE_2)
        {
          hcdc[1].CmdOpCode = req->bRequest;
          hcdc[1].CmdLength = req->wLength;
          USBD_CtlPrepareRx (pdev, 
                             (uint8_t *)hcdc[1].data,
                             req->wLength);
        }
        else
        {
          // ERROR
        }
      }
      
    }
    else
    {
      if(req->wIndex == CDC_CMD_INTERFACE_1)
      {
        icdc[0]->Control(req->bRequest,
                         (uint8_t*)req,
                         0);
      }
      else
      {
        icdc[1]->Control(req->bRequest,
                         (uint8_t*)req,
                         0);
      }
    }
    break;
    
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        &ifalt,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      break;
    }
    
  default: 
    break;
  }
  return USBD_OK;
}

/**
* @brief  USBD_CDC_DataIn
*         Data sent on non-control IN endpoint
* @param  pdev: device instance
* @param  epnum: endpoint number
* @retval status
*/
static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    switch (epnum|0x80)
    {
    case CDC_IN_EP1:
    case CDC_CMD_EP1:
      hcdc[0].TxState = 0;
      break;
      
    case CDC_IN_EP2:
    case CDC_CMD_EP2:
      hcdc[1].TxState = 0;
      break;
      
    default: 
      break;
    }
    
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
* @brief  USBD_CDC_DataOut
*         Data received on non-control Out endpoint
* @param  pdev: device instance
* @param  epnum: endpoint number
* @retval status
*/
static uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  USBD_CDC_ItfTypeDef   **icdc = (USBD_CDC_ItfTypeDef**) pdev->pUserData;
  
  /* Get the received data length */
  switch (epnum)
  {
  case CDC_OUT_EP1:
    USBD_CDC_SetRxBuffer(pdev, UserRxBuffer[0], epnum);
    hcdc[0].RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
    break;
    
  case CDC_OUT_EP2:
    USBD_CDC_SetRxBuffer(pdev, UserRxBuffer[1], epnum);
    hcdc[1].RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
    break;
    
  default: 
    break;
  }
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the application Xfer */
  if(pdev->pClassData != NULL)
  {
    switch (epnum)
    {
    case CDC_OUT_EP1:
      icdc[0]->Receive(hcdc[0].RxBuffer, &(hcdc[0].RxLength));
      break;
      
    case CDC_OUT_EP2:
      icdc[1]->Receive(hcdc[1].RxBuffer, &(hcdc[1].RxLength));
      break;
      
    default: 
      break;
    }
    
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}



/**
* @brief  USBD_CDC_DataOut
*         Data received on non-control Out endpoint
* @param  pdev: device instance
* @retval status
*/
static uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{ 
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  USBD_CDC_ItfTypeDef   **icdc = (USBD_CDC_ItfTypeDef**) pdev->pUserData;
  
  if (CurrentwIndx == CDC_CMD_INTERFACE_1)
  {
    if((icdc[0] != NULL) && (hcdc[0].CmdOpCode != 0xFF))
    {
      icdc[0]->Control(hcdc[0].CmdOpCode,
                       (uint8_t *)hcdc[0].data,
                       hcdc[0].CmdLength);
      hcdc[0].CmdOpCode = 0xFF;
    }
  }
  else if (CurrentwIndx == CDC_CMD_INTERFACE_2)
  {
    if((icdc[1] != NULL) && (hcdc[1].CmdOpCode != 0xFF))
    {
      icdc[1]->Control(hcdc[1].CmdOpCode,
                       (uint8_t *)hcdc[1].data,
                       hcdc[1].CmdLength);
      hcdc[1].CmdOpCode = 0xFF; 
    }
  }
  else
  {
    //ERROR
  }
  
  return USBD_OK;
}

/**
* @brief  USBD_CDC_GetFSCfgDesc 
*         Return configuration descriptor
* @param  speed : current device speed
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_CDC_GetFSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_CDC_CfgFSDesc);
  return USBD_CDC_CfgFSDesc;
}

/**
* @brief  USBD_CDC_GetHSCfgDesc 
*         Return configuration descriptor
* @param  speed : current device speed
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_CDC_GetHSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_CDC_CfgHSDesc);
  return USBD_CDC_CfgHSDesc;
}

/**
* @brief  USBD_CDC_GetCfgDesc 
*         Return configuration descriptor
* @param  speed : current device speed
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_CDC_GetOtherSpeedCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_CDC_OtherSpeedCfgDesc);
  return USBD_CDC_OtherSpeedCfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_CDC_GetDeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_CDC_DeviceQualifierDesc);
  return USBD_CDC_DeviceQualifierDesc;
}

/**
* @brief  USBD_CDC_RegisterInterface
* @param  pdev: device instance
* @param  fops: CD  Interface callback
* @retval status
*/
uint8_t  USBD_CDC_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      USBD_CDC_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;
  USBD_CDC_ItfTypeDef **icdc;
  
  pdev->pUserData = USBD_malloc(2 * sizeof (USBD_CDC_ItfTypeDef*));
  
  if(pdev->pUserData == NULL)
  {
    ret = 1; 
  }
  else
  {
    icdc = (USBD_CDC_ItfTypeDef**) pdev->pUserData;
    
    icdc[0] = &fops[0];
    icdc[1] = &fops[1];
    ret = USBD_OK;    
  }
  
  return ret;
}

/**
* @brief  USBD_CDC_SetTxBuffer
* @param  pdev: device instance
* @param  pbuff: Tx Buffer
* @param  length : pointer data length
* @param  epnum: endpoint number
* @retval status
*/
uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length,
                                uint8_t  epnum)
{
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  
  if (epnum == CDC_IN_EP1)
  {
    hcdc[0].TxBuffer = pbuff;
    hcdc[0].TxLength = length;
  }
  else if (epnum == CDC_IN_EP2)
  {
    hcdc[1].TxBuffer = pbuff;
    hcdc[1].TxLength = length;
  }
  else
  {
    //ERROR
  }
  
  return USBD_OK;  
}


/**
* @brief  USBD_CDC_SetRxBuffer
* @param  pdev: device instance
* @param  pbuff: Rx Buffer
* @param  epnum: endpoint number
* @retval status
*/
uint8_t  USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint8_t   epnum)
{
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  
  if (epnum == CDC_OUT_EP1)
  {
    hcdc[0].RxBuffer = pbuff;
  }
  else if (epnum == CDC_OUT_EP2)
  {
    hcdc[1].RxBuffer = pbuff;
  }
  else
  {
    //ERROR
  }
  
  return USBD_OK;
}

/**
* @brief  USBD_CDC_DataOut
*         Data received on non-control Out endpoint
* @param  pdev: device instance
* @param  epnum: endpoint number
* @retval status
*/
uint8_t  USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    if((hcdc[0].TxState != 0) && (hcdc[1].TxState != 0))
    {
      return USBD_BUSY;
    }
    else
    {
      if((epnum == CDC_IN_EP1) && (hcdc[0].TxState == 0))
      {
        /* Tx Transfer in progress */
        hcdc[0].TxState = 1;
        
        /* Transmit next packet */
        USBD_LL_Transmit(pdev,
                         CDC_IN_EP1,
                         hcdc[0].TxBuffer,
                         hcdc[0].TxLength);
      }
      
      if((epnum == CDC_IN_EP2) && (hcdc[1].TxState == 0))
      {
        /* Tx Transfer in progress */
        hcdc[1].TxState = 1;
        
        /* Transmit next packet */      
        USBD_LL_Transmit(pdev,
                         CDC_IN_EP2,
                         hcdc[1].TxBuffer,
                         hcdc[1].TxLength);
      }
      
      return USBD_OK;
    }
  }
  else
  {
    return USBD_FAIL;
  }
}


/**
* @brief  USBD_CDC_ReceivePacket
*         prepare OUT Endpoint for reception
* @param  pdev: device instance
* @param  Port: Port number
* @retval status
*/
uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev, uint32_t Port)
{      
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;
  
  /* Suspend or Resume USB Out process */
  if(pdev->pClassData != NULL)
  {
    if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
    {      
      if (Port == 0)
      {
        /* Prepare Out endpoint to receive next packet */
        USBD_LL_PrepareReceive(pdev,
                               CDC_OUT_EP1,
                               hcdc[0].RxBuffer,
                               CDC_DATA_HS_OUT_PACKET_SIZE);
      }
      else
      {
        USBD_LL_PrepareReceive(pdev,
                               CDC_OUT_EP2,
                               hcdc[1].RxBuffer,
                               CDC_DATA_HS_OUT_PACKET_SIZE);
      }
    }
    else
    {
      if (Port == 0)
      {
        /* Prepare Out endpoint to receive next packet */
        USBD_LL_PrepareReceive(pdev,
                               CDC_OUT_EP1,
                               hcdc[0].RxBuffer,
                               CDC_DATA_FS_OUT_PACKET_SIZE);
      }
      else
      {
        
        /* Prepare Out endpoint to receive next packet */
        USBD_LL_PrepareReceive(pdev,
                               CDC_OUT_EP2,
                               hcdc[1].RxBuffer,
                               CDC_DATA_FS_OUT_PACKET_SIZE);
      }
    }
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_CDC_GetUsrStrDescriptor 
  *         return user defined string descriptors
  * @param  pdev : device handler
  * @param  index : index of the string descriptor requested by host
  * @param  length : pointer to data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetUsrStrDescriptor1 (uint16_t *length)
{
	USBD_GetString((uint8_t *)INTERFACE1_STRING_DESCRIPTOR, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
  * @brief  USBD_CDC_GetUsrStrDescriptor 
  *         return user defined string descriptors
  * @param  pdev : device handler
  * @param  index : index of the string descriptor requested by host
  * @param  length : pointer to data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_GetUsrStrDescriptor2 (uint16_t *length)
{
  USBD_GetString((uint8_t *)INTERFACE2_STRING_DESCRIPTOR, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
