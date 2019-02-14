
#ifndef __USB_SPEC_H__
#define __USB_SPEC_H__

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
// HID
#define                             UHS_USB_CLASS_HID 0x03U
////////////////////////////////////////////////////////////////////////////////
// Physical
#define                        UHS_USB_CLASS_PHYSICAL 0x05U

////////////////////////////////////////////////////////////////////////////////

#define                              USB_XFER_TIMEOUT 5000    //USB transfer timeout in milliseconds, per section 9.2.6.1 of USB 2.0 spec
#define                                 USB_NAK_LIMIT 32000   //NAK limit for a transfer. o meand NAKs are not counted
#define                               USB_RETRY_LIMIT 3       //retry limit for a transfer
#define                              USB_SETTLE_DELAY 200     //settle delay in milliseconds
#define                                USB_NAK_NOWAIT 1       //used in Richard's PS2/Wiimote code

#define                                USB_NUMDEVICES 2           //number of USB devices

// USB state machine states
#define                       UHS_USB_HOST_STATE_MASK 0xF0U

// Configure states, MSN == 0 --------------------------V
#define                   UHS_USB_HOST_STATE_DETACHED 0x00U
#define                   UHS_USB_HOST_STATE_DEBOUNCE 0x01U
#define      UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE 0x02U
#define         UHS_USB_HOST_STATE_RESET_NOT_COMPLETE 0x03U
#define                   UHS_USB_HOST_STATE_WAIT_SOF 0x04U
#define             UHS_USB_HOST_STATE_WAIT_BUS_READY 0x05U
#define               UHS_USB_HOST_STATE_RESET_DEVICE 0x0AU
#define                UHS_USB_HOST_STATE_CONFIGURING 0x0CU // Looks like "CO"nfig (backwards)
#define           UHS_USB_HOST_STATE_CONFIGURING_DONE 0x0DU // Looks like "DO"one (backwards)
#define                      UHS_USB_HOST_STATE_CHECK 0x0EU
#define                    UHS_USB_HOST_STATE_ILLEGAL 0x0FU // Foo

// Run states, MSN != 0 --------------------------------V
#define                    UHS_USB_HOST_STATE_RUNNING 0x60U // Looks like "GO"
#define                       UHS_USB_HOST_STATE_IDLE 0x1DU // Looks like "ID"le
#define                      UHS_USB_HOST_STATE_ERROR 0xF0U // Looks like "FO"o
#define                 UHS_USB_HOST_STATE_INITIALIZE 0x10U // Looks like "I"nit

// Host SE result codes.
// Common SE results are stored in the low nybble, all interface drivers understand these plus 0x1f.
// Extended SE results are 0x10-0x1e. SE code only understands these internal to the hardware.
// Values > 0x1F are driver or other internal error conditions.
// Return these result codes from your host controller driver to match the error condition
// ALL Non-zero values are errors.
// Values not listed in this table are not handled in the base class, or any host driver.

#define                           UHS_HOST_ERROR_NONE 0x00U // No error
#define                           UHS_HOST_ERROR_BUSY 0x01U // transfer pending
#define                         UHS_HOST_ERROR_BADREQ 0x02U // Transfer Launch Request was bad
#define                            UHS_HOST_ERROR_DMA 0x03U // DMA was too short, or too long
#define                            UHS_HOST_ERROR_NAK 0x04U // Peripheral returned NAK
#define                          UHS_HOST_ERROR_STALL 0x05U // Peripheral returned STALL
#define                         UHS_HOST_ERROR_TOGERR 0x06U // Toggle error/ISO over-underrun
#define                       UHS_HOST_ERROR_WRONGPID 0x07U // Received wrong Packet ID
#define                          UHS_HOST_ERROR_BADBC 0x08U // Byte count is bad
#define                         UHS_HOST_ERROR_PIDERR 0x09U // Received Packet ID is corrupted
#define                          UHS_HOST_ERROR_BADRQ 0x0AU // Packet error. Increase max packet.
#define                            UHS_HOST_ERROR_CRC 0x0BU // USB CRC was incorrect
#define                           UHS_HOST_ERROR_KERR 0x0CU // K-state instead of response, usually indicates wrong speed
#define                           UHS_HOST_ERROR_JERR 0x0DU // J-state instead of response, usually indicates wrong speed
#define                        UHS_HOST_ERROR_TIMEOUT 0x0EU // Device did not respond in time
#define                         UHS_HOST_ERROR_BABBLE 0x0FU // Line noise/unexpected data
#define                        UHS_HOST_ERROR_MEM_LAT 0x10U // Error caused by memory latency.
#define                           UHS_HOST_ERROR_NYET 0x11U // OUT transfer accepted with NYET

// Addressing error codes
#define                      ADDR_ERROR_INVALID_INDEX 0xA0U
#define                    ADDR_ERROR_INVALID_ADDRESS 0xA1U

// Common Interface Driver error codes
#define           UHS_HOST_ERROR_DEVICE_NOT_SUPPORTED 0xD1U // Driver doesn't support the device or interfaces
#define         UHS_HOST_ERROR_DEVICE_INIT_INCOMPLETE 0xD2U // Init partially finished, but died.
#define     UHS_HOST_ERROR_CANT_REGISTER_DEVICE_CLASS 0xD3U // There was no driver for the interface requested.
#define              UHS_HOST_ERROR_ADDRESS_POOL_FULL 0xD4U // No addresses left in the address pool.
#define           UHS_HOST_ERROR_HUB_ADDRESS_OVERFLOW 0xD5U // No hub addresses left. The maximum is 7.
#define             UHS_HOST_ERROR_NO_ADDRESS_IN_POOL 0xD6U // Address was not allocated in the pool, thus not found.
#define                    UHS_HOST_ERROR_NULL_EPINFO 0xD7U // The supplied endpoint was NULL, indicates a bug or other problem.
#define                   UHS_HOST_ERROR_BAD_ARGUMENT 0xD8U // Indicates a range violation bug.
#define             UHS_HOST_ERROR_DEVICE_DRIVER_BUSY 0xD9U // The interface driver is busy or out buffer is full, try again later.
#define            UHS_HOST_ERROR_BAD_MAX_PACKET_SIZE 0xDAU // The maximum packet size was exceeded. Try again with smaller size.
#define           UHS_HOST_ERROR_NO_ENDPOINT_IN_TABLE 0xDBU // The endpoint could not be found in the endpoint table.
#define                      UHS_HOST_ERROR_UNPLUGGED 0xDEU // Someone removed the USB device, or Vbus was turned off.
#define                          UHS_HOST_ERROR_NOMEM 0xDFU // Out Of Memory.

// Control request stream errors
#define                UHS_HOST_ERROR_FailGetDevDescr 0xE1U
#define             UHS_HOST_ERROR_FailSetDevTblEntry 0xE2U
#define               UHS_HOST_ERROR_FailGetConfDescr 0xE3U
#define                    UHS_HOST_ERROR_FailSetConf 0xE4U
#define                    UHS_HOST_ERROR_FailGetHIDr 0xE5U
#define                  UHS_HOST_ERROR_END_OF_STREAM 0xEFU

// Host base class specific Error codes
#define                UHS_HOST_ERROR_NOT_IMPLEMENTED 0xFEU
#define               UHS_HOST_ERROR_TRANSFER_TIMEOUT 0xFFU

// SEI interaction defaults
#define                      UHS_HOST_TRANSFER_MAX_MS 10000 // USB transfer timeout in ms, per section 9.2.6.1 of USB 2.0 spec
#define               UHS_HOST_TRANSFER_RETRY_MAXIMUM 3     // 3 retry limit for a transfer
#define                    UHS_HOST_DEBOUNCE_DELAY_MS 500   // settle delay in milliseconds
#define                        UHS_HUB_RESET_DELAY_MS 20    // hub port reset delay, 10ms recomended, but can be up to 20ms

// Misc.USB constants
#define                           DEV_DESCR_LEN 18      //device descriptor length
#define                          CONF_DESCR_LEN 9       //configuration descriptor length
#define                          INTR_DESCR_LEN 9       //interface descriptor length
#define                            EP_DESCR_LEN 7       //endpoint descriptor length

// Standard Device Requests
#define                  USB_REQUEST_GET_STATUS 0       // Standard Device Request - GET STATUS
#define               USB_REQUEST_CLEAR_FEATURE 1       // Standard Device Request - CLEAR FEATURE
#define                 USB_REQUEST_SET_FEATURE 3       // Standard Device Request - SET FEATURE
#define                 USB_REQUEST_SET_ADDRESS 5       // Standard Device Request - SET ADDRESS
#define              USB_REQUEST_GET_DESCRIPTOR 6       // Standard Device Request - GET DESCRIPTOR
#define              USB_REQUEST_SET_DESCRIPTOR 7       // Standard Device Request - SET DESCRIPTOR
#define           USB_REQUEST_GET_CONFIGURATION 8       // Standard Device Request - GET CONFIGURATION
#define           USB_REQUEST_SET_CONFIGURATION 9       // Standard Device Request - SET CONFIGURATION
#define               USB_REQUEST_GET_INTERFACE 10      // Standard Device Request - GET INTERFACE
#define               USB_REQUEST_SET_INTERFACE 11      // Standard Device Request - SET INTERFACE
#define                 USB_REQUEST_SYNCH_FRAME 12      // Standard Device Request - SYNCH FRAME

// Wireless USB Device Requests
#define                  USB_REQ_SET_ENCRYPTION 0x0D
#define                  USB_REQ_GET_ENCRYPTION 0x0E
#define                     USB_REQ_RPIPE_ABORT 0x0E
#define                   USB_REQ_SET_HANDSHAKE 0x0F
#define                     USB_REQ_RPIPE_RESET 0x0F
#define                   USB_REQ_GET_HANDSHAKE 0x10
#define                  USB_REQ_SET_CONNECTION 0x11
#define               USB_REQ_SET_SECURITY_DATA 0x12
#define               USB_REQ_GET_SECURITY_DATA 0x13
#define                   USB_REQ_SET_WUSB_DATA 0x14
#define             USB_REQ_LOOPBACK_DATA_WRITE 0x15
#define              USB_REQ_LOOPBACK_DATA_READ 0x16
#define                USB_REQ_SET_INTERFACE_DS 0x17

// USB feature flags
#define                 USB_DEVICE_SELF_POWERED 0   /* (read only) */
#define                USB_DEVICE_REMOTE_WAKEUP 1   /* dev may initiate wakeup */
#define                    USB_DEVICE_TEST_MODE 2   /* (wired high speed only) */
#define                      USB_DEVICE_BATTERY 2   /* (wireless) */
#define                 USB_DEVICE_B_HNP_ENABLE 3   /* (otg) dev may initiate HNP */
#define                  USB_DEVICE_WUSB_DEVICE 3   /* (wireless)*/
#define                USB_DEVICE_A_HNP_SUPPORT 4   /* (otg) RH port supports HNP */
#define            USB_DEVICE_A_ALT_HNP_SUPPORT 5   /* (otg) other RH port does */
#define                   USB_DEVICE_DEBUG_MODE 6   /* (special devices only) */

#define               USB_FEATURE_ENDPOINT_HALT 0       // CLEAR/SET FEATURE - Endpoint Halt
#define        USB_FEATURE_DEVICE_REMOTE_WAKEUP 1       // CLEAR/SET FEATURE - Device remote wake-up
#define                   USB_FEATURE_TEST_MODE 2       // CLEAR/SET FEATURE - Test mode
// OTG SET FEATURE Constants
#define               OTG_FEATURE_B_HNP_ENABLE  3       // SET FEATURE OTG - Enable B device to perform HNP
#define               OTG_FEATURE_A_HNP_SUPPORT 4       // SET FEATURE OTG - A device supports HNP
#define           OTG_FEATURE_A_ALT_HNP_SUPPORT 5       // SET FEATURE OTG - Another port on the A device supports HNP

// Setup Data Constants
#define                USB_SETUP_HOST_TO_DEVICE 0x00    // Device Request bmRequestType transfer direction - host to device transfer
#define                USB_SETUP_DEVICE_TO_HOST 0x80    // Device Request bmRequestType transfer direction - device to host transfer
#define                 USB_SETUP_TYPE_STANDARD 0x00    // Device Request bmRequestType type - standard
#define                    USB_SETUP_TYPE_CLASS 0x20    // Device Request bmRequestType type - class
#define                   USB_SETUP_TYPE_VENDOR 0x40    // Device Request bmRequestType type - vendor
#define              USB_SETUP_RECIPIENT_DEVICE 0x00    // Device Request bmRequestType recipient - device
#define           USB_SETUP_RECIPIENT_INTERFACE 0x01    // Device Request bmRequestType recipient - interface
#define            USB_SETUP_RECIPIENT_ENDPOINT 0x02    // Device Request bmRequestType recipient - endpoint
#define               USB_SETUP_RECIPIENT_OTHER 0x03    // Device Request bmRequestType recipient - other
#define                USB_SETUP_RECIPIENT_PORT 0x04    // Wireless USB 1.0
#define               USB_SETUP_RECIPIENT_RPIPE 0x05    // Wireless USB 1.0


// USB descriptors
#define                   USB_DESCRIPTOR_DEVICE 0x01    // bDescriptorType for a Device Descriptor.
#define            USB_DESCRIPTOR_CONFIGURATION 0x02    // bDescriptorType for a Configuration Descriptor.
#define                   USB_DESCRIPTOR_STRING 0x03    // bDescriptorType for a String Descriptor.
#define                USB_DESCRIPTOR_INTERFACE 0x04    // bDescriptorType for an Interface Descriptor.
#define                 USB_DESCRIPTOR_ENDPOINT 0x05    // bDescriptorType for an Endpoint Descriptor.
#define         USB_DESCRIPTOR_DEVICE_QUALIFIER 0x06    // bDescriptorType for a Device Qualifier.
#define              USB_DESCRIPTOR_OTHER_SPEED 0x07    // bDescriptorType for a Other Speed Configuration.
#define          USB_DESCRIPTOR_INTERFACE_POWER 0x08    // bDescriptorType for Interface Power.
#define                      USB_DESCRIPTOR_OTG 0x09    // bDescriptorType for an OTG Descriptor.
#define                    USB_DESCRIPTOR_DEBUG 0x0a
#define    USB_DESCRIPTOR_INTERFACE_ASSOCIATION 0x0b
#define                 USB_DESCRIPTOR_SECURITY 0x0c
#define                      USB_DESCRIPTOR_KEY 0x0d
#define          USB_DESCRIPTOR_ENCRYPTION_TYPE 0x0e
#define                      USB_DESCRIPTOR_BOS 0x0f
#define        USB_DESCRIPTOR_DEVICE_CAPABILITY 0x10
#define   USB_DESCRIPTOR_WIRELESS_ENDPOINT_COMP 0x11
#define             USB_DESCRIPTOR_WIRE_ADAPTER 0x21
#define                    USB_DESCRIPTOR_RPIPE 0x22
#define         USB_DESCRIPTOR_CS_RADIO_CONTROL 0x23
#define         USB_DESCRIPTOR_SS_ENDPOINT_COMP 0x30

#define                          HID_DESCRIPTOR 0x21


// Conventional codes for class-specific descriptors. "Common Class" Spec (3.11)
#define                USB_DESCRIPTOR_CS_DEVICE 0x21
#define                USB_DESCRIPTOR_CS_CONFIG 0x22
#define                USB_DESCRIPTOR_CS_STRING 0x23
#define             USB_DESCRIPTOR_CS_INTERFACE 0x24
#define              USB_DESCRIPTOR_CS_ENDPOINT 0x25



// USB Endpoint Transfer Types
#define               USB_TRANSFER_TYPE_CONTROL 0x00    // Endpoint is a control endpoint.
#define           USB_TRANSFER_TYPE_ISOCHRONOUS 0x01    // Endpoint is an isochronous endpoint.
#define                  USB_TRANSFER_TYPE_BULK 0x02    // Endpoint is a bulk endpoint.
#define             USB_TRANSFER_TYPE_INTERRUPT 0x03    // Endpoint is an interrupt endpoint.
#define                     bmUSB_TRANSFER_TYPE 0x03    // bit mask to separate transfer type from ISO attributes
#define               USB_TRANSFER_DIRECTION_IN 0x80    // Indicate direction is IN

// Standard Feature Selectors for CLEAR_FEATURE Requests
#define              USB_FEATURE_ENDPOINT_STALL 0       // Endpoint recipient
#define        USB_FEATURE_DEVICE_REMOTE_WAKEUP 1       // Device recipient
#define                   USB_FEATURE_TEST_MODE 2       // Device recipient

//get descriptor request type
#define                           UHS_bmREQ_GET_DESCR (USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE)

//set request type for all but 'set feature' and 'set interface'
#define                                 UHS_bmREQ_SET (USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE)

//get interface request type
#define                         UHS_bmREQ_CL_GET_INTF (USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE)

// D7           data transfer direction (0 - host-to-device, 1 - device-to-host)
// D6-5         Type (0- standard, 1 - class, 2 - vendor, 3 - reserved)
// D4-0         Recipient (0 - device, 1 - interface, 2 - endpoint, 3 - other, 4..31 - reserved)




// Device descriptor structure
typedef struct {
    uint8_t bLength; // size of descriptor in bytes
    uint8_t bDescriptorType; // DEVICE descriptor type (USB_DESCRIPTOR_DEVICE).
    uint16_t bcdUSB; // USB Spec Release Number (BCD).
    uint8_t bDeviceClass; // Class code (assigned by the USB-IF). 0xFF-Vendor specific.
    uint8_t bDeviceSubClass; // Subclass code (assigned by the USB-IF).
    uint8_t bDeviceProtocol; // Protocol code (assigned by the USB-IF). 0xFF-Vendor specific.
    uint8_t bMaxPacketSize0; // Maximum packet size for endpoint 0.
    uint16_t idVendor; // Vendor ID (assigned by the USB-IF).
    uint16_t idProduct; // Product ID (assigned by the manufacturer).
    uint16_t bcdDevice; // Device release number (BCD).
    uint8_t iManufacturer; // Index of String Descriptor describing the manufacturer.
    uint8_t iProduct; // Index of String Descriptor describing the product.
    uint8_t iSerialNumber; // Index of String Descriptor with the device's serial number.
    uint8_t bNumConfigurations; // Number of possible configurations.
} __attribute__((packed)) USB_DEVICE_DESCRIPTOR;

// Configuration descriptor structure
typedef struct {
    uint8_t bLength; // Length of this descriptor.
    uint8_t bDescriptorType; // CONFIGURATION descriptor type (USB_DESCRIPTOR_CONFIGURATION).
    uint16_t wTotalLength; // Total length of all descriptors for this configuration.
    uint8_t bNumInterfaces; // Number of interfaces in this configuration.
    uint8_t bConfigurationValue; // Value of this configuration (1 based).
    uint8_t iConfiguration; // Index of String Descriptor describing the configuration.
    uint8_t bmAttributes; // Configuration characteristics.
    uint8_t bMaxPower; // Maximum power consumed by this configuration.
} __attribute__((packed)) USB_CONFIGURATION_DESCRIPTOR;

// Interface descriptor structure
typedef struct {
    uint8_t bLength; // Length of this descriptor.
    uint8_t bDescriptorType; // INTERFACE descriptor type (USB_DESCRIPTOR_INTERFACE).
    uint8_t bInterfaceNumber; // Number of this interface (0 based).
    uint8_t bAlternateSetting; // Value of this alternate interface setting.
    uint8_t bNumEndpoints; // Number of endpoints in this interface.
    uint8_t bInterfaceClass; // Class code (assigned by the USB-IF).  0xFF-Vendor specific.
    uint8_t bInterfaceSubClass; // Subclass code (assigned by the USB-IF).
    uint8_t bInterfaceProtocol; // Protocol code (assigned by the USB-IF).  0xFF-Vendor specific.
    uint8_t iInterface; // Index of String Descriptor describing the interface.
} __attribute__((packed)) USB_INTERFACE_DESCRIPTOR;

// Endpoint descriptor structure
typedef struct {
    uint8_t bLength; // Length of this descriptor.
    uint8_t bDescriptorType; // ENDPOINT descriptor type (USB_DESCRIPTOR_ENDPOINT).
    uint8_t bEndpointAddress; // Endpoint address. Bit 7 indicates direction (0=OUT, 1=IN).
    uint8_t bmAttributes; // Endpoint transfer type.
    uint16_t wMaxPacketSize; // Maximum packet size.
    uint8_t bInterval; // Polling interval in frames.
} __attribute__((packed)) USB_ENDPOINT_DESCRIPTOR;

// HID descriptor
typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdHID; // HID class specification release
    uint8_t bCountryCode;
    uint8_t bNumDescriptors; // Number of additional class specific descriptors
    uint8_t bDescrType; // Type of class descriptor
    uint16_t wDescriptorLength; // Total size of the Report descriptor
} __attribute__((packed)) USB_HID_DESCRIPTOR;

struct ENDPOINT_INFO {
    uint8_t bEndpointAddress;       // Endpoint address. Bit 7 indicates direction (0=OUT, 1=IN).
    uint8_t bmAttributes;           // Endpoint transfer type.
    uint16_t wMaxPacketSize;        // Maximum packet size.
    uint8_t bInterval;              // Polling interval in frames.
} __attribute__((packed));

struct INTERFACE_INFO {
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t numep;
    uint8_t klass;
    uint8_t subklass;
    uint8_t protocol;
    struct ENDPOINT_INFO epInfo[16];
} __attribute__((packed));

struct ENUMERATION_INFO {
    uint16_t vid;
    uint16_t pid;
    uint16_t bcdDevice;
    uint8_t klass;
    uint8_t subklass;
    uint8_t protocol;
    uint8_t bMaxPacketSize0;
    uint8_t currentconfig;
    uint8_t parent;
    uint8_t port;
    uint8_t address;
    struct INTERFACE_INFO interface;
} __attribute__((packed));

// USB Setup Packet Structure
// USB spec 2.0 chapter 9.3
struct SETUP_PKT {
    uint8_t bmRequestType;
    uint8_t bRequest;
    union {
        uint16_t wValue;
        struct {
            uint8_t wValueLo;
            uint8_t wValueHi;
        } __attribute__((packed));
    } wVal_u;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__((packed));

/*
// Endpoint information structure
// bToggle of endpoint 0 initialized to 0xff
// during enumeration bToggle is set to 00
typedef struct {        
    uint8_t epAddr;        //copy from endpoint descriptor. Bit 7 indicates direction ( ignored for control endpoints )
    uint8_t Attr;          // Endpoint transfer type.
    uint16_t MaxPktSize;    // Maximum packet size.
    uint8_t Interval;      // Polling interval in frames.
    uint8_t sndToggle;     //last toggle value, bitmask for HCTL toggle bits
    uint8_t rcvToggle;     //last toggle value, bitmask for HCTL toggle bits
} __attribute__((packed)) EP_RECORD;

// device record structure
typedef struct {
    EP_RECORD* epinfo;      //device endpoint information
    uint8_t devclass;          //device class    
} __attribute__((packed)) DEV_RECORD;
*/

/* NAK powers. To save space in endpoint data structure, amount of retries before giving up and returning 0x4 is stored in */
/* bmNakPower as a power of 2. The actual nak_limit is then calculated as nak_limit = ( 2^bmNakPower - 1) */
#define UHS_USB_NAK_MAX_POWER               14      // NAK binary order maximum value
#define UHS_USB_NAK_DEFAULT                 2       // default 16K-1 NAKs before giving up
#define UHS_USB_NAK_NOWAIT                  1       // Single NAK stops transfer
#define UHS_USB_NAK_NONAK                   0       // Do not count NAKs, stop retrying after USB Timeout. Try not to use this.

#define bmUSB_DEV_ADDR_PORT             0x07
#define bmUSB_DEV_ADDR_PARENT           0x78
#define bmUSB_DEV_ADDR_HUB              0x40

// TODO: embed parent?
struct UHS_EpInfo {
    uint8_t epAddr; // Endpoint address
    uint8_t bIface;
    uint16_t maxPktSize; // Maximum packet size

    union {
        uint8_t epAttribs;

        struct {
            uint8_t bmSndToggle : 1; // Send toggle, when zero bmSNDTOG0, bmSNDTOG1 otherwise
            uint8_t bmRcvToggle : 1; // Send toggle, when zero bmRCVTOG0, bmRCVTOG1 otherwise
            uint8_t bmNeedPing : 1; // 1 == ping protocol needed for next out packet
            uint8_t bmNakPower : 5; // Binary order for NAK_LIMIT value
        } __attribute__((packed));
    };
} __attribute__((packed));

// TODO: embed parent address and port into epinfo struct,
// and nuke this address stupidity.
// This is a compact scheme. Should also support full spec.
// This produces a 7 hub limit, 49 devices + 7 hubs, 56 total.
//
//    7   6   5   4   3   2   1   0
//  ---------------------------------
//  |   | H | P | P | P | A | A | A |
//  ---------------------------------
//
// H - if 1 the address is a hub address
// P - parent hub number
// A - port number of parent
//

struct UHS_DeviceAddress {
    union {
        struct {
            uint8_t bmAddress : 3; // port number
            uint8_t bmParent : 3; // parent hub address
            uint8_t bmHub : 1; // hub flag
            uint8_t bmReserved : 1; // reserved, must be zero
        } __attribute__((packed));
        uint8_t devAddress;
    };
} __attribute__((packed));

#define UHS_HOST_MAX_INTERFACE_DRIVERS 2

struct UHS_Device {
    volatile struct UHS_EpInfo *epinfo[UHS_HOST_MAX_INTERFACE_DRIVERS]; // endpoint info pointer
    struct UHS_DeviceAddress address;
    uint8_t epcount; // number of endpoints
    uint8_t speed; // indicates device speed
} __attribute__((packed));


// little endian :-)                                                                             8                                8                          8                         8                          16                      16
#define mkSETUP_PKT8(bmReqType, bRequest, wValLo, wValHi, wInd, total) ((uint64_t)(((uint64_t)(bmReqType)))|(((uint64_t)(bRequest))<<8)|(((uint64_t)(wValLo))<<16)|(((uint64_t)(wValHi))<<24)|(((uint64_t)(wInd))<<32)|(((uint64_t)(total)<<48)))
#define mkSETUP_PKT16(bmReqType, bRequest, wVal, wInd, total)          ((uint64_t)(((uint64_t)(bmReqType)))|(((uint64_t)(bRequest))<<8)|(((uint64_t)(wVal  ))<<16)                           |(((uint64_t)(wInd))<<32)|(((uint64_t)(total)<<48)))

// Big endian -- but we aren't able to use this :-/
//#define mkSETUP_PKT8(bmReqType, bRequest, wValLo, wValHi, wInd, total) ((uint64_t)(((uint64_t)(bmReqType))<<56)|(((uint64_t)(bRequest))<<48)|(((uint64_t)(wValLo))<<40)|(((uint64_t)(wValHi))<<32)|(((uint64_t)(wInd))<<16)|((uint64_t)(total)))
//#define mkSETUP_PKT16(bmReqType, bRequest, wVal, wInd, total)          ((uint64_t)(((uint64_t)(bmReqType))<<56)|(((uint64_t)(bRequest))<<48)                           |(((uint64_t)(wVal))<<32)  |(((uint64_t)(wInd))<<16)|((uint64_t)(total)))

#ifdef __cplusplus
}
#endif

#endif

