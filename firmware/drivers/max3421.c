
// C library for the MAX3421E USB peripheral/host controller IC
//   author:          Petre Rodan <2b4eda@subdimension.ro>
//   available from:  https://github.com/rodan/
//   license:         GNU GPLv3

// USB library stol^W borrowed from https://github.com/felis/UHS30
/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Circuits At Home, LTD
Web      :  http://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
*/

#include <inttypes.h>
#include <string.h>
#include "max3421.h"
#include "spi.h"
#include "drivers/sys_messagebus.h"
#include "drivers/timer_a0.h"
#include "proj.h"
#include "usb_spec.h"

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_NONE)
#include "drivers/uart0.h"
#include "drivers/helper.h"
char itoa_buf[18];
#endif

volatile uint8_t vbusState;
volatile uint16_t sof_countdown;
volatile uint8_t bus_event;

volatile uint8_t usb_error;
volatile uint8_t usb_task_state;
//volatile uint8_t usb_task_polling_disabled;
volatile uint8_t usb_host_speed;
//volatile uint8_t hub_present;

volatile uint8_t ifg_int_last_event;
volatile uint8_t ifg_gpx_last_event;

struct UHS_Device thePool[UHS_HOST_MAX_INTERFACE_DRIVERS];
// Endpoint data structure used during enumeration for uninitialized device
struct UHS_EpInfo dev0ep;

void VBUS_changed(void)
{
    /* modify USB task state because Vbus changed or unknown */
    uint8_t speed = 1;
    uint8_t iostate;

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_INFO)
    uart0_print("state ");
    uart0_print(_utoh(itoa_buf, usb_task_state));
#endif
    switch (vbusState) {
    case LSHOST:               // Low speed
        speed = 0;
        // Intentional fall-through
    case FSHOST:               // Full speed
        // Start device initialization if we are not initializing
        // Resets to the device cause an IRQ
        // usb_task_state == UHS_USB_HOST_STATE_RESET_NOT_COMPLETE;
        //if((usb_task_state & UHS_USB_HOST_STATE_MASK) != UHS_USB_HOST_STATE_DETACHED) {
        //ReleaseChildren(); // FIXME
        if (!(bus_event & doingreset)) {
            if (usb_task_state == UHS_USB_HOST_STATE_RESET_NOT_COMPLETE) {
                usb_task_state = UHS_USB_HOST_STATE_WAIT_BUS_READY;
            } else if (usb_task_state != UHS_USB_HOST_STATE_WAIT_BUS_READY) {
                usb_task_state = UHS_USB_HOST_STATE_DEBOUNCE;
            }
        }
        sof_countdown = 0;
        break;
    case SE1:                  //illegal state
        sof_countdown = 0;
        bus_event &= ~doingreset;
        //ReleaseChildren(); FIXME
        usb_task_state = UHS_USB_HOST_STATE_ILLEGAL;
        break;
    case SE0:                  //disconnected
    default:
        sof_countdown = 0;
        bus_event &= ~doingreset;
        //ReleaseChildren(); FIXME
        usb_task_state = UHS_USB_HOST_STATE_IDLE;

        // only one device can be connected and it's currently not
        FreeAddress(1);
        // turn green led off
        iostate = regRd(rIOPINS2) & ~bmGPOUT7;
        regWr(rIOPINS2, iostate);
        break;
    }
    usb_host_speed = speed;
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_INFO)
    uart0_print("-> ");
    uart0_print(_utoh(itoa_buf, usb_task_state));
    uart0_print("\r\n");
#endif
    return;
}

// state machine
void MAX3421_sm(void)
{
    uint8_t x;
    uint8_t iostate;
    if (bus_event & condet) {
        VBUS_changed();
        bus_event &= ~condet;
    }

    switch (usb_task_state) {
    case UHS_USB_HOST_STATE_INITIALIZE:
        // should never happen...
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_WARNING)
        uart0_print("sm init\r\n");
#endif
        busprobe();
        VBUS_changed();
        break;
    case UHS_USB_HOST_STATE_DEBOUNCE:
        sof_countdown = UHS_HOST_DEBOUNCE_DELAY_MS;
        usb_task_state = UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE;
        break;
    case UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE:
        if (!sof_countdown) {
            usb_task_state = UHS_USB_HOST_STATE_RESET_DEVICE;
        }
        break;
    case UHS_USB_HOST_STATE_RESET_DEVICE:
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_WARNING)
        uart0_print("sm rst dev\r\n");
#endif
        bus_event |= busevent;
        usb_task_state = UHS_USB_HOST_STATE_RESET_NOT_COMPLETE;
        regWr(rHIRQ, bmBUSEVENTIRQ);    // see data sheet.
        regWr(rHCTL, bmBUSRST); // issue bus reset
        break;
    case UHS_USB_HOST_STATE_RESET_NOT_COMPLETE:
        if (!busevent)
            usb_task_state = UHS_USB_HOST_STATE_WAIT_BUS_READY;
        break;
    case UHS_USB_HOST_STATE_WAIT_BUS_READY:
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_WARNING)
        uart0_print("sm wait bus rdy\r\n");
#endif
        usb_task_state = UHS_USB_HOST_STATE_CONFIGURING;
        break;                  // don't fall through

    case UHS_USB_HOST_STATE_CONFIGURING:
        usb_task_state = UHS_USB_HOST_STATE_CHECK;
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_WARNING)
        uart0_print("sm conf\r\n");
#endif
        x = configure(0, 1, usb_host_speed);
        usb_error = x;
        if (usb_task_state == UHS_USB_HOST_STATE_CHECK) {
            if (x) {
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_NONE)
                uart0_print("sm err ");
                uart0_print(_utoh(itoa_buf, x));
#endif
                if (x == UHS_HOST_ERROR_JERR) {
                    usb_task_state = UHS_USB_HOST_STATE_IDLE;
                } else if (x != UHS_HOST_ERROR_DEVICE_INIT_INCOMPLETE) {
                    usb_error = x;
                    usb_task_state = UHS_USB_HOST_STATE_ERROR;
                }
            } else {
                usb_task_state = UHS_USB_HOST_STATE_CONFIGURING_DONE;
            }
        }
        break;
    case UHS_USB_HOST_STATE_CHECK:
        break;
    case UHS_USB_HOST_STATE_CONFIGURING_DONE:
        usb_task_state = UHS_USB_HOST_STATE_RUNNING;

        // assert led2 (green)
        iostate = regRd(rIOPINS2) | bmGPOUT7;
        regWr(rIOPINS2, iostate);

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_WARNING)
        uart0_print("config done\r\n");
#endif
        break;
    case UHS_USB_HOST_STATE_RUNNING:
        /*
           Poll_Others();
           for (x = 0;
           (usb_task_state == UHS_USB_HOST_STATE_RUNNING) && (x < UHS_HOST_MAX_INTERFACE_DRIVERS);
           x++) {
           if (devConfig[x]) {
           if (devConfig[x]->bPollEnable)
           devConfig[x]->Poll();
           }
           }
         */
        // fall thru
    default:
        // Do nothing
        break;
    }                           // switch( usb_task_state )
/*
    if (bus_event & condet) {
       VBUS_changed();
       bus_event &= ~condet;
    }
*/
}

static void gpx_irq_handler(const uint16_t msg)
{
    uint8_t iv, ret_iv = 0x0;
    uint8_t iostate;

    gpx_cnt_hl++;

    // assert led1 (red)
    iostate = regRd(rIOPINS2) | bmGPOUT6;
    regWr(rIOPINS2, iostate);

    // read interrupt register from SIE
    iv = regRd(rGPINIRQ);

    if (iv & bmGPINIRQ7) {
        // stop VBUS since MAX4789's FLAG has been asserted
        // FIXME
        // clear interrupt
        ret_iv |= bmGPINIRQ7;
        // red led remains on as warning
    } else {
        // fake activation?
        // deassert red led
        iostate = regRd(rIOPINS2) & ~bmGPOUT6;
        regWr(rIOPINS2, iostate);
    }

    // clear serviced irqs
    regWr(rGPINIRQ, ret_iv);
}

static void int_irq_handler(const uint16_t msg)
{
    uint8_t iv, ret_iv = 0x0;

    int_cnt_hl++;

    iv = regRd(rHIRQ);

    bus_event &= ~counted;

    if (iv & bmBUSEVENTIRQ) {
        //int_cnt_hl++;
        ret_iv |= bmBUSEVENTIRQ;
        if (!(bus_event & doingreset)) {
            bus_event |= condet;
        }
        busprobe();
        bus_event &= ~busevent;
    }

    if (iv & bmCONDETIRQ) {
        if (!(bus_event & doingreset)) {
            bus_event |= condet;
        }
        busprobe();
        ret_iv |= bmCONDETIRQ;
    }

    if (iv & bmFRAMEIRQ) {
        // 1ms SOF PID irq
        if (sof_countdown) {
            sof_countdown--;
            bus_event |= counted;
        }
        bus_event &= ~sofevent;
        ret_iv |= bmFRAMEIRQ;
    }
    // clear serviced irqs
    regWr(rHIRQ, ret_iv);

    if (!sof_countdown && !(bus_event & counted)) {
        MAX3421_sm();
    }
}

void InitEntry(const uint8_t index)
{
    thePool[index].address.devAddress = 0;
    thePool[index].epcount = 1;
    thePool[index].speed = 0;
    for (uint8_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++) {
        thePool[index].epinfo[i] = &dev0ep;
    }
};

uint8_t FindAddressIndex(const uint8_t address)
{
    for (uint8_t i = 1; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++) {
        if (thePool[i].address.devAddress == address)
            return i;
    }
    return 0;
}

// Returns thePool child index for a given parent
uint8_t FindChildIndex(struct UHS_DeviceAddress addr, const uint8_t start)
{
    for (uint8_t i = (start < 1 || start >= UHS_HOST_MAX_INTERFACE_DRIVERS) ? 1 : start;
         i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++) {
        if (thePool[i].address.bmParent == addr.bmAddress)
            return i;
    }
    return 0;
}

// Frees address entry specified by index parameter
void FreeAddressByIndex(const uint8_t index)
{
    // Zero field is reserved and should not be affected
    if (index == 0)
        return;

    struct UHS_DeviceAddress uda = thePool[index].address;
    // If a hub was switched off all port addresses should be freed
    if (uda.bmHub == 1) {
        for (uint8_t i = 1; (i = FindChildIndex(uda, i));)
            FreeAddressByIndex(i);
    }
    InitEntry(index);
}

void InitAllAddresses(void)
{
    for (uint8_t i = 1; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++)
        InitEntry(i);
}

struct UHS_Device *GetUsbDevicePtr(uint8_t addr)
{
    if (!addr)
        return thePool;

    uint8_t index = FindAddressIndex(addr);
    return (!index) ? NULL : &thePool[index];
}

uint8_t AllocAddress(const uint8_t parent, const uint8_t port)
{
    struct UHS_DeviceAddress _parent;
    _parent.devAddress = parent;
    if (_parent.bmReserved || port > 7) {
        return 0;
    }
    // finds first empty address entry starting from one
    uint8_t index = FindAddressIndex(0);

    if (!index) {
        // if empty entry is not found
        return 0;
    }

    struct UHS_DeviceAddress addr;
    addr.devAddress = port;
    addr.bmParent = _parent.bmAddress;

    thePool[index].address = addr;

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_DEBUG)
    uart0_print("* AllocAddress dA ");
    uart0_print(_utoh(itoa_buf, addr.devAddress));
    uart0_print(" bP ");
    uart0_print(_utoh(itoa_buf, addr.bmParent));
    uart0_print(" bA ");
    uart0_print(_utoh(itoa_buf, addr.bmAddress));
    uart0_print("\r\n");
#endif
    return thePool[index].address.devAddress;
}

void FreeAddress(const uint8_t addr)
{
    // if the root hub is disconnected all the addresses should be initialized
    if (addr == 0x41) {
        InitAllAddresses();
        return;
    }
    uint8_t index = FindAddressIndex(addr);
    FreeAddressByIndex(index);
}

void doHostReset(void)
{
    noInterrupts();
    bus_event |= doingreset;
    bus_event |= busevent;
    regWr(rSNDBC, 0);           // FIXME
    regWr(rHIRQ, bmBUSEVENTIRQ | bmFRAMEIRQ);
    // issue bus reset
    regWr(rHCTL, bmBUSRST);
    interrupts();
    while (bus_event & busevent) {
        /*
           uart0_print(_utoh(itoa_buf, regRd(rHIRQ)));
           uart0_print(" ");
           uart0_print(_utoh(itoa_buf, int_cnt_hl));
           uart0_print(" ");
           uart0_print(_utoh(itoa_buf, P1IE));
           uart0_print(" ");
           uart0_print(_utoh(itoa_buf, P1IFG));
           uart0_print("\r\n");
           timer_a0_delay_ccr4(_500ms);
         */
        int_irq_handler(0);
    }
    noInterrupts();
    bus_event |= sofevent;
    interrupts();
    while (bus_event & sofevent) {
        int_irq_handler(0);
    }
    noInterrupts();
    bus_event &= ~doingreset;
    interrupts();
}

uint8_t MAX3421_init(void)
{
    uint16_t i = 0;

    int_cnt = 0;
    int_cnt_hl = 0;
    gpx_cnt = 0;
    gpx_cnt_hl = 0;
    bus_event = 0;
    sof_countdown = 0;
    ifg_int_last_event = 0;
    ifg_gpx_last_event = 0;

    // init address space
    InitEntry(0);
    //thePool[0].epinfo[0] = &dev0ep;
    dev0ep.epAddr = 0;
    dev0ep.maxPktSize = 0x08;
    dev0ep.epAttribs = 0;
    dev0ep.bmNakPower = UHS_USB_NAK_MAX_POWER;
    InitAllAddresses();

    usb_task_state = UHS_USB_HOST_STATE_INITIALIZE;
    //regWr(rPINCTL, bmFDUPSPI);

    // red led on
    regWr(rIOPINS2, bmGPOUT6);

    // remove RST signal
    MAX3421E_RST_OFF;

    // clear old irqs, reset iopins
    regWr(rIOPINS1, 0x0);
    regWr(rUSBIRQ, (bmVBUSIRQ | bmNOVBUSIRQ | bmOSCOKIRQ));
    // detect presence and absence of VBUS
    regWr(rUSBIEN, bmVBUSIE);

    // set full duplex SPI, edge-active INT interrupt
    //regWr(rPINCTL, bmFDUPSPI | GPX_INIRQ);
    regWr(rPINCTL, bmFDUPSPI | GPX_VBDET);

    // MAX4789 is connected to GPOUT0 to controls the VBUS
    // enable VBUS 
    regWr(rIOPINS1, bmGPOUT0);

    // detect if VBUS is up
    while (++i) {
        if ((regRd(rUSBIRQ) & bmVBUSIRQ)) {
            break;
        }
    }
    if (!i) {
        // VBUS voltage did not reach 5v after 2^16 ticks
        return EXIT_FAILURE;
    }
    // reset the chip, verify OSCOKIRQ
    regWr(rUSBCTL, bmCHIPRES);
    regWr(rUSBCTL, 0x00);
    while (++i) {
        if ((regRd(rUSBIRQ) & bmOSCOKIRQ)) {
            regWr(rUSBIRQ, bmOSCOKIRQ);
            break;
        }
    }
    if (!i) {
        // oscilator did not settle after 2^16 ticks
        return EXIT_FAILURE;
    }
    // enable useful interrupts
    regWr(rUSBIRQ, (bmVBUSIRQ | bmNOVBUSIRQ | bmOSCOKIRQ));
    regWr(rUSBIEN, (bmVBUSIE | bmNOVBUSIE));

    // MAX4789 has the FLAG pin tied to GPINIE7
    regWr(rGPINIRQ, 0xff);
    regWr(rGPINIEN, bmGPINIEN7);

    // set host mode
    // set pulldowns for peripheral plugin and speed detection
    regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);

    // peripheral and frame generator interrupt enable
    regWr(rHIEN, bmCONNIE | bmFRAMEIE | bmBUSEVENTIE);

    // check if a peripheral is connected
    regWr(rHCTL, bmSAMPLEBUS);
    while (!(regRd(rHCTL) & bmSAMPLEBUS)) ;     //wait for sample operation to finish

    busprobe();
    VBUS_changed();

    // clear connection detect interrupt
    regWr(rHIRQ, bmCONNIRQ);
    // enable INT pin
    regWr(rCPUCTL, bmIE);

    sys_messagebus_register(&gpx_irq_handler, SYS_MSG_P1IFG_GPX);
    sys_messagebus_register(&int_irq_handler, SYS_MSG_P1IFG_INT);

    // status leds off
    regWr(rIOPINS2, 0);

    return EXIT_SUCCESS;
}

uint8_t MAX3421_getVbusState(void)
{
    return vbusState;
};

void regWr(const uint8_t reg, const uint8_t val)
{
    uint8_t data[2];

    // data[0] is the command byte, data[1] is the data byte
    data[0] = reg | MAX3421_WRITE;
    data[1] = val;

    MAX3421_CS_SELECT;
    spi_send_frame(data, 2);
    MAX3421_CS_DESELECT;
}

uint8_t *bytesWr(const uint8_t reg, const uint8_t nbytes, uint8_t * data_p)
{
    uint8_t data;
    data = reg | MAX3421_WRITE;
    MAX3421_CS_SELECT;
    spi_send_frame(&data, 1);
    spi_send_frame(data_p, nbytes);
    MAX3421_CS_DESELECT;
    return data_p + nbytes;
}

uint8_t regRd(const uint8_t reg)
{
    uint8_t rv = 0;

    MAX3421_CS_SELECT;
    spi_send_frame((uint8_t *) & reg, 1);
    spi_read_frame(&rv, 1);
    MAX3421_CS_DESELECT;

    return rv;
}

uint8_t *bytesRd(const uint8_t reg, const uint8_t nbytes, uint8_t * data_p)
{
    MAX3421_CS_SELECT;
    spi_send_frame((uint8_t *) & reg, 1);
    spi_read_frame(data_p, nbytes);
    MAX3421_CS_DESELECT;
    //data_p += nbytes;
    return (data_p + nbytes);
}

/** @brief  GPIO write
*   @retval void
*/
/*  GPOUT bits are in the low nibble. 0-3 in IOPINS1, 4-7 in IOPINS2   */
void gpioWr(const uint8_t data)
{
    uint8_t pins2;
    regWr(rIOPINS1, data);
    pins2 = data >> 4;
    regWr(rIOPINS2, pins2);
}

/** @brief  Reads the current GPI input values
*   @retval uint8_t Bitwise value of all 8 GPI inputs
*/
/* GPIN pins are in high nibbles of IOPINS1, IOPINS2    */
uint8_t gpioRd(void)
{
    uint8_t gpin = 0;
    gpin = regRd(rIOPINS2);
    gpin &= 0xf0;
    gpin |= (regRd(rIOPINS1) >> 4);
    return (gpin);
}

/** @brief  Reads the current GPI output values
*   @retval uint8_t Bitwise value of all 8 GPI outputs
*/
/* GPOUT pins are in low nibbles of IOPINS1, IOPINS2    */
uint8_t gpioRdOutput(void)
{
    uint8_t gpout = 0;
    gpout = regRd(rIOPINS1);
    gpout &= 0x0f;
    gpout |= (regRd(rIOPINS2) << 4);
    return (gpout);
}

/* probe bus to determine device presence and speed and switch host to this speed */
void busprobe(void)
{
    uint8_t bus_sample;
    uint8_t tmpdata;
    bus_sample = regRd(rHRSL);
    bus_sample &= (bmJSTATUS | bmKSTATUS);
    switch (bus_sample) {
        // start full-speed or low-speed host
    case (bmJSTATUS):
        if ((regRd(rMODE) & bmLOWSPEED) == 0) {
            regWr(rMODE, MODE_FS_HOST);
            vbusState = FSHOST;
        } else {
            regWr(rMODE, MODE_LS_HOST);
            vbusState = LSHOST;
        }
        tmpdata = regRd(rMODE) | bmSOFKAENAB;
        regWr(rHIRQ, bmFRAMEIRQ);
        regWr(rMODE, tmpdata);
        break;
    case (bmKSTATUS):
        if ((regRd(rMODE) & bmLOWSPEED) == 0) {
            regWr(rMODE, MODE_LS_HOST);
            vbusState = LSHOST;
        } else {
            regWr(rMODE, MODE_FS_HOST);
            vbusState = FSHOST;
        }
        // start SOF generation
        tmpdata = regRd(rMODE) | bmSOFKAENAB;
        regWr(rHIRQ, bmFRAMEIRQ);
        regWr(rMODE, tmpdata);
        break;
    case (bmSE1):
        // illegal state
        regWr(rMODE, (bmDPPULLDN | bmDMPULLDN | bmHOST | bmSEPIRQ));
        vbusState = SE1;
        break;
    case (bmSE0):
        // disconnected state
        regWr(rMODE, (bmDPPULLDN | bmDMPULLDN | bmHOST | bmSEPIRQ));
        vbusState = SE0;
        break;
    }
}

// * Initialize the descriptor stream, works much like opening a file.
// *
// * @param ei
// * @param ucd
// * @param pep
// * @param data
// * @param left
// * @param read
// * @param offset
// * @return zero for success or error code

uint8_t initDescrStream(struct ENUMERATION_INFO *ei,
                        USB_CONFIGURATION_DESCRIPTOR * ucd, struct UHS_EpInfo *pep,
                        uint8_t * data, uint16_t * left, uint16_t * read, uint8_t * offset)
{
    if (!ei || !ucd) {
        return UHS_HOST_ERROR_BAD_ARGUMENT;
    }
    if (!pep) {
        return UHS_HOST_ERROR_NULL_EPINFO;
    }
    *left = ucd->wTotalLength;
    *read = 0;
    *offset = 1;
    uint8_t rcode;
    pep->maxPktSize = ei->bMaxPacketSize0;
    rcode = getone(pep, left, read, data, offset);
    return rcode;
}

uint8_t getNextInterface(struct ENUMERATION_INFO * ei, struct UHS_EpInfo * pep, uint8_t data[],
                         uint16_t * left, uint16_t * read, uint8_t * offset)
{
    uint16_t remain;
    uint8_t ty;
    uint8_t rcode = UHS_HOST_ERROR_END_OF_STREAM;
    uint8_t *ptr;
    uint8_t epc = 0;
    ei->interface.numep = 0;
    ei->interface.klass = 0;
    ei->interface.subklass = 0;
    ei->interface.protocol = 0;
    while (*left + *read) {
        remain = data[*offset]; // bLength
        while (remain < 2) {
            rcode = getone(pep, left, read, data, offset);
            if (rcode)
                return rcode;
            remain = data[*offset];
        }
        rcode = getone(pep, left, read, data, offset);
        if (rcode)
            return rcode;
        ty = data[*offset];     // bDescriptorType
        //HOST_DUBUG("bLength: %i ", remain);
        //HOST_DUBUG("bDescriptorType: %2.2x\r\n", ty);
        remain--;
        if (ty == USB_DESCRIPTOR_INTERFACE) {
            //HOST_DUBUG("INTERFACE DESCRIPTOR FOUND\r\n");
            ptr = (uint8_t *) (&(ei->interface.bInterfaceNumber));
            for (int i = 0; i < 6; i++) {
                rcode = getone(pep, left, read, data, offset);
                if (rcode)
                    return rcode;
                *ptr = data[*offset];
                ptr++;
            }
            rcode = getone(pep, left, read, data, offset);
            if (rcode)
                return rcode;
            // Now at iInterface
            // Get endpoints.
            //HOST_DUBUG("Getting %i endpoints\r\n", ei->interface.numep);
            while (epc < ei->interface.numep) {
                rcode = getone(pep, left, read, data, offset);
                if (rcode) {
                    //HOST_DUBUG("ENDPOINT DESCRIPTOR DIED WAY EARLY\r\n");
                    return rcode;
                }
                remain = data[*offset]; // bLength
                while (remain < 2) {
                    rcode = getone(pep, left, read, data, offset);
                    if (rcode)
                        return rcode;
                    remain = data[*offset];
                }
                rcode = getone(pep, left, read, data, offset);
                if (rcode) {
                    //HOST_DUBUG("ENDPOINT DESCRIPTOR DIED EARLY\r\n");
                    return rcode;
                }
                ty = data[*offset];     // bDescriptorType
                //HOST_DUBUG("bLength: %i ", remain);
                //HOST_DUBUG("bDescriptorType: %2.2x\r\n", ty);
                remain -= 2;
                if (ty == USB_DESCRIPTOR_ENDPOINT) {
                    //HOST_DUBUG("ENDPOINT DESCRIPTOR: %i\r\n", epc);
                    ptr = (uint8_t *) (&(ei->interface.epInfo[epc].bEndpointAddress));
                    for (unsigned int i = 0; i < sizeof(struct ENDPOINT_INFO); i++) {
                        rcode = getone(pep, left, read, data, offset);
                        if (rcode) {
                            //HOST_DUBUG("ENDPOINT DESCRIPTOR DIED LATE\r\n");
                            return rcode;
                        }
                        *ptr = data[*offset];
                        ptr++;
                        remain--;
                    }
                    epc++;
                    //HOST_DUBUG("ENDPOINT DESCRIPTOR OK\r\n");
                }
                rcode = eat(pep, left, read, data, offset, &remain);
                if (rcode) {
                    //HOST_DUBUG("ENDPOINT DESCRIPTOR DIED EATING\r\n");
                    return rcode;
                }
                remain = 0;
            }
            remain = 1;
            // queue ahead, but do not report if error.
            rcode = eat(pep, left, read, data, offset, &remain);
            if (!ei->interface.numep && rcode) {
                return rcode;
            }
            //HOST_DUBUG("ENDPOINT DESCRIPTORS FILLED\r\n");
            return 0;
        } else {
            rcode = eat(pep, left, read, data, offset, &remain);
            if (rcode)
                return rcode;
        }
        rcode = UHS_HOST_ERROR_END_OF_STREAM;
    }
    return rcode;
}

// * Receive a packet
// *
// * @param pep pointer to a valid UHS_EpInfo structure
// * @param nak_limit how many NAKs before aborting
// * @param nbytesptr pointer to maximum number of bytes of data to receive
// * @param data pointer to data buffer
// * @return 0 on success

uint8_t InTransfer(struct UHS_EpInfo * pep, const uint16_t nak_limit, uint16_t * nbytesptr,
                   uint8_t * data)
{
    uint8_t rcode = 0;
    uint8_t pktsize;

    uint16_t nbytes = *nbytesptr;
    uint8_t maxpktsize = pep->maxPktSize;

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_DEBUG)
    uart0_print("\r\n* InTransfer ");
    uart0_print(_utoh(itoa_buf, nbytes));
    uart0_print("B\r\n");
#endif

    *nbytesptr = 0;
    regWr(rHCTL, (pep->bmRcvToggle) ? bmRCVTOG1 : bmRCVTOG0);   //set toggle value

    // use a 'break' to exit this loop
    while (1) {
        rcode = dispatchPkt(tokIN, pep->epAddr, nak_limit);     //IN packet to EP-'endpoint'. Function takes care of NAKS.
#if 0
        // This issue should be resolved now.
        if (rcode == UHS_HOST_ERROR_TOGERR) {
            //MAX_HOST_DEBUG("toggle wrong\r\n");
            // yes, we flip it wrong here so that next time it is actually correct!
            pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
            regWr(rHCTL, (pep->bmRcvToggle) ? bmRCVTOG1 : bmRCVTOG0);   //set toggle value
            continue;
        }
#endif
        if (rcode) {
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_FATAL)
            uart0_print("! InTransfer err dP ");
            uart0_print(_utoh(itoa_buf, rcode));
            uart0_print("\r\n");
#endif
            break;              //should be 0, indicating ACK. Else return error code.
        }
        /* check for RCVDAVIRQ and generate error if not present */
        /* the only case when absence of RCVDAVIRQ makes sense is when toggle error occurred. Need to add handling for that */
        if ((regRd(rHIRQ) & bmRCVDAVIRQ) == 0) {
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_FATAL)
            uart0_print("! InTransfer err no RCVDAVIRQ\r\n");
#endif
            rcode = 0xf0;       //receive error
            break;
        }
        pktsize = regRd(rRCVBC);        //number of received bytes


#if (CONFIG_LOG_LVL > LOG_LEVEL_TRACE)
        uart0_print("* InTransfer got ");
        uart0_print(_utoh(itoa_buf, pktsize));
        uart0_print("B\r\n");
#endif

        if (pktsize > nbytes) { //certain devices send more than asked
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_ERROR)
            uart0_print("! InTransfer ");
            uart0_print(_utoh(itoa_buf, nbytes));
            uart0_print("B != ");
            uart0_print(_utoh(itoa_buf, pktsize));
            uart0_print("B\r\n");
#endif
            pktsize = nbytes;
        }

        int16_t mem_left = (int16_t) nbytes - *((int16_t *) nbytesptr);

        if (mem_left < 0)
            mem_left = 0;

        data = bytesRd(rRCVFIFO, ((pktsize > mem_left) ? mem_left : pktsize), data);

        regWr(rHIRQ, bmRCVDAVIRQ);      // Clear the IRQ & free the buffer
        *nbytesptr += pktsize;  // add this packet's byte count to total transfer length

        /* The transfer is complete under two conditions:           */
        /* 1. The device sent a short packet (L.T. maxPacketSize)   */
        /* 2. 'nbytes' have been transferred.                       */
        if ((pktsize < maxpktsize) || (*nbytesptr >= nbytes))   // have we transferred 'nbytes' bytes?
        {
            // Save toggle value
            pep->bmRcvToggle = ((regRd(rHRSL) & bmRCVTOGRD)) ? 1 : 0;
            rcode = 0;
            break;
        }
    }
    return (rcode);
}

// * Set the address of a device to a new address via endpoint Zero.
// *
// * @param oldaddr current address
// * @param newaddr new address
// * @return status of the request, zero is success.

uint8_t setAddr(const uint8_t oldaddr, const uint8_t newaddr)
{
    uint8_t rcode = ctrlReq(oldaddr,
                            mkSETUP_PKT8(UHS_bmREQ_SET, USB_REQUEST_SET_ADDRESS, newaddr, 0x00,
                                         0x0000, 0x0000),
                            0x0000, NULL);
    sof_delay(300);             // Older spec says you should wait at least 200ms
    return rcode;
}

// * Set the configuration for the device to use via endpoint Zero.
// *
// * @param addr Address of the device
// * @param conf_value configuration index value
// * @return status of the request, zero is success.

uint8_t setConf(const uint8_t addr, const uint8_t conf_value)
{
    return (ctrlReq
            (addr,
             mkSETUP_PKT8(UHS_bmREQ_SET, USB_REQUEST_SET_CONFIGURATION, conf_value, 0x00, 0x0000,
                          0x0000), 0x0000, NULL));
}

// * Transmit a packet
// *
// * @param pep pointer to a valid UHS_EpInfo structure
// * @param nak_limit how many NAKs before aborting
// * @param nbytes number of bytes of data to send
// * @param data pointer to data buffer
// * @return 0 on success

uint8_t OutTransfer(struct UHS_EpInfo * pep, const uint16_t nak_limit, const uint16_t nbytes,
                    uint8_t * data)
{
    uint8_t rcode = UHS_HOST_ERROR_NONE;
    uint8_t retry_count;
    uint8_t *data_p = data;     //local copy of the data pointer
    uint16_t bytes_tosend;
    uint16_t nak_count;
    uint16_t bytes_left = nbytes;

    uint8_t maxpktsize = pep->maxPktSize;

    if (maxpktsize < 1 || maxpktsize > 64)
        return UHS_HOST_ERROR_BAD_MAX_PACKET_SIZE;

    unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;

    regWr(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0);   //set toggle value

    while (bytes_left) {
        retry_count = 0;
        nak_count = 0;
        bytes_tosend = (bytes_left >= maxpktsize) ? maxpktsize : bytes_left;
        bytesWr(rSNDFIFO, bytes_tosend, data_p);        //filling output FIFO
        regWr(rSNDBC, bytes_tosend);    //set number of bytes
        regWr(rHXFR, (tokOUT | pep->epAddr));   //dispatch packet
        while (!(regRd(rHIRQ) & bmHXFRDNIRQ)) ; //wait for the completion IRQ
        regWr(rHIRQ, bmHXFRDNIRQ);      //clear IRQ
        rcode = (regRd(rHRSL) & 0x0f);

        while (rcode && ((long)(millis() - timeout) < 0L)) {
            switch (rcode) {
            case UHS_HOST_ERROR_NAK:
                nak_count++;
                if (nak_limit && (nak_count == nak_limit))
                    goto breakout;
                break;
            case UHS_HOST_ERROR_TIMEOUT:
                retry_count++;
                if (retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
                    goto breakout;
                break;
            case UHS_HOST_ERROR_TOGERR:
                // yes, we flip it wrong here so that next time it is actually correct!
                pep->bmSndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                regWr(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0);       //set toggle value
                break;
            default:
                goto breakout;
            }                   //switch( rcode

            /* process NAK according to Host out NAK bug */
            regWr(rSNDBC, 0);
            regWr(rSNDFIFO, *data_p);
            regWr(rSNDBC, bytes_tosend);
            regWr(rHXFR, (tokOUT | pep->epAddr));       //dispatch packet
            while (!(regRd(rHIRQ) & bmHXFRDNIRQ)) ;     //wait for the completion IRQ
            regWr(rHIRQ, bmHXFRDNIRQ);  //clear IRQ
            rcode = (regRd(rHRSL) & 0x0f);
        }                       //while( rcode && ....
        bytes_left -= bytes_tosend;
        data_p += bytes_tosend;
    }                           //while( bytes_left...
 breakout:

    pep->bmSndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 1 : 0;     //bmSNDTOG1 : bmSNDTOG0;  //update toggle
    return (rcode);             //should be 0 in all cases
}

// * Send the actual packet.
// *
// * @param token
// * @param ep Endpoint
// * @param nak_limit how many NAKs before aborting, 0 == exit after timeout
// * @return 0 on success, 0xFF indicates NAK timeout. @see
// *
// Assumes peripheral address is set and relevant buffer is loaded/empty
// If NAK, tries to re-send up to nak_limit times
// If nak_limit == 0, do not count NAKs, exit after timeout
// If bus timeout, re-sends up to USB_RETRY_LIMIT times
// return codes 0x00-0x0f are HRSLT( 0x00 being success ), 0xff means timeout
uint8_t dispatchPkt(const uint8_t token, const uint8_t ep, const uint16_t nak_limit)
{
    unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;
    uint8_t tmpdata;
    uint8_t rcode = UHS_HOST_ERROR_NONE;
    uint8_t retry_count = 0;
    uint16_t nak_count = 0;

    for (;;) {
        // launch the host transfer
        regWr(rHXFR, (token | ep));
        while ((long)(millis() - timeout) < 0L)
        {
            // check if host transfer is complete
            tmpdata = regRd(rHIRQ);
            if (tmpdata & bmHXFRDNIRQ) {
                // clear the interrupt
                regWr(rHIRQ, bmHXFRDNIRQ);
                break;
            }
        }

        // read the HRSLT[0-3] register bits
        // that contain the transfer result
        rcode = (regRd(rHRSL) & 0x0f);

        switch (rcode) {
        case UHS_HOST_ERROR_NAK:
            nak_count++;
            if (nak_limit && (nak_count == nak_limit)) {
                return (rcode);
            }
            timer_a0_delay_ccr4(_200us);
            break;
        case UHS_HOST_ERROR_TIMEOUT:
            retry_count++;
            if (retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM) {
                return (rcode);
            }
            break;
        default:
            return (rcode);
        }
    }
}

struct UHS_EpInfo *getEpInfoEntry(const uint8_t addr, const uint8_t ep)
{
    struct UHS_Device *p = GetUsbDevicePtr(addr);

    if (!p || !p->epinfo)
        return NULL;

    struct UHS_EpInfo *pep;
    for (uint8_t j = 0; j < UHS_HOST_MAX_INTERFACE_DRIVERS; j++) {
        pep = (struct UHS_EpInfo *)(p->epinfo[j]);

        for (uint8_t i = 0; i < p->epcount; i++) {
            if ((pep)->epAddr == ep) {
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_INFO)
                uart0_print("* getEpInfoEntry if ");
                uart0_print(_utoh(itoa_buf, pep->bIface));
                uart0_print(" ep ");
                uart0_print(_utoh(itoa_buf, ep));
                uart0_print("\r\n");
#endif
                return pep;
            }
            pep++;
        }
    }
    return NULL;
}

// * Setup UHS_EpInfo structure
// *
// * @param addr USB device address
// * @param ep Endpoint
// * @param ppep pointer to the pointer to a valid UHS_EpInfo structure
// * @param nak_limit how many NAKs before aborting
// * @return 0 on success

uint8_t SetAddress(const uint8_t addr, const uint8_t ep, struct UHS_EpInfo ** ppep,
                   uint16_t * nak_limit)
{
    uint16_t nak_lim;
    struct UHS_Device *p = GetUsbDevicePtr(addr);

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_WARNING)
    uart0_print("* SetAddress ");
    uart0_print(_utoh(itoa_buf, addr));
    uart0_print("\r\n");
#endif

    if (!p) {
        return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;
    }

    if (!p->epinfo) {
        return UHS_HOST_ERROR_NULL_EPINFO;
    }

    *ppep = getEpInfoEntry(addr, ep);

    if (!*ppep)
        return UHS_HOST_ERROR_NO_ENDPOINT_IN_TABLE;

    nak_lim =
        (0x0001UL <<
         (((*ppep)->bmNakPower >
           UHS_USB_NAK_MAX_POWER) ? UHS_USB_NAK_MAX_POWER : (*ppep)->bmNakPower));
    nak_lim--;
    *nak_limit = nak_lim;

    //timer_a0_delay_ccr4(_1ms);
    regWr(rPERADDR, addr);      //set peripheral address

    uint8_t mode = regRd(rMODE);

    // Set bmLOWSPEED and bmHUBPRE in case of low-speed device, reset them otherwise
    regWr(rMODE, (p->speed) ? mode & ~(bmHUBPRE | bmLOWSPEED) : mode | bmLOWSPEED);

    return 0;
}

struct UHS_EpInfo *ctrlReqOpen(const uint8_t addr, const uint64_t Request, uint8_t * dataptr)
{
    uint8_t rcode;
    struct UHS_EpInfo *pep = NULL;
    uint16_t nak_limit = 0;

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_WARNING)
    uart0_print("* ctrlReqOpen  ad ");
    uart0_print(_utoh(itoa_buf, addr));
    uart0_print(" rq ");
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_INFO)
    uint8_t i;
    uint8_t *req = (uint8_t *) & Request;

    for (i = 0; i < 8; i++) {
        uart0_print(_utoh(itoa_buf, req[i]));
        uart0_print(" ");
    }
#endif
    uart0_print("\r\n");
#endif

    rcode = SetAddress(addr, 0, &pep, &nak_limit);

    if (!rcode) {

        bytesWr(rSUDFIFO, 8, (uint8_t *) (&Request));   //transfer to setup packet FIFO

        rcode = dispatchPkt(tokSETUP, 0, nak_limit);    //dispatch packet
        if (!rcode) {
            if (dataptr != NULL) {
                if (((Request) /* bmReqType */ &0x80) == 0x80) {
                    pep->bmRcvToggle = 1;       //bmRCVTOG1;
                } else {
                    pep->bmSndToggle = 1;       //bmSNDTOG1;
                }
            }
        } else {
            pep = NULL;
        }
    }
    return pep;
}

uint8_t ctrlReqRead(struct UHS_EpInfo * pep, uint16_t * left, uint16_t * read,
                    const uint16_t nbytes, uint8_t * dataptr)
{
    *read = 0;
    uint16_t nak_limit = 0;

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_INFO)
    uart0_print("* ctrlReqRead ");
    uart0_print(_utoh(itoa_buf, *left));
#endif

    if (*left) {
 again:
        *read = nbytes;
        uint8_t rcode = InTransfer(pep, nak_limit, read, dataptr);
        if (rcode == UHS_HOST_ERROR_TOGERR) {
            // yes, we flip it wrong here so that next time it is actually correct!
            pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
            goto again;
        }

        if (rcode) {
            //MAX_HOST_DEBUG("ctrlReqRead ERROR: %2.2x, left: %i, read %i\r\n", rcode, *left, *read);
            return rcode;
        }
        *left -= *read;
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_DEBUG)
        uart0_print(" left ");
        uart0_print(_utoh(itoa_buf, *left));
        uart0_print(" read ");
        uart0_print(_utoh(itoa_buf, *read));
        uart0_print("\r\n");
#endif
    }
    return 0;
}

uint8_t ctrlReqClose(struct UHS_EpInfo * pep, const uint8_t bmReqType, uint16_t left,
                     const uint16_t nbytes, uint8_t * dataptr)
{
    uint8_t rcode = 0;

    //Serial.println("Closing");
    //Serial.flush();
    if (((bmReqType & 0x80) == 0x80) && pep && left && dataptr) {
        //Serial.println("Drain");
        //MAX_HOST_DEBUG("ctrlReqRead Sinking %i\r\n", left);
        // If reading, sink the rest of the data.
        while (left) {
            uint16_t read = nbytes;
            rcode = InTransfer(pep, 0, &read, dataptr);
            if (rcode == UHS_HOST_ERROR_TOGERR) {
                // yes, we flip it wrong here so that next time it is actually correct!
                pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                continue;
            }
            if (rcode)
                break;
            left -= read;
            if (read < nbytes)
                break;
        }
        //        } else {
        //                Serial.println("Nothing to drain");
    }
    if (!rcode) {
        //               Serial.println("Dispatching");
        rcode = dispatchPkt(((bmReqType & 0x80) == 0x80) ? tokOUTHS : tokINHS, 0, 0);   //GET if direction
        //        } else {
        //                Serial.println("Bypassed Dispatch");
    }
    return rcode;
}

uint8_t getone(struct UHS_EpInfo * pep, uint16_t * left, uint16_t * read, uint8_t * dataptr,
               uint8_t * offset)
{
    uint8_t rcode = 0;
    *offset += 1;
    if (*offset < *read) {
        return 0;
    } else if (*left > 0) {
        // uint16_t num = *left;
        uint16_t num = pep->maxPktSize;
        if (num > *left)
            num = *left;
        *offset = 0;
        rcode = ctrlReqRead(pep, left, read, num, dataptr);
        if (rcode == 0) {
            if (*read == 0) {
                rcode = UHS_HOST_ERROR_END_OF_STREAM;
            } else if (*read < num)
                *left = 0;
        }
    } else {
        rcode = UHS_HOST_ERROR_END_OF_STREAM;
    }
    return rcode;
}

uint8_t eat(struct UHS_EpInfo * pep, uint16_t * left, uint16_t * read, uint8_t * dataptr,
            uint8_t * offset, uint16_t * yum)
{
    uint8_t rcode = 0;
    //HOST_DUBUG("eating %i\r\n", *yum);
    while (*yum) {
        *yum -= 1;
        rcode = getone(pep, left, read, dataptr, offset);
        if (rcode)
            break;
    }
    return rcode;
}

uint8_t ctrlReq(uint8_t addr, uint64_t Request, uint16_t nbytes, uint8_t * dataptr)
{
    uint8_t rcode = 0;

    struct UHS_EpInfo *pep = ctrlReqOpen(addr, Request, dataptr);
    if (!pep) {
        return UHS_HOST_ERROR_NULL_EPINFO;
    }
    uint8_t rt = (uint8_t) (Request & 0xFFU);

    uint16_t left = (uint16_t) (Request >> 48) /*total */ ;
    if (dataptr != NULL) {
        //data stage
        if ((rt & 0x80) == 0x80) {
            //IN transfer
            while (left) {
                // Bytes read into buffer
                uint16_t read = nbytes;
                rcode = ctrlReqRead(pep, &left, &read, nbytes, dataptr);
                if (rcode) {
                    return rcode;
                }
                // Should only be used for GET_DESCRIPTOR USB_DESCRIPTOR_DEVICE
                if (!addr
                    && ((Request & (uint32_t) 0xFF00FF00U) ==
                        (((uint32_t) USB_REQUEST_GET_DESCRIPTOR << 8) |
                         ((uint32_t) USB_DESCRIPTOR_DEVICE << 24)))) {
                    left = 0;
                    break;
                }
            }
        } else {
            // OUT transfer
            rcode = OutTransfer(pep, 0, nbytes, dataptr);
        }
        if (rcode) {
            //return error
            return (rcode);
        }
    }
    // Status stage
    rcode = ctrlReqClose(pep, rt, left, nbytes, dataptr);
    return rcode;
}

// * Gets the device descriptor, or part of it from endpoint Zero.
// *
// * @param addr Address of the device
// * @param nbytes how many bytes to return
// * @param dataptr pointer to the data to return
// * @return status of the request, zero is success.

uint8_t getDevDescr(const uint8_t addr, const uint16_t nbytes, uint8_t * dataptr)
{
    return (ctrlReq
            (addr,
             mkSETUP_PKT8(UHS_bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, 0x00,
                          USB_DESCRIPTOR_DEVICE, 0x0000, nbytes), nbytes, dataptr));
}

// * Gets the config descriptor, or part of it from endpoint Zero.
// *
// * @param addr Address of the device
// * @param nbytes how many bytes to return
// * @param conf index to descriptor to return
// * @param dataptr ointer to the data to return
// * @return status of the request, zero is success.

uint8_t getConfDescr(const uint8_t addr, const uint16_t nbytes, const uint8_t conf,
                     uint8_t * dataptr)
{
    return (ctrlReq
            (addr,
             mkSETUP_PKT8(UHS_bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf,
                          USB_DESCRIPTOR_CONFIGURATION, 0x0000, nbytes), nbytes, dataptr));
}

// * Get the string descriptor from a device
// *
// * @param addr Address of the device
// * @param ns
// * @param index
// * @param langid language ID
// * @param dataptr pointer to the data to return
// * @return status of the request, zero is success.

uint8_t getStrDescr(const uint8_t addr, const uint16_t ns, const uint8_t index,
                    const uint16_t langid, uint8_t * dataptr)
{
    return (ctrlReq
            (addr,
             mkSETUP_PKT8(UHS_bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index,
                          USB_DESCRIPTOR_STRING, langid, ns), ns, dataptr));
}

uint8_t sof_delay(const uint16_t x)
{
    uint16_t ticks = x;
    if (!(usb_task_state & UHS_USB_HOST_STATE_MASK))
        return false;
    uint8_t current_state = usb_task_state;
    while (current_state == usb_task_state && ticks--) {
        timer_a0_delay_ccr4(_1ms);
    }
    return (current_state == usb_task_state);
};

//
//set address
//

// * Perform a bus reset to the port of the connected device
// *
// * @param parent index to Parent
// * @param port what port on the parent
// * @param address address of the device
// * @return Zero for success, or error code

uint8_t doSoftReset(const uint8_t parent, const uint8_t port, const uint8_t address)
{
    uint8_t rcode = 0;

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_WARNING)
    uart0_print("* doSoftReset ");
    uart0_print(_utoh(itoa_buf, parent));
    uart0_print(" ");
    uart0_print(_utoh(itoa_buf, port));
    uart0_print(" ");
    uart0_print(_utoh(itoa_buf, address));
    uart0_print("\r\n");
#endif

    if (parent == 0) {
        // Send a bus reset on the root interface.
        doHostReset();
    } else {
        // reset parent port
        //devConfig[parent]->ResetHubPort(port); // FIXME
    }

    //
    // Many devices require a delay before setting the address here...
    // We loop upon fails for up to 2 seconds instead.
    // Most devices will be happy without a retry.
    //
    uint8_t retries = 0;

    if (address) {
        do {
            rcode = setAddr(0, address);
            if (!rcode)
                break;
            retries++;
            sof_delay(10);
        } while (retries < 200);
#if (CONFIG_LOG_LEVEL > LOG_LEVEL_DEBUG)
        uart0_print("* doSoftReset ");
        uart0_print(_utoh(itoa_buf,retries));
        uart0_print(" retries\r\n");
#endif
    }
    return rcode;
}

// DEVICE_DESCRIPTOR size is 18bytes
// CONFIGURATION is 34bytes
// HID report size is 74bytes
//#define MAX_PACKET_LEN  0x12
#define MAX_PACKET_LEN  128

// * enumerates interfaces on devices
// *
// * @param parent index to Parent
// * @param port what port on the parent
// * @param speed the speed of the device
// * @return Zero for success, or error code

uint8_t configure(const uint8_t parent, const uint8_t port, const uint8_t speed)
{
    uint8_t rcode = 0;
    //uint16_t i;

    struct UHS_Device *p = NULL;
    struct ENUMERATION_INFO ei;
    uint8_t buf[MAX_PACKET_LEN];
    uint8_t udd_bNumConfigurations = 0;
    uint8_t ucd_wTotalLength = 0;

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_DEBUG)
    uint8_t i;

    uart0_print("* configuring parent ");
    uart0_print(_utoh(itoa_buf, parent));
    uart0_print(" port ");
    uart0_print(_utoh(itoa_buf, port));
    uart0_print(" speed ");
    uart0_print(_utoh(itoa_buf, speed));
    uart0_print("\r\n");
#endif

    sof_delay(200);
    p = GetUsbDevicePtr(0);
    if (!p) {
        return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;
    }

    p->speed = speed;
    p->epinfo[0][0].maxPktSize = 0x08;

    ///////////////////////////////////////
    // get device descriptor
    //

    memset(buf, 0x00, MAX_PACKET_LEN);
    timer_a0_delay_ccr4(_1ms);
    rcode = getDevDescr(0, MAX_PACKET_LEN, buf);
    // TX 0x80 0x06 0x0 0x01 0x0 0x0 0x12 0x0
    // RX 0x12 0x01 0x10 0x01 0x0 0x0 0x0 0x08 0x6d 0x04 0x14 0xc2 0x05 0x02 0x01 0x02 0x0 0x01
    if (rcode) {
        return UHS_HOST_ERROR_FailGetConfDescr;
    }

    {
        USB_DEVICE_DESCRIPTOR *udd;
        udd = (USB_DEVICE_DESCRIPTOR *) buf;

        ei.vid = udd->idVendor;
        ei.pid = udd->idProduct;
        ei.bcdDevice = udd->bcdDevice;
        ei.klass = udd->bDeviceClass;
        ei.subklass = udd->bDeviceSubClass;
        ei.protocol = udd->bDeviceProtocol;
        ei.bMaxPacketSize0 = udd->bMaxPacketSize0;
        ei.currentconfig = 0;
        ei.parent = parent;
        ei.port = port;
        udd_bNumConfigurations = udd->bNumConfigurations;
    }

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_DEBUG)
    for (i = 0; i < buf[0]; i++) {
        uart0_print(_utoh(itoa_buf, buf[i]));
        uart0_print(" ");
    }
    uart0_print("\r\nbuf ^\r\n");
#endif

    ei.address = AllocAddress(parent, port);
    if (!ei.address) {
        return UHS_HOST_ERROR_ADDRESS_POOL_FULL;
    }

    // shift p to address = 1;
    p = GetUsbDevicePtr(ei.address);
    p->speed = speed;

    //timer_a0_delay_ccr4(_1ms);
    rcode = doSoftReset(parent, port, ei.address);
    if (rcode) {
        FreeAddress(ei.address);
        return rcode;
    }

    if (udd_bNumConfigurations < 1) {
        //uart0_print("err: no interfaces\r\n");
        FreeAddress(ei.address);
        return UHS_HOST_ERROR_DEVICE_NOT_SUPPORTED;
    }

    /////////////////////////////////////////
    // read the minimal config descriptor
    //

    memset(buf, 0x00, MAX_PACKET_LEN);
    timer_a0_delay_ccr4(_1ms);
    rcode = getConfDescr(ei.address, sizeof(USB_CONFIGURATION_DESCRIPTOR), 0, buf);
    // TX 0x80 0x06 0x00 0x02 0x00 0x00 0x09 0x00
    // RX 0x09 0x02 0x22 0x00 0x01 0x01 0x00 0x80 0x0f
    if (rcode) {
        return UHS_HOST_ERROR_FailGetConfDescr;
    }

    {
        USB_CONFIGURATION_DESCRIPTOR *ucd;
        ucd = (USB_CONFIGURATION_DESCRIPTOR *) buf;
        ucd_wTotalLength = ucd->wTotalLength;
    }

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_DEBUG)
    // size is 9
    for (i = 0; i < sizeof(USB_CONFIGURATION_DESCRIPTOR); i++) {
        uart0_print(_utoh(itoa_buf, buf[i]));
        uart0_print(" ");
    }
    uart0_print("\r\nucd ^\r\n");
#endif

    //memset(buf, 0x00, MAX_PACKET_LEN);
    //rcode = getStrDescr(ei.address, 64, 2, 0x0409, data);


    /////////////////////////////////////////
    // read the full config descriptor
    //

    memset(buf, 0x00, MAX_PACKET_LEN);
    timer_a0_delay_ccr4(_1ms);
    rcode = getConfDescr(ei.address, ucd_wTotalLength, 0, buf);
    // TX 0x80 0x06 0x01 0x02 0x00 0x00 0x22 0x00
    // RX 0x09 0x02 0x22 0    0x01 0x01 0    0x80 0x0f \  // CONFIGURATION DESCRIPTOR
    //    0x09 0x04 0    0    0x01 0x03 0    0    0    \  // INTERFACE DESCRIPTOR
    //    0x09 0x21 0x10 0x01 0    0x01 0x22 0x4a 0    \  // HID DESCRIPTOR
    //    0x07 0x05 0x81 0x03 0x05 0    0x0a              // ENDPOINT DESCRIPTOR
    if (rcode) {
        return UHS_HOST_ERROR_FailGetConfDescr;
    }

#if (CONFIG_LOG_LEVEL > LOG_LEVEL_DEBUG)
    // size ucd->wTotalLength aka 34
    for (i = 0; i < 64; i++) {
        uart0_print(_utoh(itoa_buf, buf[i]));
        uart0_print(" ");
    }
    uart0_print("\r\nstr full ^\r\n");
#endif

    rcode = setConf(ei.address, 1);
    // TX 0 0x09 0x01 0 0 0 0 0

    if (rcode) {
        FreeAddress(ei.address);
        return UHS_HOST_ERROR_FailGetConfDescr;
    }

    return 0;
}

uint8_t get_ifg_int_event(void)
{
    return ifg_int_last_event;
}

void rst_ifg_int_event(void)
{
    ifg_int_last_event = 0;
}

uint8_t get_ifg_gpx_event(void)
{
    return ifg_gpx_last_event;
}

void rst_ifg_gpx_event(void)
{
    ifg_gpx_last_event = 0;
}

__attribute__ ((interrupt(PORT1_VECTOR)))
void Port1_ISR(void)
{
    uint16_t iv;
    iv = P1IFG;

    if (iv & INT_TRIG) {
        int_cnt++;
        ifg_int_last_event = 1;
        P1IFG &= ~INT_TRIG;
        __bic_SR_register_on_exit(LPM3_bits);
    } else if (iv & GPX_TRIG) {
        gpx_cnt++;
        ifg_gpx_last_event = 1;
        P1IFG &= ~GPX_TRIG;
        __bic_SR_register_on_exit(LPM3_bits);
    }
}
