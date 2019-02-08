
// C library for the MAX3421E USB peripheral/host controller IC

//   author:          Petre Rodan <2b4eda@subdimension.ro>
//   available from:  https://github.com/rodan/
//   license:         GNU GPLv3

#include <inttypes.h>
#include "max3421.h"
#include "spi.h"
#include "drivers/sys_messagebus.h"
#include "drivers/timer_a0.h"
#include "proj.h"
#include "usb_spec.h"

#ifdef CONFIG_PRINTF
#include "drivers/uart0.h"
#include "drivers/helper.h"
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

DEV_RECORD devtable[USB_NUMDEVICES];

void VBUS_changed(void)
{
    /* modify USB task state because Vbus changed or unknown */
    uint8_t speed = 1;
    uint8_t iostate;

#ifdef CONFIG_PRINTF
    char buf[18];
    uart0_print("state ");
    uart0_print(_utoh(buf, usb_task_state));
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
        
        // turn green led off
        iostate = regRd(rIOPINS2) & ~bmGPOUT7;
        regWr(rIOPINS2, iostate);
        break;
    }
    usb_host_speed = speed;
#ifdef CONFIG_PRINTF
    uart0_print("-> ");
    uart0_print(_utoh(buf, usb_task_state));
    uart0_print("\r\n");
#endif
    return;
}

// state machine
void MAX3421_sm(void)
{
    uint8_t x;
    uint8_t iostate;
#ifdef CONFIG_PRINTF
    char buf[18];
#endif
    if (bus_event & condet) {
        VBUS_changed();
        bus_event &= ~condet;
    }

    switch (usb_task_state) {
    case UHS_USB_HOST_STATE_INITIALIZE:
        // should never happen...
#ifdef CONFIG_PRINTF
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
#ifdef CONFIG_PRINTF
        //uart0_print("sm deb nc\r\n");
#endif
        if (!sof_countdown) {
            usb_task_state = UHS_USB_HOST_STATE_RESET_DEVICE;
        }
        break;
    case UHS_USB_HOST_STATE_RESET_DEVICE:
#ifdef CONFIG_PRINTF
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
#ifdef CONFIG_PRINTF
        uart0_print("sm wait bus rdy\r\n");
#endif
        usb_task_state = UHS_USB_HOST_STATE_CONFIGURING;
        break;                  // don't fall through

    case UHS_USB_HOST_STATE_CONFIGURING:
        usb_task_state = UHS_USB_HOST_STATE_CHECK;
#ifdef CONFIG_PRINTF
        uart0_print("sm conf\r\n");
#endif
        x = configure(0, 1, usb_host_speed);
        usb_error = x;
        if (usb_task_state == UHS_USB_HOST_STATE_CHECK) {
            if (x) {
                uart0_print("Error ");
                uart0_print(_utoh(buf, x));
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

#ifdef CONFIG_PRINTF
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

    //usb_task_state = UHS_USB_HOST_STATE_INITIALIZE;

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
    regWr(rPINCTL, bmFDUPSPI);

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

// dispatch usb packet. Assumes peripheral address is set and relevant buffer is loaded/empty
// If NAK, tries to re-send up to nak_limit times
// If nak_limit == 0, do not count NAKs, exit after timeout
// If bus timeout, re-sends up to USB_RETRY_LIMIT times
// return codes 0x00-0x0f are HRSLT( 0x00 being success ), 0xff means timeout
uint8_t dispatchPkt(const uint8_t token, const uint8_t ep, uint16_t nak_limit)
{
    uint32_t timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;
    uint8_t tmpdata;
    uint8_t rcode = UHS_HOST_ERROR_NONE;
    uint8_t retry_count = 0;
    uint16_t nak_count = 0;

    for (;;) {
        regWr(rHXFR, (token | ep));     //launch the transfer
        while ((int32_t)(millis() - timeout) < 0L) { //wait for transfer completion
            tmpdata = regRd(rHIRQ);

            if (tmpdata & bmHXFRDNIRQ) {
                regWr(rHIRQ, bmHXFRDNIRQ);      //clear the interrupt
                //rcode = 0x00;
                break;
            }
        }

        rcode = (regRd(rHRSL) & 0x0f);  //analyze transfer result

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
            if (retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
                return (rcode);
            break;
        default:
            return (rcode);
        }
    }
}

// OUT transfer to arbitrary endpoint. Assumes PERADDR is set. Handles multiple packets if necessary. Transfers 'nbytes' bytes
// Handles NAK bug per Maxim Application Note 4000 for single buffer transfer
// rcode 0 if no errors. rcode 01-0f is relayed from HRSL
// major part of this function borrowed from code shared by Richard Ibbotson
uint8_t outTransfer(const uint8_t addr, const uint8_t ep, const uint16_t nbytes, uint8_t *data, const uint16_t nak_limit)
{
    uint8_t rcode, retry_count;
    uint8_t *data_p = data;        //local copy of the data pointer
    uint16_t bytes_tosend, nak_count;
    uint16_t bytes_left = nbytes;
    uint8_t maxpktsize = devtable[addr].epinfo[ep].MaxPktSize;
    unsigned long timeout = millis() + USB_XFER_TIMEOUT;

    if (!maxpktsize) {          //todo: move this check close to epinfo init. Make it 1< pktsize <64
        return 0xFE;
    }

    regWr(rHCTL, devtable[addr].epinfo[ep].sndToggle);  //set toggle value
    while (bytes_left) {
        retry_count = 0;
        nak_count = 0;
        bytes_tosend = (bytes_left >= maxpktsize) ? maxpktsize : bytes_left;
        bytesWr(rSNDFIFO, bytes_tosend, data_p);        //filling output FIFO
        regWr(rSNDBC, bytes_tosend);    //set number of bytes    
        regWr(rHXFR, (tokOUT | ep));    //dispatch packet
        while (!(regRd(rHIRQ) & bmHXFRDNIRQ)) ; //wait for the completion IRQ
        regWr(rHIRQ, bmHXFRDNIRQ);      //clear IRQ
        rcode = (regRd(rHRSL) & 0x0f);
        while ((rcode) && (timeout > millis())) {
            switch (rcode) {
            case hrNAK:
                nak_count++;
                if (nak_limit && (nak_count == USB_NAK_LIMIT)) {
                    return (rcode);     //return NAK
                }
                break;
            case hrTIMEOUT:
                retry_count++;
                if (retry_count == USB_RETRY_LIMIT) {
                    return (rcode);     //return TIMEOUT
                }
                break;
            default:
                return (rcode);
            }                   //switch( rcode...
            // process NAK according to Host out NAK bug
            regWr(rSNDBC, 0);
            regWr(rSNDFIFO, *data_p);
            regWr(rSNDBC, bytes_tosend);
            regWr(rHXFR, (tokOUT | ep));        //dispatch packet
            while (!(regRd(rHIRQ) & bmHXFRDNIRQ)) ;     //wait for the completion IRQ
            regWr(rHIRQ, bmHXFRDNIRQ);  //clear IRQ
            rcode = (regRd(rHRSL) & 0x0f);
        }                       //while( rcode && ....
        bytes_left -= bytes_tosend;
        data_p += bytes_tosend;
    }                           //while( bytes_left...
    devtable[addr].epinfo[ep].sndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? bmSNDTOG1 : bmSNDTOG0;  //update toggle
    return (rcode);             //should be 0 in all cases
}

// IN transfer to arbitrary endpoint. Assumes PERADDR is set. Handles multiple packets if necessary. Transfers 'nbytes' bytes.
// Keep sending INs and writes data to memory area pointed by 'data'
// rcode 0 if no errors. rcode 01-0f is relayed from dispatchPkt(). Rcode f0 means RCVDAVIRQ error,
//            fe USB xfer timeout
uint8_t inTransfer(const uint8_t addr, const uint8_t ep, const uint16_t nbytes, uint8_t * data,
                   uint16_t nak_limit)
{
    uint8_t rcode;
    uint8_t pktsize;
    uint8_t maxpktsize = devtable[addr].epinfo[ep].MaxPktSize;
    unsigned int xfrlen = 0;
    char buf[18];
    regWr(rHCTL, devtable[addr].epinfo[ep].rcvToggle);  //set toggle value
    while (1) {                 // use a 'return' to exit this loop
        rcode = dispatchPkt(tokIN, ep, nak_limit);      //IN packet to EP-'endpoint'. Function takes care of NAKS.
        uart0_print(_utoh(buf, rcode));
        uart0_print(" rcode\r\n");

        if (rcode) {
            return (rcode);     //should be 0, indicating ACK. Else return error code.
        }
        // check for RCVDAVIRQ and generate error if not present
        // the only case when absense of RCVDAVIRQ makes sense is when toggle error occured. Need to add handling for that
        if ((regRd(rHIRQ) & bmRCVDAVIRQ) == 0) {
            return (0xf0);      //receive error
        }
        pktsize = regRd(rRCVBC);        //number of received bytes
        uart0_print(_utoa(buf, pktsize));
        uart0_print("\r\n");

        data = bytesRd(rRCVFIFO, pktsize, data);
        regWr(rHIRQ, bmRCVDAVIRQ);      // Clear the IRQ & free the buffer

        xfrlen += pktsize;      // add this packet's byte count to total transfer length
        // The transfer is complete under two conditions:
        // 1. The device sent a short packet (L.T. maxPacketSize)
        // 2. 'nbytes' have been transferred.
        if ((pktsize < maxpktsize) || (xfrlen >= nbytes)) {     // have we transferred 'nbytes' bytes?
            if (regRd(rHRSL) & bmRCVTOGRD) {    //save toggle value
                devtable[addr].epinfo[ep].rcvToggle = bmRCVTOG1;
            } else {
                devtable[addr].epinfo[ep].rcvToggle = bmRCVTOG0;
            }
            return (0);
        }
    }                           //while( 1 )
}

// Control transfer with status stage and no data stage
// Assumed peripheral address is already set
uint8_t ctrlStatus(const uint8_t ep, const uint8_t direction, const uint16_t nak_limit)
{
    uint8_t rcode;
    if (direction) {            //GET
        rcode = dispatchPkt(tokOUTHS, ep, nak_limit);
    } else {
        rcode = dispatchPkt(tokINHS, ep, nak_limit);
    }
    return (rcode);
}

// Control transfer with data stage. Stages 2 and 3 of control transfer. Assumes peripheral address is set and setup packet has been sent
uint8_t ctrlData(const uint8_t addr, const uint8_t ep, const uint16_t nbytes, uint8_t * dataptr,
                 const uint8_t direction, const uint16_t nak_limit)
{
    uint8_t rcode;
    if (direction) {            //IN transfer
        devtable[addr].epinfo[ep].rcvToggle = bmRCVTOG1;
        //uart0_print("in data start\r\n");
        rcode = inTransfer(addr, ep, nbytes, dataptr, nak_limit);
        //uart0_print("after data\r\n");
        return (rcode);
    } else {                    //OUT transfer
        devtable[addr].epinfo[ep].sndToggle = bmSNDTOG1;
        rcode = outTransfer(addr, ep, nbytes, dataptr, nak_limit);
        return (rcode);
    }
}

// Control transfer. Sets address, endpoint, fills control packet with necessary data, dispatches control packet, and initiates bulk IN transfer,
// depending on request. Actual requests are defined as inlines
// return codes:
// 00       =   success
// 01-0f    =   non-zero HRSLT
uint8_t ctrlReq(const uint8_t addr, const uint8_t ep, const uint8_t bmReqType,
                const uint8_t bRequest, const uint8_t wValLo, const uint8_t wValHi, uint16_t wInd,
                uint16_t nbytes, uint8_t * dataptr, uint16_t nak_limit)
{
    uint8_t direction = 0;      //request direction, IN or OUT
    uint8_t rcode;
    struct SETUP_PKT setup_pkt;
#ifdef CONFIG_PRINTF
    char buf[18];
    uint8_t *setup = (uint8_t *) &setup_pkt;
    uint8_t i;
#endif

    regWr(rPERADDR, addr);      //set peripheral address
    if (bmReqType & 0x80) {
        direction = 1;          //determine request direction
    }
    // fill in setup packet
    setup_pkt.bmRequestType = bmReqType;
    setup_pkt.bRequest = bRequest;
    setup_pkt.wVal_u.wValueLo = wValLo;
    setup_pkt.wVal_u.wValueHi = wValHi;
    setup_pkt.wIndex = wInd;
    setup_pkt.wLength = nbytes;
    bytesWr(rSUDFIFO, 8, (uint8_t *) & setup_pkt);      //transfer to setup packet FIFO
#ifdef CONFIG_PRINTF
//    for (i=0; i<9; i++) {
//        uart0_print(_utoh(buf, setup[i] ));
//        uart0_print(" ");
//    }
//    uart0_print("\r\nsetup pkt ^\r\n");
#endif
    rcode = dispatchPkt(tokSETUP, ep, nak_limit);       //dispatch packet
    uart0_print("setup sent\r\n");
    //Serial.println("Setup packet");   //DEBUG
    if (rcode) {                //return HRSLT if not zero
#ifdef CONFIG_PRINTF
        uart0_print("Setup packet error: ");
        uart0_print(_utoh(buf, rcode));
#endif
        return (rcode);
    }
    //Serial.println( direction, HEX ); 
    if (dataptr != NULL) {      //data stage, if present
        rcode = ctrlData(addr, ep, nbytes, dataptr, direction, nak_limit);
    }
    if (rcode) {                //return error
#ifdef CONFIG_PRINTF
        uart0_print("Data packet error: ");
        uart0_print(_utoh(buf, rcode));
#endif
        return (rcode);
    }
    rcode = ctrlStatus(ep, direction, nak_limit);       //status stage
    return (rcode);
}

/**
 * enumerates interfaces on devices
 *
 * @param parent index to Parent
 * @param port what port on the parent
 * @param speed the speed of the device
 * @return Zero for success, or error code
 */
uint8_t configure(const uint8_t parent, const uint8_t port, const uint8_t speed)
{
    uint16_t nak_limit = 3;
    uint8_t rv = 0;
    USB_DEVICE_DESCRIPTOR buf;

    devtable[0].epinfo->MaxPktSize = 8;

    // get device descriptor
    rv = ctrlReq(0, 0, USB_SETUP_DEVICE_TO_HOST, USB_REQUEST_GET_DESCRIPTOR, 0x00,
                 USB_DESCRIPTOR_DEVICE, 0x0000, 8, (uint8_t *) & buf, nak_limit);

    return rv;
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
