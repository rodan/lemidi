/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as publishe7d by the Free Software
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

#if defined(USB_HOST_SHIELD_H) && !defined(USB_HOST_SHIELD_LOADED)
#define USB_HOST_SHIELD_LOADED

// uncomment to get 'printf' console debugging. NOT FOR UNO!
//#define DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD


void UHS_NI MAX3421E_HOST::gpx_irq_handler(const uint16_t msg)
{
    uint8_t iv, ret_iv = 0x0;
    uint8_t iostate;

    //gpx_cnt_hl++;

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

void UHS_NI MAX3421E_HOST::int_irq_handler(const uint16_t msg)
{
    uint8_t iv, ret_iv = 0x0;
    uint8_t iostate;

    //int_cnt_hl++;

    // assert led2 (green)
    iostate = regRd(rIOPINS2) | bmGPOUT7;
    regWr(rIOPINS2, iostate);

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

    // turn green led off
    iostate = regRd(rIOPINS2) & ~bmGPOUT7;
    regWr(rIOPINS2, iostate);

    // clear serviced irqs
    regWr(rHIRQ, ret_iv);
   
    if (!sof_countdown && !(bus_event & counted)) {
        ISRbottom();
    }
}

/* write single byte into MAX3421e register */
void UHS_NI MAX3421E_HOST::regWr(const uint8_t reg, const uint8_t val)
{
    uint8_t data[2];

    // data[0] is the command byte, data[1] is the data byte
    data[0] = reg | MAX3421_WRITE;
    data[1] = val;

    MAX3421_CS_SELECT;
    spi_send_frame(data, 2);
    MAX3421_CS_DESELECT;
}

/* multiple-byte write                            */

/* returns a pointer to memory position after last written */
uint8_t *UHS_NI MAX3421E_HOST::bytesWr(const uint8_t reg, const uint8_t nbytes, uint8_t * data_p)
{
    uint8_t data;
    data = reg | MAX3421_WRITE;
    MAX3421_CS_SELECT;
    spi_send_frame(&data, 1);
    spi_send_frame(data_p, nbytes);
    MAX3421_CS_DESELECT;
    return data_p + nbytes;

}

/* GPIO write                                           */
/*GPIO byte is split between 2 registers, so two writes are needed to write one byte */

/* GPOUT bits are in the low nibble. 0-3 in IOPINS1, 4-7 in IOPINS2 */
void UHS_NI MAX3421E_HOST::gpioWr(uint8_t data)
{
    uint8_t pins2;
    regWr(rIOPINS1, data);
    pins2 = data >> 4;
    regWr(rIOPINS2, pins2);
}

/* single host register read    */
uint8_t UHS_NI MAX3421E_HOST::regRd(const uint8_t reg)
{
    uint8_t rv = 0;
    MAX3421_CS_SELECT;
    spi_send_frame((uint8_t *) & reg, 1);
    spi_read_frame(&rv, 1);
    MAX3421_CS_DESELECT;
    return rv;
}

/* multiple-byte register read  */

/* returns a pointer to a memory position after last read   */
uint8_t *UHS_NI MAX3421E_HOST::bytesRd(const uint8_t reg, const uint8_t nbytes, uint8_t * data_p)
{
    MAX3421_CS_SELECT;
    spi_send_frame((uint8_t *) & reg, 1);
    spi_read_frame(data_p, nbytes);
    MAX3421_CS_DESELECT;
    //data_p += nbytes;
    return (data_p + nbytes);
}

/* GPIO read. See gpioWr for explanation */

/* GPIN pins are in high nibbles of IOPINS1, IOPINS2    */
uint8_t UHS_NI MAX3421E_HOST::gpioRd(void)
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
uint8_t UHS_NI MAX3421E_HOST::gpioRdOutput(void)
{
    uint8_t gpout = 0;
    gpout = regRd(rIOPINS1);
    gpout &= 0x0f;
    gpout |= (regRd(rIOPINS2) << 4);
    return (gpout);
}


void UHS_NI MAX3421E_HOST::VBUS_changed(void)
{
    /* modify USB task state because Vbus changed or unknown */
    uint8_t speed = 1;
#ifdef CONFIG_PRINTF
    char str_temp[STR_LEN];
    snprintf(str_temp, STR_LEN, "state 0x%x -> ", usb_task_state );
    uart0_tx_str(str_temp, strlen(str_temp));
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
        //ReleaseChildren(); // FIXME
        usb_task_state = UHS_USB_HOST_STATE_ILLEGAL;
        break;
    case SE0:                  //disconnected
    default:
        sof_countdown = 0;
        bus_event &= ~doingreset;
        //ReleaseChildren(); // FIXME
        usb_task_state = UHS_USB_HOST_STATE_IDLE;
        break;
    }
    usb_host_speed = speed;
#ifdef CONFIG_PRINTF
    snprintf(str_temp, STR_LEN, "0x%x\r\n", usb_task_state );
    uart0_tx_str(str_temp, strlen(str_temp));
#endif
    return;
};

/**
 *  Probe bus to determine device presence and speed,
 *  then switch host to detected speed.
 */
void UHS_NI MAX3421E_HOST::busprobe(void)
{
    uint8_t bus_sample;
    uint8_t tmpdata;
    bus_sample = regRd(rHRSL);  //Get J,K status
    bus_sample &= (bmJSTATUS | bmKSTATUS);      //zero the rest of the byte
    switch (bus_sample) {       //start full-speed or low-speed host
    case (bmJSTATUS):
        // Serial.println("J");
        if ((regRd(rMODE) & bmLOWSPEED) == 0) {
            regWr(rMODE, MODE_FS_HOST); // start full-speed host
            vbusState = FSHOST;
        } else {
            regWr(rMODE, MODE_LS_HOST); // start low-speed host
            vbusState = LSHOST;
        }
        tmpdata = regRd(rMODE) | bmSOFKAENAB;   // start SOF generation
        regWr(rHIRQ, bmFRAMEIRQ);       // see data sheet.
        regWr(rMODE, tmpdata);
        break;
    case (bmKSTATUS):
        // Serial.println("K");
        if ((regRd(rMODE) & bmLOWSPEED) == 0) {
            regWr(rMODE, MODE_LS_HOST); // start low-speed host
            vbusState = LSHOST;
        } else {
            regWr(rMODE, MODE_FS_HOST); // start full-speed host
            vbusState = FSHOST;
        }
        tmpdata = regRd(rMODE) | bmSOFKAENAB;   // start SOF generation
        regWr(rHIRQ, bmFRAMEIRQ);       // see data sheet.
        regWr(rMODE, tmpdata);
        break;
    case (bmSE1):              //illegal state
        // Serial.println("I");
        regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);
        vbusState = SE1;
        // sofevent = false;
        break;
    case (bmSE0):              //disconnected state
        // Serial.println("D");
        regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);
        vbusState = SE0;
        // sofevent = false;
        break;
    }                           //end switch( bus_sample )
}

/**
 * Initialize USB hardware, turn on VBUS
 *
 * @param mseconds Delay energizing VBUS after mseconds, A value of INT16_MIN means no delay.
 * @return 0 on success, -1 on error
 */
int16_t UHS_NI MAX3421E_HOST::Init(int16_t mseconds)
{
    uint16_t i = 0;

    usb_task_state = UHS_USB_HOST_STATE_INITIALIZE;     //set up state machine
    //UHS_printf_HELPER_init();

    int_cnt = 0;
    int_cnt_hl = 0;
    gpx_cnt = 0;
    gpx_cnt_hl = 0;
    bus_event = 0;
    sof_countdown = 0;


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

    busprobe(); // check if anything is connected
    VBUS_changed();

    // clear connection detect interrupt
    regWr(rHIRQ, bmCONNIRQ | bmBUSEVENTIRQ);
    // enable INT pin
    regWr(rCPUCTL, bmIE);

    //sys_messagebus_register(&gpx_irq_handler, SYS_MSG_P1IFG_GPX);
    //sys_messagebus_register(&int_irq_handler, SYS_MSG_P1IFG_INT);

    // status leds off
    regWr(rIOPINS2, 0);

    return 0;
}

/**
 * Setup UHS_EpInfo structure
 *
 * @param addr USB device address
 * @param ep Endpoint
 * @param ppep pointer to the pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::SetAddress(uint8_t addr, uint8_t ep, UHS_EpInfo ** ppep,
                                         uint16_t & nak_limit)
{
    UHS_Device *p = addrPool.GetUsbDevicePtr(addr);

    if (!p)
        return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;

    if (!p->epinfo)
        return UHS_HOST_ERROR_NULL_EPINFO;

    *ppep = getEpInfoEntry(addr, ep);

    if (!*ppep)
        return UHS_HOST_ERROR_NO_ENDPOINT_IN_TABLE;

    nak_limit =
        (0x0001UL <<
         (((*ppep)->bmNakPower >
           UHS_USB_NAK_MAX_POWER) ? UHS_USB_NAK_MAX_POWER : (*ppep)->bmNakPower));
    nak_limit--;
    /*
       USBTRACE2("\r\nAddress: ", addr);
       USBTRACE2(" EP: ", ep);
       USBTRACE2(" NAK Power: ",(*ppep)->bmNakPower);
       USBTRACE2(" NAK Limit: ", nak_limit);
       USBTRACE("\r\n");
     */
    regWr(rPERADDR, addr);      //set peripheral address

    uint8_t mode = regRd(rMODE);

    //Serial.print("\r\nMode: ");
    //Serial.println( mode, HEX);
    //Serial.print("\r\nLS: ");
    //Serial.println(p->speed, HEX);

    // Set bmLOWSPEED and bmHUBPRE in case of low-speed device, reset them otherwise
    regWr(rMODE, (p->speed) ? mode & ~(bmHUBPRE | bmLOWSPEED) : mode | bmLOWSPEED | hub_present);

    return 0;
}

/**
 * Receive a packet
 *
 * @param pep pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @param nbytesptr pointer to maximum number of bytes of data to receive
 * @param data pointer to data buffer
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::InTransfer(UHS_EpInfo * pep, uint16_t nak_limit, uint16_t * nbytesptr,
                                         uint8_t * data)
{
    uint8_t rcode = 0;
    uint8_t pktsize;

    uint16_t nbytes = *nbytesptr;
    MAX_HOST_DEBUG("Requesting %i bytes ", nbytes);
    uint8_t maxpktsize = pep->maxPktSize;

    *nbytesptr = 0;
    regWr(rHCTL, (pep->bmRcvToggle) ? bmRCVTOG1 : bmRCVTOG0);   //set toggle value

    // use a 'break' to exit this loop
    while (1) {
        rcode = dispatchPkt(MAX3421E_tokIN, pep->epAddr, nak_limit);    //IN packet to EP-'endpoint'. Function takes care of NAKS.
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
            //MAX_HOST_DEBUG(">>>>>>>> Problem! dispatchPkt %2.2x\r\n", rcode);
            break;              //should be 0, indicating ACK. Else return error code.
        }
        /* check for RCVDAVIRQ and generate error if not present */
        /* the only case when absence of RCVDAVIRQ makes sense is when toggle error occurred. Need to add handling for that */
        if ((regRd(rHIRQ) & bmRCVDAVIRQ) == 0) {
            //MAX_HOST_DEBUG(">>>>>>>> Problem! NO RCVDAVIRQ!\r\n");
            rcode = 0xf0;       //receive error
            break;
        }
        pktsize = regRd(rRCVBC);        //number of received bytes
        MAX_HOST_DEBUG("Got %i bytes \r\n", pktsize);

        if (pktsize > nbytes) { //certain devices send more than asked
            //MAX_HOST_DEBUG(">>>>>>>> Warning: wanted %i bytes but got %i.\r\n", nbytes, pktsize);
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
            //MAX_HOST_DEBUG("\r\n");
            rcode = 0;
            break;
        }                       // if
    }                           //while( 1 )
    return (rcode);
}

/**
 * Transmit a packet
 *
 * @param pep pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @param nbytes number of bytes of data to send
 * @param data pointer to data buffer
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::OutTransfer(UHS_EpInfo * pep, uint16_t nak_limit, uint16_t nbytes,
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
        regWr(rHXFR, (MAX3421E_tokOUT | pep->epAddr));  //dispatch packet
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
            regWr(rHXFR, (MAX3421E_tokOUT | pep->epAddr));      //dispatch packet
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

/**
 * Send the actual packet.
 *
 * @param token
 * @param ep Endpoint
 * @param nak_limit how many NAKs before aborting, 0 == exit after timeout
 * @return 0 on success, 0xFF indicates NAK timeout. @see
 */
/* Assumes peripheral address is set and relevant buffer is loaded/empty       */
/* If NAK, tries to re-send up to nak_limit times                                                   */
/* If nak_limit == 0, do not count NAKs, exit after timeout                                         */
/* If bus timeout, re-sends up to USB_RETRY_LIMIT times                                             */

/* return codes 0x00-0x0f are HRSLT( 0x00 being success ), 0xff means timeout                       */
uint8_t UHS_NI MAX3421E_HOST::dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit)
{
    unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;
    uint8_t tmpdata;
    uint8_t rcode = UHS_HOST_ERROR_NONE;
    uint8_t retry_count = 0;
    uint16_t nak_count = 0;

    for (;;) {
        regWr(rHXFR, (token | ep));     //launch the transfer
        while ((long)(millis() - timeout) < 0L) //wait for transfer completion
        {
            tmpdata = regRd(rHIRQ);

            if (tmpdata & bmHXFRDNIRQ) {
                regWr(rHIRQ, bmHXFRDNIRQ);      //clear the interrupt
                //rcode = 0x00;
                break;
            }                   //if( tmpdata & bmHXFRDNIRQ

        }                       //while ( millis() < timeout

        rcode = (regRd(rHRSL) & 0x0f);  //analyze transfer result

        switch (rcode) {
        case UHS_HOST_ERROR_NAK:
            nak_count++;
            if (nak_limit && (nak_count == nak_limit))
                return (rcode);
            timer_a0_delay_ccr4(_200us);
            break;
        case UHS_HOST_ERROR_TIMEOUT:
            retry_count++;
            if (retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
                return (rcode);
            break;
        default:
            return (rcode);
        }                       //switch( rcode
    }
}

//
// NULL is error, we don't need to know the reason.
//

UHS_EpInfo *UHS_NI MAX3421E_HOST::ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t * dataptr)
{
    uint8_t rcode;
    UHS_EpInfo *pep = NULL;
    uint16_t nak_limit = 0;
    // MAX_HOST_DEBUG("ctrlReqOpen: addr: 0x%2.2x bmReqType: 0x%2.2x bRequest: 0x%2.2x\r\nctrlReqOpen: wValLo: 0x%2.2x  wValHi: 0x%2.2x wInd: 0x%4.4x total: 0x%4.4x dataptr: 0x%4.4p\r\n", addr, bmReqType, bRequest, wValLo, wValHi, wInd, total, dataptr);
    rcode = SetAddress(addr, 0, &pep, nak_limit);

    if (!rcode) {

        bytesWr(rSUDFIFO, 8, (uint8_t *) (&Request));   //transfer to setup packet FIFO

        rcode = dispatchPkt(MAX3421E_tokSETUP, 0, nak_limit);   //dispatch packet
        if (!rcode) {
            if (dataptr != NULL) {
                if (((Request) /* bmReqType */ &0x80) == 0x80) {
                    pep->bmRcvToggle = 1;       //bmRCVTOG1;
                } else {
                    pep->bmSndToggle = 1;       //bmSNDTOG1;
                }
            }
        } else {
            //                        Serial.println(">>>>>>>>>>>> dispatchPkt Failed <<<<<<<<<<<<<<");
            //                        Serial.println(rcode, HEX);
            //                        Serial.println(bmReqType, HEX);
            //                        Serial.println(bRequest, HEX);
            //                        Serial.println(">>>>>>>>>>>> dispatchPkt Failed <<<<<<<<<<<<<<");
            pep = NULL;
        }
    }
    return pep;
}

uint8_t UHS_NI MAX3421E_HOST::ctrlReqRead(UHS_EpInfo * pep, uint16_t * left, uint16_t * read,
                                          uint16_t nbytes, uint8_t * dataptr)
{
    *read = 0;
    uint16_t nak_limit = 0;
    MAX_HOST_DEBUG("ctrlReqRead left: %i\r\n", *left);
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
            MAX_HOST_DEBUG("ctrlReqRead ERROR: %2.2x, left: %i, read %i\r\n", rcode, *left, *read);
            return rcode;
        }
        *left -= *read;
        MAX_HOST_DEBUG("ctrlReqRead left: %i, read %i\r\n", *left, *read);
    }
    return 0;
}

uint8_t UHS_NI MAX3421E_HOST::ctrlReqClose(UHS_EpInfo * pep, uint8_t bmReqType, uint16_t left,
                                           uint16_t nbytes, uint8_t * dataptr)
{
    uint8_t rcode = 0;

    //Serial.println("Closing");
    //Serial.flush();
    if (((bmReqType & 0x80) == 0x80) && pep && left && dataptr) {
        //Serial.println("Drain");
        MAX_HOST_DEBUG("ctrlReqRead Sinking %i\r\n", left);
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
        rcode = dispatchPkt(((bmReqType & 0x80) == 0x80) ? MAX3421E_tokOUTHS : MAX3421E_tokINHS, 0, 0); //GET if direction
        //        } else {
        //                Serial.println("Bypassed Dispatch");
    }
    return rcode;
}

/**
 * Bottom half of the ISR task
 */
void UHS_NI MAX3421E_HOST::ISRbottom(void)
{
    //uint8_t x;

    if (bus_event & condet) {
        VBUS_changed();
        bus_event &= ~condet;
    }

    switch (usb_task_state) {
    case UHS_USB_HOST_STATE_INITIALIZE:
        // should never happen...
#ifdef CONFIG_PRINTF
        uart0_tx_str((char *)"sm init\r\n", 9);
#endif
        busprobe();
        VBUS_changed();
        break;
    case UHS_USB_HOST_STATE_DEBOUNCE:
#ifdef CONFIG_PRINTF
        uart0_tx_str((char *)"sm debounce\r\n", 13);
#endif
        // This seems to not be needed. The host controller has debounce built in.
        sof_countdown = UHS_HOST_DEBOUNCE_DELAY_MS;
        usb_task_state = UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE;
        break;
    case UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE:
#ifdef CONFIG_PRINTF
        //uart0_tx_str("sm deb nc\r\n", 11);
#endif
        if (!sof_countdown) {
            usb_task_state = UHS_USB_HOST_STATE_RESET_DEVICE;
        }
        break;
    case UHS_USB_HOST_STATE_RESET_DEVICE:
#ifdef CONFIG_PRINTF
        uart0_tx_str((char *)"sm rst dev\r\n", 12);
#endif
        bus_event |= busevent;
        usb_task_state = UHS_USB_HOST_STATE_RESET_NOT_COMPLETE;
        regWr(rHIRQ, bmBUSEVENTIRQ);    // see data sheet.
        regWr(rHCTL, bmBUSRST); // issue bus reset
        break;
    case UHS_USB_HOST_STATE_RESET_NOT_COMPLETE:
#ifdef CONFIG_PRINTF
        //uart0_tx_str("sm rst nc\r\n", 11);
#endif
        if (!busevent)
            usb_task_state = UHS_USB_HOST_STATE_WAIT_BUS_READY;
        break;
    case UHS_USB_HOST_STATE_WAIT_BUS_READY:
#ifdef CONFIG_PRINTF
        uart0_tx_str((char *)"sm wait bus rdy\r\n", 17);
#endif
        usb_task_state = UHS_USB_HOST_STATE_CONFIGURING;
        break;                  // don't fall through

    case UHS_USB_HOST_STATE_CONFIGURING:
        usb_task_state = UHS_USB_HOST_STATE_CHECK;
#ifdef CONFIG_PRINTF
        uart0_tx_str((char *)"sm conf\r\n", 9);
#endif
        /*
           x = Configuring(0, 1, usb_host_speed);
           usb_error = x;
           if (usb_task_state == UHS_USB_HOST_STATE_CHECK) {
           if (x) {
           //_dbg("Error 0x%2.2x", x);
           if (x == UHS_HOST_ERROR_JERR) {
           usb_task_state = UHS_USB_HOST_STATE_IDLE;
           } else if (x != UHS_HOST_ERROR_DEVICE_INIT_INCOMPLETE) {
           usb_error = x;
           usb_task_state = UHS_USB_HOST_STATE_ERROR;
           }
           } else
           usb_task_state = UHS_USB_HOST_STATE_CONFIGURING_DONE;
           }
         */
        break;
    case UHS_USB_HOST_STATE_CHECK:
        // Serial.println((uint32_t)__builtin_return_address(0), HEX);
        break;
    case UHS_USB_HOST_STATE_CONFIGURING_DONE:
        usb_task_state = UHS_USB_HOST_STATE_RUNNING;
        break;
    case UHS_USB_HOST_STATE_RUNNING:
#ifdef CONFIG_PRINTF
        uart0_tx_str((char *)"sm running\r\n", 12);
#endif
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

uint8_t UHS_NI MAX3421E_HOST::get_ifg_int_event(void)
{
    return ifg_int_last_event;
}

void UHS_NI MAX3421E_HOST::rst_ifg_int_event(void)
{
    ifg_int_last_event = 0;
}

uint8_t UHS_NI MAX3421E_HOST::get_ifg_gpx_event(void)
{
    return ifg_gpx_last_event;
}

void UHS_NI MAX3421E_HOST::rst_ifg_gpx_event(void)
{
    ifg_gpx_last_event = 0;
}

void UHS_NI MAX3421E_HOST::Task(void)
{
}

#if 0
DDSB();
#endif
#else
#error "Never include USB_HOST_SHIELD_INLINE.h, include UHS_host.h instead"
#endif
