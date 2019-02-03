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

#ifndef USB_HOST_SHIELD_H
#define USB_HOST_SHIELD_H

#ifdef LOAD_USB_HOST_SHIELD
#include "UHS_max3421e.h"

#if DEBUG_PRINTF_EXTRA_HUGE
#ifdef DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD
#define MAX_HOST_DEBUG(...) printf(__VA_ARGS__)
#else
#define MAX_HOST_DEBUG(...) VOID0
#endif
#else
#define MAX_HOST_DEBUG(...) VOID0
#endif

#define USB_HOST_SHIELD_USE_ISR 1
#define IRQ_IS_EDGE 1

// IRQs used if CPU polls
#define   ENIBITSPOLLED (bmCONDETIE | bmBUSEVENTIE  | bmFRAMEIE)
// IRQs used if CPU is interrupted
#define      ENIBITSISR (bmCONDETIE | bmBUSEVENTIE | bmFRAMEIE /* | bmRCVDAVIRQ | bmSNDBAVIRQ | bmHXFRDNIRQ */ )

#define IRQ_CHECK_MASK (ENIBITSISR & ICLRALLBITS)

#define IRQ_SENSE FALLING

class MAX3421E_HOST:public UHS_USB_HOST_BASE
#if defined(SWI_IRQ_NUM)
, public dyn_SWI
#endif
{
    // TO-DO: move these into the parent class.
    volatile uint16_t sof_countdown;
    volatile uint8_t bus_event;

    volatile uint32_t int_cnt, int_cnt_hl;
    volatile uint32_t gpx_cnt, gpx_cnt_hl;

    volatile uint8_t ifg_int_last_event;
    volatile uint8_t ifg_gpx_last_event;

 public:
    volatile uint8_t vbusState;

     UHS_NI MAX3421E_HOST(void) {
        sof_countdown = 0;
        bus_event = 0;
        hub_present = 0;
    };

/*
        virtual bool UHS_NI sof_delay(uint16_t x) {
                sof_countdown = x;
                while((sof_countdown != 0) && !condet) {

#if !USB_HOST_SHIELD_USE_ISR
                        Task();
#endif
                }
                //                Serial.println("...Wake");
                return (!condet);
        };
*/
    virtual UHS_EpInfo *ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t * dataptr);

    virtual void UHS_NI vbusPower(VBUS_t state) {
        regWr(rPINCTL, bmFDUPSPI | (uint8_t) (state));
    };

    void UHS_NI Task(void);

    virtual uint8_t SetAddress(uint8_t addr, uint8_t ep, UHS_EpInfo ** ppep, uint16_t & nak_limit);
    virtual uint8_t OutTransfer(UHS_EpInfo * pep, uint16_t nak_limit, uint16_t nbytes,
                                uint8_t * data);
    virtual uint8_t InTransfer(UHS_EpInfo * pep, uint16_t nak_limit, uint16_t * nbytesptr,
                               uint8_t * data);
    virtual uint8_t ctrlReqClose(UHS_EpInfo * pep, uint8_t bmReqType, uint16_t left,
                                 uint16_t nbytes, uint8_t * dataptr);
    virtual uint8_t ctrlReqRead(UHS_EpInfo * pep, uint16_t * left, uint16_t * read, uint16_t nbytes,
                                uint8_t * dataptr);
    virtual uint8_t dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit);

    void UHS_NI ReleaseChildren(void) {
        for (uint8_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++)
            if (devConfig[i])
                devConfig[i]->Release();
        hub_present = 0;
    };

    virtual bool IsHub(uint8_t klass) {
        if (klass == UHS_USB_CLASS_HUB) {
            hub_present = bmHUBPRE;
            return true;
        }
        return false;
    };

    virtual void VBUS_changed(void);

    virtual void UHS_NI doHostReset(void) {
#if USB_HOST_SHIELD_USE_ISR
        noInterrupts();
#endif
        bus_event |= doingreset | busevent;
        regWr(rHIRQ, bmBUSEVENTIRQ);    // see data sheet.
        regWr(rHCTL, bmBUSRST); //issue bus reset
#if USB_HOST_SHIELD_USE_ISR
        DDSB();
        // Enable interrupts
        interrupts();
#endif
        while (busevent) {
            DDSB();
        }
#endif
#if USB_HOST_SHIELD_USE_ISR
        // Enable interrupts
        noInterrupts();
#endif
        bus_event |= sofevent;
#if USB_HOST_SHIELD_USE_ISR
        DDSB();
        // Enable interrupts
        interrupts();
#endif
        // Wait for SOF
        while (sofevent) {
        }
#if USB_HOST_SHIELD_USE_ISR
        // Enable interrupts
        noInterrupts();
#endif
        bus_event &= ~doingreset;
#if USB_HOST_SHIELD_USE_ISR
        DDSB();
        // Enable interrupts
        interrupts();
    };

    int16_t UHS_NI Init(int16_t mseconds);

    int16_t UHS_NI Init(void) {
        return Init(INT16_MIN);
    };

    void ISRbottom(void);
    void busprobe(void);

    void gpx_irq_handler(const uint16_t msg);
    void int_irq_handler(const uint16_t msg);

    // MAX3421e specific
    void regWr(const uint8_t reg, const uint8_t data);
    void gpioWr(const uint8_t data);
    uint8_t regRd(const uint8_t reg);
    uint8_t gpioRd(void);
    uint8_t gpioRdOutput(void);
    uint8_t *bytesWr(const uint8_t reg, const uint8_t nbytes, uint8_t * data_p);
    uint8_t *bytesRd(const uint8_t reg, const uint8_t nbytes, uint8_t * data_p);

    uint8_t get_ifg_int_event(void);
    void rst_ifg_int_event(void);
    uint8_t get_ifg_gpx_event(void);
    void rst_ifg_gpx_event(void);

    // ARM/NVIC specific, used to emulate reentrant ISR.
#if defined(SWI_IRQ_NUM)
    void dyn_SWISR(void) {
        ISRbottom();
    };
#endif

};

#if !defined(USB_HOST_SHIELD_LOADED)
#include "USB_HOST_SHIELD_INLINE.h"
#endif

#else
#error "define LOAD_USB_HOST_SHIELD in your sketch, never include USB_HOST_SHIELD.h in a driver."
#endif
#endif                          /* USB_HOST_SHIELD_H */
