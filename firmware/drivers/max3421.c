
// C library for the MAX3421E USB peripheral/host controller IC

//   author:          Petre Rodan <2b4eda@subdimension.ro>
//   available from:  https://github.com/rodan/
//   license:         GNU GPLv3

#include <inttypes.h>
#include "max3421.h"
#include "spi.h"
#include "drivers/sys_messagebus.h"
#include "proj.h"
#include "UHS_host.h"

volatile uint8_t vbusState;
volatile uint16_t sof_countdown;
volatile uint8_t bus_state;

//volatile uint8_t usb_error;
volatile uint8_t usb_task_state;
//volatile uint8_t usb_task_polling_disabled;
volatile uint8_t usb_host_speed;
//volatile uint8_t hub_present;

static void gpx_irq_handler(enum sys_message msg)
{
    uint8_t iv;
    uint8_t iostate;

    gpx_cnt_hl++;

    // signal led1 (red)
    iostate = regRd(rIOPINS2) | bmGPOUT6;
    regWr(rIOPINS2, iostate);
    iv = regRd(rGPINIRQ);

    if (iv & bmGPINIRQ7) {
        // stop VBUS since MAX4789's FLAG has been asserted
        // FIXME
        // clear interrupt
        regWr(rGPINIRQ, bmGPINIRQ7);
    } else {
        // fake activation?
        iostate = regRd(rIOPINS2) & ~bmGPOUT6;
        regWr(rIOPINS2, iostate);
    }
}

static void int_irq_handler(enum sys_message msg)
{
    uint8_t iv, ret_iv = 0x0;
    uint8_t iostate;

    int_cnt_hl++;

    // assert led2 (green)
    iostate = regRd(rIOPINS2) | bmGPOUT7;
    regWr(rIOPINS2, iostate);

    iv = regRd(rHIRQ);

    if (iv & bmCONNIRQ) {
        busprobe();

        // turn green led off
        iostate = regRd(rIOPINS2) & ~bmGPOUT7;
        regWr(rIOPINS2, iostate);

        ret_iv |= bmCONNIRQ;
    }
    if (iv & bmFRAMEIRQ) {
        // 1ms SOF PID irq
        ret_iv |= bmFRAMEIRQ;
    }
    // clear serviced irqs
    regWr(rHIRQ, ret_iv);
}

uint8_t MAX3421_init(void)
{
    uint16_t i = 0;

    int_cnt = 0;
    int_cnt_hl = 0;
    gpx_cnt = 0;
    gpx_cnt_hl = 0;
    bus_state = 0;
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

    // set full duplex SPI, level-active INT interrupt
    //regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL));

    // set full duplex SPI, edge-active INT interrupt
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
    regWr(rHIEN, bmCONNIE | bmFRAMEIE);

    // check if a peripheral is connected
    regWr(rHCTL, bmSAMPLEBUS);
    while (!(regRd(rHCTL) & bmSAMPLEBUS)) ;     //wait for sample operation to finish

    busprobe();

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

void VBUS_changed(void)
{
    /* modify USB task state because Vbus changed or unknown */
    uint8_t speed = 1;
    //printf("\r\n\r\n\r\n\r\nSTATE %2.2x -> ", usb_task_state);
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
        if (!(bus_state & doingreset)) {
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
        bus_state &= ~doingreset;
        //ReleaseChildren(); FIXME
        usb_task_state = UHS_USB_HOST_STATE_ILLEGAL;
        break;
    case SE0:                  //disconnected
    default:
        sof_countdown = 0;
        bus_state &= ~doingreset;
        //ReleaseChildren(); FIXME
        usb_task_state = UHS_USB_HOST_STATE_IDLE;
        break;
    }
    usb_host_speed = speed;
    //printf("0x%2.2x\r\n\r\n\r\n\r\n", usb_task_state);
    return;
}



__attribute__ ((interrupt(PORT1_VECTOR)))
void Port1_ISR(void)
{
    if (P1IFG & TRIG0) {
        gpx_cnt++;
        port1_ifg_gpx_last_event = 1;
        P1IFG &= ~TRIG0;
        LPM2_EXIT;
    } else if (P1IFG & TRIG1) {
        int_cnt++;
        port1_ifg_int_last_event = 1;
        P1IFG &= ~TRIG1;
        LPM2_EXIT;
    }
}
