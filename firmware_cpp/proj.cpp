
//  GPS/GPRS tracking system based on a MSP430F5510 uC
//
//  author:          Petre Rodan <2b4eda@subdimension.ro>
//  available from:  https://github.com/rodan/
//  license:         GNU GPLv3

// Load the USB Host System core
#define LOAD_USB_HOST_SYSTEM
// Load USB Host Shield
#define LOAD_USB_HOST_SHIELD
// Use USB hub, you might need this even for a combo dongle.
//#define LOAD_UHS_HUB

// Patch printf so we can use it.
#define LOAD_UHS_PRINTF_HELPER 0
#define DEBUG_PRINTF_EXTRA_HUGE 0
#define DEBUG_PRINTF_EXTRA_HUGE_UHS_HOST 0
#define DEBUG_PRINTF_EXTRA_HUGE_USB_HID 0

#define LOAD_UHS_HID
#define LOAD_UHS_HIDRAWBOOT_KEYBOARD
#define LOAD_UHS_HIDRAWBOOT_MOUSE

#include <stdio.h>
#include <string.h>

#include "proj.h"
#include "drivers/sys_messagebus.h"
#include "drivers/timer_a0.h"
#include "drivers/uart0.h"
#include "drivers/spi.h"
#include "drivers/helper.h"
#include "drivers/mcp42xxx.h"
#include "version.h"
#include "UHS_host.h"

/*
class myHID_processor:public UHS_HID_PROCESSOR {
 public:

    myHID_processor(void) {
    } void onRelease(UHS_HID_base * d) {
        //printf_P("HID device unplugged driver type %d no longer available.\r\n", d->driver);
    }

    void onStart(UHS_HID_base * d) {
        //printf_P("HID driver type %d started, Subclass %02x, Protocol %02x ", d->driver,
        //         d->parent->bSubClass, d->parent->bProtocol);
        switch (d->driver) {
        case UHS_HID_raw:
            //printf_P(PSTR("HID-RAW"));
            break;
        case UHS_HID_mouse:
            //printf_P(PSTR("HIDBOOT-RAW-MOUSE"));
            break;
        case UHS_HID_keyboard:
            //printf_P(PSTR("HIDBOOT-RAW-KEYBOARD"));
            // This twinkles the LEDs a few times as an example.
            for (uint8_t i = 0, led = 0x40U, rv = 0; i < 10; i++) {
                while (led) {
                    led >>= 1;
                    rv = ((UHS_HIDBOOT_keyboard *) d)->SetLEDs(led);
                    if (rv != 0)
                        return; // skip onStart if unplugged.
                    if (!d->parent->UHS_SLEEP_MS(100))
                        return; // skip remainder if unplugged.
                }
            }

            break;
        default:
            //printf_P(PSTR("HID-NOT_USED"));
            break;
        }
        //printf_P(PSTR("\r\n"));
    }

    void onPoll(UHS_HID_base * d, uint8_t * data, uint16_t length) {
        MOUSEINFO *squeek = (MOUSEINFO *) data;
        switch (d->driver) {
        case UHS_HID_raw:
//            printf_P(PSTR("RAW input %d bytes interface %d, Subclass %02x, Protocol %02x Data:"),
//                     length, d->parent->bIface, d->parent->bSubClass, d->parent->bProtocol);
//            for (uint8_t i = 0; i < length; i++) {
//                printf_P(PSTR(" %02x"), data[i]);
//            }
            break;
        case UHS_HID_mouse:
//            printf_P(PSTR
//                     ("Mouse buttons left %s right %s mid %s fourth %s fifth %s motion (X,Y) %4d,%4d wheel %4d"),
//                     squeek->bmLeftButton == 1 ? "t" : "f", squeek->bmRightButton == 1 ? "t" : "f",
//                     squeek->bmMiddleButton == 1 ? "t" : "f", squeek->bmButton4 == 1 ? "t" : "f",
//                     squeek->bmButton5 == 1 ? "t" : "f", squeek->dX, squeek->dY, squeek->wheel1);
            break;
        case UHS_HID_keyboard:
//            printf_P(PSTR
//                     ("keyboard input %d bytes interface %d, Subclass %02x, Protocol %02x Data:"),
//                     length, d->parent->bIface, d->parent->bSubClass, d->parent->bProtocol);
//            for (uint8_t i = 0; i < length; i++) {
//                printf_P(PSTR(" %02x"), data[i]);
//            }
            break;
        default:
            break;
        }
        //printf_P(PSTR("\r\n"));
    }
};
*/

//myHID_processor HID_processor1;
//myHID_processor HID_processor2;
MAX3421E_HOST UHS_Usb;
//UHS_USBHub hub_1(&UHS_Usb);
//UHS_HID hid1(&UHS_Usb, &HID_processor1);
//UHS_HID hid2(&UHS_Usb, &HID_processor2);

void main_init(void)
{

    // watchdog triggers after 4 minutes when not cleared
#ifdef USE_WATCHDOG
    WDTCTL = WDTPW + WDTIS__8192K + WDTSSEL__ACLK + WDTCNTCL;
#else
    WDTCTL = WDTPW + WDTHOLD;
#endif
    //SetVCore(3);

    // ports - consult pinout.ods
    P1SEL = 0;
    //P1OUT = 0x3;
    P1OUT = 0x0;
    P1DIR = 0xfc;
    //P1REN = 0x3;
    P1REN = 0x0;

    // IRQ triggers on a high to low transition
    P1IES |= INT_TRIG | GPX_TRIG;
    // Reset IRQ flags
    P1IFG &= ~(INT_TRIG + GPX_TRIG);
    // Enable interrupts
    P1IE |= INT_TRIG | GPX_TRIG;

    P2SEL = 0;
    P2OUT = 0;
    P2DIR = 0xff;
    P2REN = 0;

    P3SEL = 0;
    P3OUT = 0;
    P3DIR = 0x1f;
    P3REN = 0;

    PMAPPWD = 0x02D52;
    // set up port mappings
    P4MAP4 = PM_UCA0TXD;
    P4MAP5 = PM_UCA0RXD;
    PMAPPWD = 0;

    P4SEL = 0x3e;
    P4OUT = 0xc1;
    P4DIR = 0xc1;
    P4REN = 0;

    P5SEL = 0xc;
    P5OUT = 0;
    P5DIR = 0x33;

    P6SEL = 0;
    P6OUT = 0;
    P6DIR = 0xff;
    P6REN = 0x0;

    //PJOUT = 0x00;
    //PJDIR = 0xFF;

    // disable VUSB LDO and SLDO
    USBKEYPID = 0x9628;
    USBPWRCTL &= ~(SLDOEN + VUSBEN);
    USBKEYPID = 0x9600;

    // crystals
#ifdef USE_XT1
    // enable external LF crystal if one is present
    P5SEL |= BIT4+BIT5;
    UCSCTL6 &= ~XT1OFF;
    // Loop until XT1 stabilizes
    do {
        UCSCTL7 &= ~XT1LFOFFG;                 // clear XT1 fault flags
        SFRIFG1 &= ~OFIFG;                     // clear fault flags
    } while ( UCSCTL7 & XT1LFOFFG );           // test XT1 fault flag
    UCSCTL6 &= ~(XT1DRIVE0 + XT1DRIVE1);
    UCSCTL4 |= SELA__XT1CLK;
#else
    // use internal 32768 Hz oscillator
    UCSCTL4 |= SELA__REFOCLK;
#endif

#ifdef USE_XT2
    // enable HF crystal if one is present
    P5SEL |= BIT2+BIT3;                       // port select XT2
    UCSCTL6 &= ~XT2OFF;                       // set XT2 On
    // Loop until XT2 & DCO stabilizes
    do {
        UCSCTL7 &= ~(XT2OFFG + DCOFFG);        // clear XT2, DCO fault flags
        SFRIFG1 &= ~OFIFG;                     // clear fault flags
    } while ( UCSCTL7 & (XT2OFFG + DCOFFG) );  // test fault flags

    UCSCTL6 &= ~XT2DRIVE1;                      // Decrease XT2 drive
    UCSCTL4 |= SELS__XT2CLK + SELM__XT2CLK;
#endif

}

/*
void display_menu(void)
{
    char str_temp[STR_LEN];

    snprintf(str_temp, STR_LEN,
            "\r\n --- lemidi build #%d\r\n  available commands:\r\n", BUILD);
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m?\e[0m                - show menu\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1mb[0-3] [on/off]\e[0m  - button [0-3] on/off\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1mj[0-1] [0-255]\e[0m   - set joystick wiper\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1mstat\e[0m             - system status\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));
}
*/

/*
void parse_user_input(void)
{
    char *in = uart0_get_rx_buf();
    char d = in[0];
    char id = in[1];

    uint16_t inum;
    uint8_t reg[32];
    uint8_t i;

    char str_temp[STR_LEN];

    if (d == '?') {
        display_menu();
    } else if (d == 'b') {
        if (id == '0') {
            if (strstr(in, "on")) {
                b0_on;
            } else if (strstr(in, "off")) {
                b0_off;
            }
        } else if (id == '1') {
            if (strstr(in, "on")) {
                b1_on;
            } else if (strstr(in, "off")) {
                b1_off;
            }
        } else if (id == '2') {
            if (strstr(in, "on")) {
                b2_on;
            } else if (strstr(in, "off")) {
                b2_off;
            }
        } else if (id == '3') {
            if (strstr(in, "on")) {
                b3_on;
            } else if (strstr(in, "off")) {
                b3_off;
            }
        }
    } else if (d == 'j') {
        struct controller_t {
            uint8_t b[4];
            uint8_t j0[2];
            uint8_t j1[2];
        };

        struct controller_t ctrl;

        if (id == '0') {
            if (str_to_uint16(in, &inum, 1, strlen(in), 0, 255) == EXIT_SUCCESS) {
                ctrl.j0[0] = inum;
                ctrl.j0[1] = inum;
                //mcp42_set_pot( 0, ctrl.j1[0], ctrl.j1[1]);
                mcp42_set_pot_ch( 0, 0, ctrl.j0[0]);
                mcp42_set_pot_ch( 0, 1, ctrl.j0[1]);
                //snprintf(str_temp, STR_LEN, "j0 0x%x 0x%x\r\n", ctrl.j0[0], ctrl.j0[1] );
                //uart0_tx_str(str_temp, strlen(str_temp));
            }

        } else if (id == '1') {
            if (str_to_uint16(in, &inum, 1, strlen(in), 0, 255) == EXIT_SUCCESS) {
                ctrl.j1[0] = inum;
                ctrl.j1[1] = inum;
                //mcp42_set_pot( 0, ctrl.j1[0], ctrl.j1[1]);
                mcp42_set_pot_ch( 1, 0, ctrl.j1[0]);
                mcp42_set_pot_ch( 1, 1, ctrl.j1[1]);
                //snprintf(str_temp, STR_LEN, "j1 0x%x 0x%x\r\n", ctrl.j1[0], ctrl.j1[1] );
                //uart0_tx_str(str_temp, strlen(str_temp));
            }
        }
    } else if (d == 'l') {
        if (strstr(in, "1 on")) {
            regWr(rIOPINS2, bmGPOUT6); //status led1 on
        } else if (strstr(in, "2 on")) {
            regWr(rIOPINS2, bmGPOUT7); //status led2 on
        }
        UHS_Usb.regWr(rIOPINS2, 0);
    } else if (d == 'i') {
        //reg = regRd(rIOPINS1);
        //snprintf(str_temp, STR_LEN, "IOPINS1 0x%x\r\n", reg);
        //uart0_tx_str(str_temp, strlen(str_temp));

        //snprintf(str_temp, STR_LEN, "R6 0x%x\r\n", regRd(rRCVBC));
        //uart0_tx_str(str_temp, strlen(str_temp));
        //snprintf(str_temp, STR_LEN, "R7 0x%x\r\n", regRd(rSNDBC));
        //uart0_tx_str(str_temp, strlen(str_temp));

        memset(reg, 0xff, 32);
        UHS_Usb.bytesRd(13<<3, 8, reg);
        for (i=0;i<8;i++) {
            snprintf(str_temp, STR_LEN, "R%u 0x%x\r\n", i+13, reg[i]);
            uart0_tx_str(str_temp, strlen(str_temp));
        }
        UHS_Usb.bytesRd(21<<3, 11, reg);
        for (i=0;i<11;i++) {
            snprintf(str_temp, STR_LEN, "R%u 0x%x\r\n", i+21, reg[i]);
            uart0_tx_str(str_temp, strlen(str_temp));
        }
        snprintf(str_temp, STR_LEN, "vbusState 0x%x\r\n", UHS_Usb.vbusState);
        uart0_tx_str(str_temp, strlen(str_temp));
        //snprintf(str_temp, STR_LEN, "int cnt %lu handled %lu\r\n", int_cnt, int_cnt_hl);
        //uart0_tx_str(str_temp, strlen(str_temp));
        //snprintf(str_temp, STR_LEN, "gpx cnt %lu handled %lu\r\n", gpx_cnt, gpx_cnt_hl);
        //uart0_tx_str(str_temp, strlen(str_temp));
        } else if (strstr(in, "stat")) {
            snprintf(str_temp, STR_LEN, "  fault 0x%x, UCSCTL7 0x%x\r\n", SFRIFG1&OFIFG, UCSCTL7 );
            uart0_tx_str(str_temp, strlen(str_temp));
            snprintf(str_temp, STR_LEN, "  UCSCTL4 0x%x, UCSCTL6 0x%x\r\n", UCSCTL4, UCSCTL6 );
            uart0_tx_str(str_temp, strlen(str_temp));
            UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
            SFRIFG1 &= ~OFIFG;
    }
}
*/

static void parse_UI(const uint16_t msg)
{
    //parse_user_input();
    uart0_set_eol();
}

static void timer_a0_ovf_irq(const uint16_t msg)
{
    //LED_SWITCH;
    /*
    if (timer_a0_ovf >= tfr) {
        if (timer_a0_ovf > 65535 - SLOW_REFRESH_DELAY) {
            return;
        }
        tfr = timer_a0_ovf + SLOW_REFRESH_DELAY;
        get_temperature();
    }
    */
}

void check_events(void)
{
    struct sys_messagebus *p = sys_messagebus_getp();
    uint16_t msg = SYS_MSG_NULL;

    // drivers/timer0a
    if (timer_a0_get_event()) {
        msg |= timer_a0_get_event();
        timer_a0_rst_event();
    }
    // drivers/uart0
    if (uart0_get_event() & UART0_EV_RX) {
        msg |= SYS_MSG_UART0_RX;
        uart0_rst_event();
    }

    // USB INT event
    if (UHS_Usb.get_ifg_int_event()) {
        msg |= SYS_MSG_P1IFG_INT;
        UHS_Usb.rst_ifg_int_event();
    }

    // USB GPX event
    if (UHS_Usb.get_ifg_gpx_event()) {
        msg |= SYS_MSG_P1IFG_GPX;
        UHS_Usb.rst_ifg_gpx_event();
    }

    while (p) {
        // notify listener if he registered for any of these messages
        if (msg & p->listens) {
            p->fn(msg);
        }
        p = p->next;
    }
}

int main(void)
{
    main_init();
    timer_a0_init();
    timer_a0_delay_noblk_ccr3(125); // millis() counter
    uart0_init();
    spi_init();
    //spi_fast_mode();
    //display_menu();

    UHS_Usb.rst_ifg_int_event();
    UHS_Usb.rst_ifg_gpx_event();

    sys_messagebus_register(&timer_a0_ovf_irq, SYS_MSG_TIMER0_IFG);
    sys_messagebus_register(&parse_UI, SYS_MSG_UART0_RX);
    //sys_messagebus_register(&UHS_Usb.gpx_irq_handler, SYS_MSG_P1IFG_GPX);
    //sys_messagebus_register(&UHS_Usb.int_irq_handler, SYS_MSG_P1IFG_INT);

    UHS_Usb.Init();

    // main loop
    while (1) {
        __bis_SR_register(LPM3_bits + GIE);
        //__no_operation();
#ifdef USE_WATCHDOG
        // reset watchdog counter
        WDTCTL = (WDTCTL & 0xff) | WDTPW | WDTCNTCL;
#endif
        check_events();
        check_events();
        check_events();
    }
}


