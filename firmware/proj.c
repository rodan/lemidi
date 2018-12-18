
//  GPS/GPRS tracking system based on a MSP430F5510 uC
//
//  author:          Petre Rodan <2b4eda@subdimension.ro>
//  available from:  https://github.com/rodan/
//  license:         GNU GPLv3

#include <stdio.h>
#include <string.h>

#include "proj.h"
#include "drivers/sys_messagebus.h"
#include "drivers/rtc.h"
#include "drivers/timer_a0.h"
#include "drivers/uart0.h"
#include "qa.h"


static void parse_UI(enum sys_message msg)
{
    parse_user_input();

    uart0_p = 0;
    uart0_rx_enable = 1;
    LED_OFF;
}

static void schedule(enum sys_message msg)
{
}

int main(void)
{
    main_init();
    rtca_init();
    timer_a0_init();
    uart0_init();

    uart0_tx_str("debug state\r\n", 13);
    display_menu();

    sys_messagebus_register(&schedule, SYS_MSG_RTC_SECOND);
    sys_messagebus_register(&parse_UI, SYS_MSG_UART0_RX);

    // main loop
    while (1) {
        _BIS_SR(LPM3_bits + GIE);
        //wake_up();
#ifdef USE_WATCHDOG
        // reset watchdog counter
        WDTCTL = (WDTCTL & 0xff) | WDTPW | WDTCNTCL;
#endif
        // new messages can be sent from within a check_events() call, so 
        // parse the message linked list multiple times
        check_events();

        // P4.0 and P4.1
        //P4SEL &= ~0x3;
        
        /*
        PMMCTL0_H = 0xA5;
        SVSMHCTL &= ~SVMHE;
        SVSMLCTL &= ~(SVSLE+SVMLE);
        PMMCTL0_H = 0x00;
        */
    }
}

void main_init(void)
{

    // watchdog triggers after 4 minutes when not cleared
#ifdef USE_WATCHDOG
    WDTCTL = WDTPW + WDTIS__8192K + WDTSSEL__ACLK + WDTCNTCL;
#else
    WDTCTL = WDTPW + WDTHOLD;
#endif
    //SetVCore(3);

    // enable LF crystal
    P5SEL |= BIT5 + BIT4;
    UCSCTL6 &= ~(XT1OFF | XT1DRIVE0);

    P1SEL = 0x0;
    //P1DIR = 0x85;
    // make sure CTS is pulled low so the software doesn't get stuck 
    // in case the sim900 is missing - or broken.
    //P1REN = 0x22;
    //P1OUT = 0x2;

    P2SEL = 0x0;
    //P2DIR = 0x1;
    //P2OUT = 0x0;

    P3SEL = 0x0;
    //P3DIR = 0x1f;
    //P3OUT = 0x0;

    //P4DIR = 0x00;
    //P4DIR = 0x3;
    //P4OUT = 0x0;

    PMAPPWD = 0x02D52;
    // set up UART port mappings

    // there is only one debug uart on (P4.1, P4.0)
    // so we either leave out the gps (P4.2, P4.3) or the gprs (P4.4, P4.5) module

    // debug interface
    P4MAP1 = PM_UCA0TXD;
    P4MAP0 = PM_UCA0RXD;
    P4SEL |= 0x3;

#ifdef MSP430F5510_DEVBOARD
    //Initialization of ports (all unused pins as outputs with low-level
    P1REN |= BIT0;                //Enable BUT1 pullup
    P1OUT = 0x01;
    P1DIR = 0xFE;                 //LCD pins are outputs

    // led
    P4DIR |= BIT7;
#endif

    PMAPPWD = 0;

    //P5SEL is set above
    P5DIR = 0xf;
    P5OUT = 0x0;

    //P6SEL = 0xc;
    //P6DIR = 0x3;
    //P6OUT = 0x2;

    //PJDIR = 0xFF;
    //PJOUT = 0x00;

    // disable VUSB LDO and SLDO
    USBKEYPID = 0x9628;
    USBPWRCTL &= ~(SLDOEN + VUSBEN);
    USBKEYPID = 0x9600;

}

void check_events(void)
{
    struct sys_messagebus *p = messagebus;
    enum sys_message msg = 0;

    // drivers/timer0a
    if (timer_a0_last_event) {
        msg |= timer_a0_last_event;
        timer_a0_last_event = 0;
    }
    // drivers/uart0
    if (uart0_last_event & UART0_EV_RX) {
        msg |= BITA;
        uart0_last_event = 0;
    }
    // drivers/rtca
    if (rtca_last_event & RTCA_EV_SECOND) {
        msg |= BITF;
        rtca_last_event = 0;
    }
    while (p) {
        // notify listener if he registered for any of these messages
        if (msg & p->listens) {
            p->fn(msg);
        }
        p = p->next;
    }
}

