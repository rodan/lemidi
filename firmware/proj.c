
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


// usb related interrupts
volatile uint8_t port1_ifg_gpx_last_event;
volatile uint8_t port1_ifg_int_last_event;

static void parse_UI(enum sys_message msg)
{
    parse_user_input();
    uart0_p = 0;
    uart0_rx_enable = 1;
}

static void schedule(enum sys_message msg)
{
}

static void port1_gpx_irq(enum sys_message msg)
{
    LED_SWITCH;
}

static void port1_int_irq(enum sys_message msg)
{
    LED_SWITCH;
}


int main(void)
{
    main_init();
    //rtca_init();
    timer_a0_init();
    uart0_init();
    display_menu();

    //sys_messagebus_register(&schedule, SYS_MSG_RTC_SECOND);
    sys_messagebus_register(&parse_UI, SYS_MSG_UART0_RX);
    sys_messagebus_register(&port1_gpx_irq, SYS_MSG_P1IFG_GPX);
    sys_messagebus_register(&port1_int_irq, SYS_MSG_P1IFG_INT);

    // main loop
    while (1) {
        _BIS_SR(LPM3_bits + GIE);
        __no_operation();
        //wake_up();
#ifdef USE_WATCHDOG
        // reset watchdog counter
        WDTCTL = (WDTCTL & 0xff) | WDTPW | WDTCNTCL;
#endif
        check_events();
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

    // ports - consult pinout.ods
    
    P1SEL = 0;
    P1OUT = 0;
    P1DIR = 0xf0;
    P1REN = 0xc;

    // IRQ triggers on rising edge
    P1IES &= ~(TRIG0 + TRIG1);
    // Reset IRQ flags
    P1IFG &= ~(TRIG0 + TRIG1);
    // Enable interrupts
    P1IE |= TRIG0 + TRIG1;

    P2SEL = 0;
    P2OUT = 0;
    P2DIR = 0;
    P2REN = 0x1;

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
    P6DIR = 0x2;
    P6REN = 0xd;

    //PJOUT = 0x00;
    //PJDIR = 0xFF;

#ifdef MSP430F5510_DEVBOARD
#endif

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

    // USB GPX event
    if (port1_ifg_gpx_last_event) {
        msg |= BITD;
        port1_ifg_gpx_last_event = 0;
    }

    // USB INT event
    if (port1_ifg_int_last_event) {
        msg |= BITE;
        port1_ifg_int_last_event = 0;
    }

    /*
    // drivers/rtca
    if (rtca_last_event & RTCA_EV_SECOND) {
        msg |= BITF;
        rtca_last_event = 0;
    }
    */
    while (p) {
        // notify listener if he registered for any of these messages
        if (msg & p->listens) {
            p->fn(msg);
        }
        p = p->next;
    }
}

__attribute__ ((interrupt(PORT1_VECTOR)))
void Port1_ISR(void)
{
    if (P1IFG & TRIG0) {
        port1_ifg_gpx_last_event = 1;
        P1IFG &= ~TRIG0;
        LPM3_EXIT;
    } else if (P1IFG & TRIG1) {
        port1_ifg_int_last_event = 1;
        P1IFG &= ~TRIG1;
        LPM3_EXIT;
    }
}

