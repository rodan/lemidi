
//  GPS/GPRS tracking system based on a MSP430F5510 uC
//
//  author:          Petre Rodan <2b4eda@subdimension.ro>
//  available from:  https://github.com/rodan/
//  license:         GNU GPLv3

#include <stdio.h>
#include <string.h>

#include "proj.h"
#include "drivers/sys_messagebus.h"
#include "drivers/timer_a0.h"
#include "drivers/uart0.h"
#include "drivers/spi.h"
#include "drivers/mcp42xxx.h"
#include "drivers/max3421.h"
#include "qa.h"

static void parse_UI(const uint16_t msg)
{
    parse_user_input();
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

int main(void)
{
    main_init();
    timer_a0_init();
    uart0_init();
    spi_init();
    //spi_fast_mode();
    display_menu();

    sys_messagebus_register(&timer_a0_ovf_irq, SYS_MSG_TIMER0_IFG);
    sys_messagebus_register(&parse_UI, SYS_MSG_UART0_RX);

    MAX3421_init();

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
    if (get_ifg_int_event()) {
        msg |= SYS_MSG_P1IFG_INT;
        rst_ifg_int_event();
    }

    // USB GPX event
    if (get_ifg_gpx_event()) {
        msg |= SYS_MSG_P1IFG_GPX;
        rst_ifg_gpx_event();
    }

    while (p) {
        // notify listener if he registered for any of these messages
        if (msg & p->listens) {
            p->fn(msg);
        }
        p = p->next;
    }
}

