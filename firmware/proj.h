#ifndef __PROJ_H__
#define __PROJ_H__

#include <msp430.h>
#include <stdlib.h>
#include <inttypes.h>
#include "config.h"

// msp430's stdlib.h is missing these
#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS    0
#endif
#ifndef EXIT_FAILURE
#define EXIT_FAILURE    1
#endif

#define true            1
#define false           0

#define           SYS_MSG_NULL 0
#define    SYS_MSG_TIMER0_CRR0 0x1
#define    SYS_MSG_TIMER0_CRR1 0x2   // timer_a0_delay_noblk_ccr1
#define    SYS_MSG_TIMER0_CRR2 0x4   // timer_a0_delay_noblk_ccr2
#define    SYS_MSG_TIMER0_CRR3 0x8   // timer_a0_delay_noblk_ccr3
#define    SYS_MSG_TIMER0_CRR4 0x10
#define     SYS_MSG_TIMER0_IFG 0x20  // timer_a0 overflow
    // UARTs
#define       SYS_MSG_UART0_RX 0x40
    // interrupts
#define      SYS_MSG_P1IFG_GPX 0x80  // port1 interrupt
#define      SYS_MSG_P1IFG_INT 0x100 // port1 interrupt
    // RTC
#define     SYS_MSG_RTC_SECOND 0x200 // second event from the hardware RTC

#define b0_off      P1OUT &= ~BIT7
#define b0_on       P1OUT |= BIT7
#define b0_switch   P1OUT ^= BIT7

#define b1_off      P1OUT &= ~BIT6
#define b1_on       P1OUT |= BIT6
#define b1_switch   P1OUT ^= BIT6

#define b2_off      P1OUT &= ~BIT5
#define b2_on       P1OUT |= BIT5
#define b2_switch   P1OUT ^= BIT5

#define b3_off      P1OUT &= ~BIT4
#define b3_on       P1OUT |= BIT4
#define b3_switch   P1OUT ^= BIT4

// customize the chip select pin for your environment
#define MAX3421_CS_SELECT   P4OUT &= ~BIT0
#define MAX3421_CS_DESELECT  P4OUT |= BIT0

// MAX3421E interrupts
#define GPX_TRIG           BIT0
#define INT_TRIG           BIT1

#define MAX3421E_RST_ON     P6OUT &= ~BIT1
#define MAX3421E_RST_OFF    P6OUT |= BIT1

#define STR_LEN 64
char str_temp[STR_LEN];

struct controller_t {
    uint8_t b[4];
    uint8_t j0[2];
    uint8_t j1[2];
};

struct controller_t ctrl;

void main_init(void);
void check_events(void);
void noInterrupts(void);
void interrupts(void);

#endif
