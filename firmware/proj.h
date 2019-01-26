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

#ifdef MSP430F5510_DEVBOARD
    #define LED_SWITCH      P4OUT ^= BIT7
    #define LED_ON          P4OUT |= BIT7
    #define LED_OFF         P4OUT &= ~BIT7
#endif

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


// USB interrupts
#define TRIG0               BIT0
#define TRIG1               BIT1

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



#endif
