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

// customize the chip select pin for your environment
#define MAX3421_CS_SELECT   P4OUT &= ~BIT6
#define MAX3421_CS_DESELECT  P4OUT |= BIT6


// USB interrupts
#define TRIG0               BIT0
#define TRIG1               BIT1

#define STR_LEN 64
char str_temp[STR_LEN];

void main_init(void);
void check_events(void);

#endif
