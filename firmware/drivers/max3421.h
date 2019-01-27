#ifndef __MAX3421_H__
#define __MAX3421_H__

// customize the chip select pin for your environment
//#define MAX3421_CS_SELECT   P4OUT |= BIT6
//#define MAX3421_CS_DESELECT  P4OUT &= ~BIT6

#include "max3421e.h"

#define MAX3421_WRITE 0x2
#define MAX3421_READ  0x0

uint8_t MAX3421_init(void);

// internals
void regWr(const uint8_t reg, const uint8_t val);
uint8_t regRd(const uint8_t reg);
uint8_t *bytesRd(uint8_t reg, uint8_t nbytes, uint8_t* data_p);

#endif

