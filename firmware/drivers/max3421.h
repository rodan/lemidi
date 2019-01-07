#ifndef __MAX3421_H__
#define __MAX3421_H__

// customize the chip select pin for your environment
//#define MAX3421_CS_SELECT   P4OUT |= BIT6
//#define MAX3421_CS_DESELECT  P4OUT &= ~BIT6

#define MAX3421_WRITE 0x2
#define MAX3421_READ  0x0

void max3421_write(const uint8_t reg, const uint8_t val);

#endif

