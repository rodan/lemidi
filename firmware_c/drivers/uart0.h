#ifndef __UART0_H__
#define __UART0_H__

#include "proj.h"

#define  UART0_EV_NULL 0
#define    UART0_EV_RX 0x1
#define    UART0_EV_TX 0x2

#define UART0_RXBUF_SZ 128

volatile char uart0_rx_buf[UART0_RXBUF_SZ];
volatile uint8_t uart0_p;
volatile uint8_t uart0_rx_enable;
//uint8_t uart0_rx_err;

void uart0_init();
uint16_t uart0_tx_str(char *str, const uint16_t size);

volatile uint8_t uart0_last_event;

#endif
