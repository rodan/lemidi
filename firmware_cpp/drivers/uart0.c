#include <string.h>

#include "uart0.h"

volatile char uart0_rx_buf[UART0_RXBUF_SZ]; // receive buffer
volatile uint8_t uart0_p; // number of characters received, 0 if none
volatile uint8_t uart0_rx_enable;

volatile uint8_t uart0_last_event;

uint8_t uart0_get_event(void)
{
    return uart0_last_event;
}

void uart0_rst_event(void)
{
    uart0_last_event = UART0_EV_NULL;
}

void uart0_set_eol(void)
{
    uart0_p = 0;
    uart0_rx_enable = 1;
}

char *uart0_get_rx_buf(void)
{
    if (uart0_p) {
        return (char *)uart0_rx_buf;
    } else {
        return NULL;
    }
}

// you'll have to initialize/map uart ports in main()
void uart0_init(void)
{
    UCA0CTL1 |= UCSWRST;        // put state machine in reset
    UCA0CTL1 |= UCSSEL_1;       // use ACLK
    UCA0BR0 = 0x03;             // 32kHz/9600=3.41
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS_3 + UCBRF_0;       // modulation UCBRSx=3, UCBRFx=0
    UCA0CTL1 &= ~UCSWRST;       // initialize USCI state machine
    UCA0IE |= UCRXIE;           // enable USCI_A0 RX interrupt
    uart0_p = 0;
    uart0_rx_enable = 1;
}

uint16_t uart0_tx_str(const char *str, const uint16_t size)
{
    uint16_t p = 0;
    while (p < size) {
        while (!(UCA0IFG & UCTXIFG)) ;  // USCI_A0 TX buffer ready?
        UCA0TXBUF = str[p];
        p++;
    }
    return p;
}

uint16_t uart0_print(const char *str)
{
    size_t p = 0;
    size_t size = strlen(str);
    while (p < size) {
        while (!(UCA0IFG & UCTXIFG)) {
        }                       // USCI_A0 TX buffer ready?
        UCA0TXBUF = str[p];
        p++;
    }
    return p;
}

__attribute__ ((interrupt(USCI_A0_VECTOR)))
void USCI_A0_ISR(void)
{
    uint16_t iv = UCA0IV;
    register char rx;
    uint8_t ev = 0;

    // iv is 2 for RXIFG, 4 for TXIFG
    switch (iv) {
    case 2:
        rx = UCA0RXBUF;
        if (uart0_rx_enable && (uart0_p < UART0_RXBUF_SZ-2)) {
            if (rx == 0x0a) {
                return;
            } else if (rx == 0x0d) {
                ev = UART0_EV_RX;
                uart0_rx_buf[uart0_p] = 0;
                uart0_rx_enable = 0;
                _BIC_SR_IRQ(LPM3_bits);
            } else {
                uart0_rx_buf[uart0_p] = rx;
                uart0_p++;
            }
        } else {
            if ((rx == 0x0d) || (rx == 0x0a)) {
                uart0_p = 0;
            }
        }
        break;
    case 4:
        ev = UART0_EV_TX;
        break;
    default:
        break;
    }
    uart0_last_event |= ev;
}
