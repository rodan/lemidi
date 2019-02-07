#ifndef __UART0_H__
#define __UART0_H__

#ifdef __cplusplus
extern "C" {
#endif

#define  UART0_EV_NULL 0
#define    UART0_EV_RX 0x1
#define    UART0_EV_TX 0x2

#define UART0_RXBUF_SZ 64

void uart0_init();
void uart0_port_init(void);
uint16_t uart0_tx_str(const char *str, const uint16_t size);
uint16_t uart0_print(const char *str);
uint8_t uart0_get_event(void);
void uart0_rst_event(void);
void uart0_set_eol(void);
char *uart0_get_rx_buf(void);

#ifdef __cplusplus
}
#endif

#endif
