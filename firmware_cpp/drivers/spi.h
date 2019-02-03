#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern void spi_init(void);
extern void spi_end(void);
extern void spi_fast_mode(void);
extern void spi_read_frame(uint8_t * pBuffer, uint16_t size);
extern void spi_send_frame(uint8_t * pBuffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif

