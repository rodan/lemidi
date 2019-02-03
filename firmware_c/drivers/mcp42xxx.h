#ifndef __MCP42XXX_H__
#define __MCP42XXX_H__

#include <msp430.h>

#define MCP42XXX_WRITE_DATA   0x10
#define MCP42XXX_SHUTDOWN     0x20

//cannot use PXOUT due to gcc querkiness, so feed the addresses instead
//P1OUT == 0x202
//P2OUT == 0x203
//P4OUT == 0x223

// chip select port location
//static const uint16_t CS_OUT[6] = {P4OUT, P4OUT};
static const uint16_t MCP42XXX_CS_OUT[6] = { 0x223, 0x223 };
static const uint8_t MCP42XXX_CS_PORT[6] = { BIT7, BIT6 };

void mcp42_set_pot_ch(const uint8_t mcp42_id, const uint8_t pot_id,
                const uint8_t pot_val);

void mcp42_set_pot(const uint8_t mcp42_id, const uint8_t pot_ch0_val, const uint8_t pot_ch1_val);

void mcp42_shutdown_pot(const uint8_t mcp42_id, const uint8_t pot_id);

#endif
