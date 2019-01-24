// C library for the MCP42010, MCP42050, MCP42100 digital potentiometer ICs

//   author:          Petre Rodan <2b4eda@subdimension.ro>
//   available from:  https://github.com/rodan/
//   license:         GNU GPLv3

#include <inttypes.h>
#include "mcp42xxx.h"
#include "spi.h"

/// set the potentiometer value to one of the channels to the chip
/// identified by 'mcp_id'
/// inputs:
///   mcp_id: [0-1]     - this example has two pot ICs each with their own CS pin
///   pot_id:  [0-1]    - the channel/pot from within the chip
///   pot_val: [0-255]
void mcp42_set_pot_ch(const uint8_t mcp42_id, const uint8_t pot_id,
                const uint8_t pot_val)
{
    uint8_t data[2];
    uint8_t *ptr;

    // data[0] is the command byte, data[1] is the data byte
    data[0] = MCP42XXX_WRITE_DATA | ( pot_id + 1 );
    data[1] = pot_val;

    // select slave
    ptr = (uint8_t *) MCP42XXX_CS_OUT[mcp42_id];
    *ptr &= ~MCP42XXX_CS_PORT[mcp42_id];

    // set volume
    spi_send_frame(data, 2);

    // deselect slave
    *ptr |= MCP42XXX_CS_PORT[mcp42_id];
}

/// set the potentiometer values of both channels of the chip
/// identified by 'mcp_id'
/// inputs:
///   mcp_id: [0-1]     - this example has two pot ICs each with their own CS pin
///   pot_ch0_val: [0-255]    - pot value for the first channel
///   pot_ch1_val: [0-255]    - pot value for the second channel
void mcp42_set_pot(const uint8_t mcp42_id, const uint8_t pot_ch0_val, const uint8_t pot_ch1_val)
{
    uint8_t data[4];
    uint8_t *ptr;

    // data[0] is the command byte, data[1] is the data byte
    data[0] = MCP42XXX_WRITE_DATA | 1;
    data[1] = pot_ch0_val;
    data[2] = MCP42XXX_WRITE_DATA | 2;
    data[3] = pot_ch1_val;

    // select slave
    ptr = (uint8_t *) MCP42XXX_CS_OUT[mcp42_id];
    *ptr &= ~MCP42XXX_CS_PORT[mcp42_id];

    // set wipers
    spi_send_frame(data, 4);

    // deselect slave
    *ptr |= MCP42XXX_CS_PORT[mcp42_id];
}

/// shutdown one of the channels of the chip identified by 'mcp_id'
/// inputs:
///   mcp_id: [0-1]     - this example has two pot ICs each with their own CS pin
///   pot_id:  [0-1]    - the channel/pot from within the chip
///   pot_val: [0-255]
void mcp42_shutdown_pot(const uint8_t mcp42_id, const uint8_t pot_id)
{
    uint8_t data[2];
    uint8_t *ptr;

    // data[0] is the command byte, data[1] is the data byte
    data[0] = MCP42XXX_SHUTDOWN | ( pot_id + 1 );
    data[1] = 0;

    // select slave
    ptr = (uint8_t *) MCP42XXX_CS_OUT[mcp42_id];
    *ptr &= ~MCP42XXX_CS_PORT[mcp42_id];

    // set volume
    spi_send_frame(data, 2);

    // deselect slave
    *ptr |= MCP42XXX_CS_PORT[mcp42_id];
}

