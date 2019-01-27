
// C library for the MAX3421E USB peripheral/host controller IC

//   author:          Petre Rodan <2b4eda@subdimension.ro>
//   available from:  https://github.com/rodan/
//   license:         GNU GPLv3

#include <inttypes.h>
#include "max3421.h"
#include "spi.h"
#include "proj.h"

uint8_t MAX3421_init(void)
{
    // red led on
    regWr(rIOPINS2, bmGPOUT6);

    // remove RST
    MAX3421E_RST_OFF;

    // MAX4789 is connected to GPOUT0 to controls the VBUS
    // enable VBUS 
    regWr(rIOPINS1, bmGPOUT0);
    
    // set full duplex SPI, level-active INT interrupt
    regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL)); 

    // FIXME check OSCOKIRQ

    // set host mode
    // set pulldowns for peripheral plugin and speed detection
    regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);

    // peripheral and frame generator interrupt enable
    regWr(rHIEN, bmCONDETIE | bmFRAMEIE);

    //regWr(rHCTL, bmSAMPLEBUS);
    //while(!(regRd(rHCTL) & bmSAMPLEBUS)); //wait for sample operation to finish

    // clear connection detect interrupt
    regWr(rHIRQ, bmCONDETIRQ);
    // enable INT pin
    regWr(rCPUCTL, bmIE);

    return EXIT_SUCCESS;
}

void regWr(const uint8_t reg, const uint8_t val)
{
    uint8_t data[2];

    // data[0] is the command byte, data[1] is the data byte
    data[0] = reg | MAX3421_WRITE;
    data[1] = val;

    MAX3421_CS_SELECT;
    spi_send_frame(data, 2);
    MAX3421_CS_DESELECT;
}

uint8_t regRd(const uint8_t reg)
{
    uint8_t rv=0;

    MAX3421_CS_SELECT;
    spi_send_frame((uint8_t *)&reg, 1);
    spi_read_frame(&rv, 1);
    MAX3421_CS_DESELECT;

    return rv;
}

uint8_t *bytesRd(uint8_t reg, uint8_t nbytes, uint8_t* data_p)
{
    MAX3421_CS_SELECT;
    spi_send_frame((uint8_t *)&reg, 1);
    spi_read_frame(data_p, nbytes);
    MAX3421_CS_DESELECT;
    data_p += nbytes;
    return data_p;
}


