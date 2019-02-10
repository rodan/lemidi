#ifndef __MAX3421_H__
#define __MAX3421_H__

#ifdef __cplusplus
extern "C" {
#endif

// customize the chip select pin for your environment
//#define MAX3421_CS_SELECT   P4OUT |= BIT6
//#define MAX3421_CS_DESELECT  P4OUT &= ~BIT6

#include "max3421e.h"
#include "usb_spec.h"

#define MAX3421_WRITE 0x2
#define MAX3421_READ  0x0

void doHostReset(void);
uint8_t MAX3421_init(void);
uint8_t MAX3421_getVbusState(void);
uint8_t configure(const uint8_t parent, const uint8_t port, const uint8_t speed);

// interrupt counters
volatile uint32_t int_cnt, int_cnt_hl;
volatile uint32_t gpx_cnt, gpx_cnt_hl;

// low level internals
void regWr(const uint8_t reg, const uint8_t val);
uint8_t *bytesWr(const uint8_t reg, const uint8_t nbytes, uint8_t * data_p);
uint8_t regRd(const uint8_t reg);
uint8_t *bytesRd(const uint8_t reg, const uint8_t nbytes, uint8_t * data_p);
void gpioWr(const uint8_t data);
uint8_t gpioRd(void);
uint8_t gpioRdOutput(void);
void busprobe(void);

uint8_t InTransfer(struct UHS_EpInfo *pep, const uint16_t nak_limit, uint16_t * nbytesptr,
                   uint8_t * data);
uint8_t OutTransfer(struct UHS_EpInfo *pep, const uint16_t nak_limit, const uint16_t nbytes,
                    uint8_t * data);
uint8_t dispatchPkt(const uint8_t token, const uint8_t ep, const uint16_t nak_limit);

struct UHS_Device *GetUsbDevicePtr(const uint8_t addr);
struct UHS_EpInfo *getEpInfoEntry(const uint8_t addr, const uint8_t ep);
uint8_t SetAddress(const uint8_t addr, const uint8_t ep, struct UHS_EpInfo **ppep,
                   uint16_t * nak_limit);

uint8_t getone(struct UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint8_t *dataptr, uint8_t *offset);
uint8_t eat(struct UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint8_t *dataptr, uint8_t *offset, uint16_t *yum);
struct UHS_EpInfo *ctrlReqOpen(const uint8_t addr, const uint64_t Request, uint8_t * dataptr);
uint8_t ctrlReqRead(struct UHS_EpInfo *pep, uint16_t * left, uint16_t * read,
                    const uint16_t nbytes, uint8_t * dataptr);
uint8_t ctrlReqClose(struct UHS_EpInfo *pep, const uint8_t bmReqType, uint16_t left,
                     const uint16_t nbytes, uint8_t * dataptr);
uint8_t ctrlReq(uint8_t addr, uint64_t Request, uint16_t nbytes, uint8_t * dataptr);

uint8_t TestInterface(struct ENUMERATION_INFO * ei);
uint8_t enumerateInterface(struct ENUMERATION_INFO * ei);

uint8_t getDevDescr(const uint8_t addr, const uint16_t nbytes, uint8_t * dataptr);
uint8_t getConfDescr(const uint8_t addr, const uint16_t nbytes, const uint8_t conf,
                     uint8_t * dataptr);
uint8_t getStrDescr(const uint8_t addr, const uint16_t ns, const uint8_t index,
                    const uint16_t langid, uint8_t * dataptr);
uint8_t sof_delay(const uint16_t x);


// interrupt related
uint8_t get_ifg_int_event(void);
void rst_ifg_int_event(void);
uint8_t get_ifg_gpx_event(void);
void rst_ifg_gpx_event(void);

// address related
void InitEntry(const uint8_t index);
uint8_t FindAddressIndex(const uint8_t address);
uint8_t FindChildIndex(struct UHS_DeviceAddress addr, const uint8_t start);
void FreeAddressByIndex(const uint8_t index);
void InitAllAddresses(void);
struct UHS_Device *GetUsbDevicePtr(uint8_t addr);

#ifdef __cplusplus
}
#endif
#endif
