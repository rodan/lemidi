#ifndef __SYS_MESSAGEBUS_H__
#define __SYS_MESSAGEBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "proj.h"

/*!
	\brief List of possible message types for the message bus.
	\sa sys_messagebus_register()
*/
/* WARNING: the enum values are optimized to work with some drivers.
	If you need to add a new entry, append it to the end! */

#define           SYS_MSG_NULL 0
    // TIMER0
#define    SYS_MSG_TIMER0_CRR0 0x1
#define    SYS_MSG_TIMER0_CRR1 0x2   // timer_a0_delay_noblk_ccr1
#define    SYS_MSG_TIMER0_CRR2 0x4   // timer_a0_delay_noblk_ccr2
#define    SYS_MSG_TIMER0_CRR3 0x8   // timer_a0_delay_noblk_ccr3
#define    SYS_MSG_TIMER0_CRR4 0x10
#define     SYS_MSG_TIMER0_IFG 0x20  // timer_a0 overflow
    // UARTs
#define       SYS_MSG_UART0_RX 0x40
    // interrupts
#define      SYS_MSG_P1IFG_GPX 0x80  // port1 interrupt
#define      SYS_MSG_P1IFG_INT 0x100 // port1 interrupt
    // RTC
#define     SYS_MSG_RTC_SECOND 0x200 // second event from the hardware RTC

/*!
	\brief Linked list of nodes listening to the message bus.
*/
struct sys_messagebus {
    /*! callback for receiving messages from the system bus */
    void (*fn) (const uint16_t sys_message);
    /*! bitfield of message types that the node wishes to receive */
    uint16_t listens;
    /*! pointer to the next node in the list */
    struct sys_messagebus *next;
};

/*!
	\brief Registers a node in the message bus.
	\details Registers (add) a node to the message bus. A node can filter what message(s) are to be received by setting the bitfield \b listens.
	\sa sys_message, sys_messagebus, sys_messagebus_unregister
*/
void sys_messagebus_register(
        // callback to receive messages from the message bus
        void (*callback) (const uint16_t sys_message),
        // only receive messages of this type
        const uint16_t listens
    );

/*!
	\brief Unregisters a node from the message bus.
	\sa sys_messagebus_register
*/
void sys_messagebus_unregister(
        // the same callback used on sys_messagebus_register()
        void (*callback) (const uint16_t sys_message)
    );

struct sys_messagebus * sys_messagebus_getp(void);

#ifdef __cplusplus
}
#endif

#endif
