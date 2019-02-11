#ifndef __TIMER_A0_H__
#define __TIMER_A0_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "proj.h"

#define _200us          25UL
#define _1ms            125UL
#define _10ms           _1ms * 10
#define _200ms          _1ms * 200
#define _500ms          _1ms * 500

#define    TIMER_A0_EVENT_NONE 0
#define    TIMER_A0_EVENT_CCR0 0x1
#define    TIMER_A0_EVENT_CCR1 0x2
#define    TIMER_A0_EVENT_CCR2 0x4
#define    TIMER_A0_EVENT_CCR3 0x8
#define    TIMER_A0_EVENT_CCR4 0x10
#define     TIMER_A0_EVENT_IFG 0x20

void timer_a0_rst_event(void);
uint8_t timer_a0_get_event(void);
void timer_a0_init(void);
void timer_a0_halt(void);
void timer_a0_delay_noblk_ccr1(uint16_t ticks);
void timer_a0_delay_noblk_ccr2(uint16_t ticks);
void timer_a0_delay_noblk_ccr3(uint16_t ticks);
void timer_a0_delay_ccr4(uint16_t ticks);

uint32_t millis(void);

#ifdef __cplusplus
}
#endif

#endif
