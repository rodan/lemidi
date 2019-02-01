#ifndef __TIMER_A0_H__
#define __TIMER_A0_H__

#include "proj.h"

#define _10ms           5UL       // ~10ms
#define _500ms          _10ms * 50
#define _1200ms         _10ms * 120
#define _1s             512UL
#define _2s             _1s * 2
#define _3s             _1s * 3
#define _3sp            _3s + SM_STEP_DELAY
#define _5s             _1s * 5
#define _5sp            _5s + SM_STEP_DELAY
#define _6s             _1s * 6
#define _6sp            _6s + SM_STEP_DELAY
#define _10s            _1s * 10
#define _10sp           _10s + SM_STEP_DELAY
#define _14s            _1s * 14
#define _30s            _1s * 30
#define _60s            _1s * 60
#define _75s            _1s * 75

#define    TIMER_A0_EVENT_NONE 0
#define    TIMER_A0_EVENT_CCR0 0x1
#define    TIMER_A0_EVENT_CCR1 0x2
#define    TIMER_A0_EVENT_CCR2 0x4
#define    TIMER_A0_EVENT_CCR3 0x8
#define    TIMER_A0_EVENT_CCR4 0x10
#define     TIMER_A0_EVENT_IFG 0x20

volatile uint8_t timer_a0_last_event;
volatile uint16_t timer_a0_ovf;

void timer_a0_init(void);
void timer_a0_halt(void);
void timer_a0_delay_noblk_ccr1(uint16_t ticks);
void timer_a0_delay_noblk_ccr2(uint16_t ticks);
void timer_a0_delay_noblk_ccr3(uint16_t ticks);
void timer_a0_delay_ccr4(uint16_t ticks);

#endif
