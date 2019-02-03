
//   timer a0 handling
//   CCR0 is currently unused
//   CCR1 is used for timer_a0_delay_noblk_ccr1()
//   CCR2 is used for timer_a0_delay_noblk_ccr2()
//   CCR3 is used for timer_a0_delay_noblk_ccr3()
//   CCR4 is currently unused
//
//   author:          Petre Rodan <2b4eda@subdimension.ro>
//   available from:  https://github.com/rodan/
//   license:         GNU GPLv3

#include "timer_a0.h"

volatile uint8_t timer_a0_last_event;
volatile uint16_t timer_a0_ovf;
volatile uint32_t milliseconds;

uint32_t millis(void)
{
    return milliseconds;
}

void timer_a0_rst_event(void)
{
    timer_a0_last_event = TIMER_A0_EVENT_NONE;
}

uint8_t timer_a0_get_event(void)
{
    return timer_a0_last_event;
}

void timer_a0_init(void)
{
    __disable_interrupt();
    _NOP();
    timer_a0_ovf = 0;
    milliseconds = 0;
    TA0EX0 |= TAIDEX_7;
    TA0CTL |= TASSEL__ACLK + MC__CONTINOUS + TACLR + ID__8 + TAIE;
    __enable_interrupt();
}

// each tick is .000008 seconds
void timer_a0_delay_noblk_ccr1(uint16_t ticks)
{
    TA0CCTL1 &= ~CCIE;
    TA0CCTL1 = 0;
    TA0CCR1 = TA0R + ticks;
    TA0CCTL1 = CCIE;
}

void timer_a0_delay_noblk_ccr2(uint16_t ticks)
{
    TA0CCTL2 &= ~CCIE;
    TA0CCTL2 = 0;
    TA0CCR2 = TA0R + ticks;
    TA0CCTL2 = CCIE;
}

// used for millis() only
void timer_a0_delay_noblk_ccr3(uint16_t ticks)
{
    TA0CCTL3 &= ~CCIE;
    TA0CCTL3 = 0;
    TA0CCR3 = TA0R + ticks;
    TA0CCTL3 = CCIE;
}

void timer_a0_delay_ccr4(uint16_t ticks)
{
    //__disable_interrupt();
    TA0CCTL4 &= ~CCIE;
    TA0CCR4 = TA0R + ticks;
    TA0CCTL4 = CCIE;
    //__enable_interrupt();
    timer_a0_last_event &= ~TIMER_A0_EVENT_CCR4;
    while (1) {
        if (timer_a0_last_event & TIMER_A0_EVENT_CCR4)
            break;
    }
    TA0CCTL4 &= ~CCIE;
    timer_a0_last_event &= ~TIMER_A0_EVENT_CCR4;
}

__attribute__ ((interrupt(TIMER0_A1_VECTOR)))
void timer0_A1_ISR(void)
{
    uint16_t iv = TA0IV;
    if (iv == TA0IV_TA0CCR4) {
        // timer used by timer_a0_delay()
        timer_a0_last_event |= TIMER_A0_EVENT_CCR4;
    } else if (iv == TA0IV_TA0CCR1) {
        // timer used by timer_a0_delay_noblk_ccr1()
        // disable interrupt
        TA0CCTL1 &= ~CCIE;
        TA0CCTL1 = 0;
        timer_a0_last_event |= TIMER_A0_EVENT_CCR1;
        _BIC_SR_IRQ(LPM3_bits);
    } else if (iv == TA0IV_TA0CCR2) {
        // timer used by timer_a0_delay_noblk_ccr2()
        // disable interrupt
        TA0CCTL2 &= ~CCIE;
        TA0CCTL2 = 0;
        timer_a0_last_event |= TIMER_A0_EVENT_CCR2;
        _BIC_SR_IRQ(LPM3_bits);
    } else if (iv == TA0IV_TA0CCR3) {
        // timer used by timer_a0_delay_noblk_ccr3()
        // disable interrupt
        //TA0CCTL3 &= ~CCIE;
        milliseconds++;
        TA0CCTL3 += 125;
        // use hardware flow control to stop the remote equipment
        // from sending more data
        // timer_a0_last_event |= TIMER_A0_EVENT_CCR3;
        _BIC_SR_IRQ(LPM3_bits);
    } else if (iv == TA0IV_TA0IFG) {
        TA0CTL &= ~TAIFG;
        timer_a0_ovf++;
        timer_a0_last_event |= TIMER_A0_EVENT_IFG;
        _BIC_SR_IRQ(LPM3_bits);
    }
}
