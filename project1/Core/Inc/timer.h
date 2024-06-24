#ifndef TIMER_2
#define TIMER_2
#include "stm32l476xx.h"
#include "keypad.h"
#include "wave.h"

void TIM2_IrqHandler(void);
void TIM2_GPIO_init(void);
void TIM2_init(void);

extern uint8_t duty_cycle;
extern waveform wave;
extern uint16_t frequency;

#endif