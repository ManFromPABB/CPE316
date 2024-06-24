#ifndef PROJECT3_ADC
#define PROJECT3_ADC
#include "stm32l476xx.h"

#define ADC_WAKEUP_DELAY 80
#define ADC_SAMPLE_DELAY 1000
#define IRQ_MASK 0x1F

void ADC_Init(void);
void ADC_Stop(void);

#endif