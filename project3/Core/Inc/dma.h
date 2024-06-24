#ifndef PROJECT3_DMA
#define PROJECT3_DMA
#include "arm_math.h"
#include "stm32l476xx.h"

#define ADC_SAMPLES 4096
#define IRQ_MASK 0x1F

void DMA_Init(void);

#endif