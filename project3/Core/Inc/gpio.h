#ifndef PROJECT3_GPIO
#define PROJECT3_GPIO
#include "stm32l476xx.h"

#define GPIO_FREQUENCY_MIN 30

void LED_Init(void);
void LED_On(uint8_t measured_frequencies[]);

#endif