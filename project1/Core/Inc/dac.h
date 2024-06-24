#ifndef DAC_H
#define DAC_H
#include "stm32l476xx.h"

void DAC_init();
void DAC_write(uint16_t data);

#endif