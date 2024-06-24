#ifndef PROJECT3_USART
#define PROJECT3_USART
#include "stm32l476xx.h"
#include "arm_math.h"
#include <stdio.h>

#define __sysc_clk 80000000
#define baudrate 115200
#define IRQ_MASK 0x1F
#define ADC_SAMPLES 4096
#define MAXIMUM_BIN 683
#define BIN_FACTOR 14.6412884
#define MAXIMUM_VALUE 12000
#define MIN2(x, y) ((x < y) ? x : y)
#define AVG2(x, y) ((x + y) / 2)

void USART_Init(void);
void USART_send(uint8_t *data);
void determine_freq_spectrum(uint8_t measured_frequencies[], float32_t max_value);
void USART_Init_Terminal(void);
void USART_Update(uint8_t measured_frequencies[]);

#endif