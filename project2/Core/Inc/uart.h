#ifndef PROJECT2_UART_H
#define PROJECT2_UART_H
#include "stm32l476xx.h"
#include "arm_math.h"

#define IRQ_MASK 0x1F
#define __sysc_clk 80000000
#define baudrate 115200
#define MAX_FREQUENCY 1000

#define FLOAT_TO_STRING_FRAC 100
#define FLOAT_TO_STRING_ROUNDOFF 0.5
#define FLOAT_TO_STRING_TENS 10
#define INT_TO_STRING_HUNDREDS 100

void UART_init();
void UART_send(uint8_t *data);
void UART_update_adc(uint8_t *freq);
void UART_init_terminal(void);
void UART_update_terminal(uint8_t *freq, uint8_t *rms, uint8_t num_bars, uint8_t *vpp);
void float_to_string(uint8_t *string, float32_t f);
uint8_t *int_to_string(uint32_t num, uint8_t str[]);

#endif