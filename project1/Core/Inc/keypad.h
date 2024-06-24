#ifndef KEYPAD
#define KEYPAD
#define KEYPAD_STAR 10
#define KEYPAD_POUND 12
#define KEYPAD_NO_PRESS 0xFF
#define KEYPAD_NUM_ROWS 4
#define KEYPAD_NUM_COLS 3
#define KEYPAD_ROW_OFF  0
#define KEYPAD_COL_OFF KEYPAD_NUM_ROWS
#define LED_OFF KEYPAD_NUM_ROWS + KEYPAD_NUM_COLS
#define MAX_DAC_VAL 3724
#define MAX_VOLTAGE 330
#define OUTPUT_GAIN_POS 13
#define SHDN_POS 12
#define DEBOUNCE_DELAY 1000
#include "timer.h"
#include "wave.h"
#include "stm32l476xx.h"

void Keypad_init();
uint8_t Keypad_read();
void Keypad_parse(uint8_t key, uint8_t *duty_cycle, uint16_t *frequency, waveform *waveform);

#endif