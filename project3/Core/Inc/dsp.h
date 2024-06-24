#ifndef PROJECT3_DSP
#define PROJECT3_DSP
#include "stm32l476xx.h"
#include "arm_math.h"

#define ADC_SAMPLES 4096
#define NUM_TAPS 38
#define MAX_BIN 430

void DSP_Init(void);
float32_t FFT_Process(void);
void FIR_LP_Half_Filtering(void);
void FIR_LP_Filtering(void);
void Hamming_Window(float32_t fir_out[], uint16_t blocks);

#endif