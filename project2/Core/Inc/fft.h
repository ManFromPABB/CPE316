#ifndef PROJECT2_FFT_H
#define PROJECT2_FFT_H
#include "arm_math.h" // DSP library

#define FFT_SIZE 4096
#define FFT_CAL_FACTOR 2

#define AC_DC_THRESHOLD 100
#define MAX_FREQUENCY 1000

float32_t FFT_calc(void);
uint32_t find_min();
uint32_t find_max();

#endif