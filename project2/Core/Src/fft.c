#include "fft.h"

arm_rfft_fast_instance_f32 fft_instance; // FFT struct
float32_t fft_input[FFT_SIZE]; // Real FFT input/ADC samples
float32_t fft_output[FFT_SIZE]; // Complex FFT output
float32_t fft_mag[FFT_SIZE / 2]; // magnitude of FFT output
extern uint16_t adc_input[FFT_SIZE]; // Vpp calculation values

/**
  * @brief  Compute FFT and return dominant frequency of input signal
  * @retval float32_t - frequency of input signal
  */
float32_t FFT_calc(void) {
  float32_t max;
  uint32_t max_index;
  arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0); // compute FFT
  arm_cmplx_mag_f32(fft_output, fft_mag, FFT_SIZE / 2); // convert complex frequencies to real magnitude
  float32_t dc_offset = fft_mag[0]; // store to compare with AC frequency bins
  fft_mag[0] = 0; // remove DC offset from FFT calc
  arm_max_f32(fft_mag, (FFT_SIZE / 2), &max, &max_index); // determine dominant frequency bin
  float32_t dc_ac_determination = dc_offset / max; // describe how dominant DC is compared to AC
  if (dc_ac_determination < AC_DC_THRESHOLD) { // check whether DC frequency bin is significantly more dominant than AC frequency bins
    if (max_index > MAX_FREQUENCY) { // filter out any noisy harmonics above max frequency
      return (float32_t) max_index;
    }
    return (float32_t) max_index * FFT_CAL_FACTOR; // return AC frequency bin
  } else {
    return 0; // return 0 if DC frequency bin is more dominant than AC frequency bins
  }
}

/**
  * @brief  Find min value in adc_data
  * @retval uint32_t - min value in adc_data
  */
uint32_t find_min() {
  uint32_t min = adc_input[0];
  for (uint16_t i = 1; i < FFT_SIZE; i++) {
    if (adc_input[i] < min) {
      min = adc_input[i];
    }
  }
  return min;
}

/**
  * @brief  Find max value in adc_data
  * @retval uint32_t - max value in adc_data
  */
uint32_t find_max() {
  uint32_t max = adc_input[0];
  for (uint16_t i = 1; i < FFT_SIZE; i++) {
    if (adc_input[i] > max) {
      max = adc_input[i];
    }
  }
  return max;
}
