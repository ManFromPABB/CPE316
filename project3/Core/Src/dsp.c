#include "dsp.h"
#include "fdacoefs.h"

arm_rfft_fast_instance_f32 fft_instance; // FFT struct
arm_fir_instance_f32 fir_instance; // FIR struct

uint16_t adc[ADC_SAMPLES]; // ADC input

float32_t fft_output[ADC_SAMPLES]; // Complex FFT output
float32_t fft_mag[ADC_SAMPLES / 2]; // magnitude of FFT output

float32_t fir_state[ADC_SAMPLES]; // FIR state buffer
float32_t fir_in[ADC_SAMPLES]; // FIR input
float32_t fir_out[ADC_SAMPLES]; // FIR output

/**
  * @brief  Initialize FIR lowpass and FFT structs
  * @retval void
  */
void DSP_Init() {
    arm_rfft_fast_init_f32(&fft_instance, ADC_SAMPLES); // initialize FFT struct
    arm_fir_init_f32(&fir_instance, NUM_TAPS, FIRCoefs, fir_state, ADC_SAMPLES / 2); // initialize FIR struct
}

/**
  * @brief  Compute FFT of input data
  * @retval float32_t the maximum value of the FFT output
  */
float32_t FFT_Process() {
    float32_t max_value = 0;
    uint32_t max_index = 0;
    arm_rfft_fast_f32(&fft_instance, fir_out, fft_output, 0); // perform FFT
    arm_cmplx_mag_f32(fft_output, fft_mag, ADC_SAMPLES / 2); // calculate magnitude
    fft_mag[0] = 0; // remove DC component
    arm_max_f32(fft_mag, ADC_SAMPLES / 2, &max_value, &max_index); // find maximum value
    return max_value;
}

/**
  * @brief  Apply a FIR lowpass filter to the first half of the ADC data
  * @retval void
  */
void FIR_LP_Half_Filtering() {
    for (uint16_t i = 0; i < ADC_SAMPLES / 2; i++) { fir_in[i] = (float32_t) adc[i]; } // convert input to float
    Hamming_Window(fir_out, ADC_SAMPLES / 2); // apply Hamming window
    arm_fir_f32(&fir_instance, fir_in, fir_out, ADC_SAMPLES / 2); // perform half of FIR filtering
}

/**
  * @brief  Apply a FIR lowpass filter to the second half of the ADC data
  * @retval void
  */
void FIR_LP_Filtering() {
    for (uint16_t i = ADC_SAMPLES / 2; i < ADC_SAMPLES; i++) { fir_in[i] = (float32_t) adc[i]; } // convert input to float
    Hamming_Window(fir_out + ADC_SAMPLES / 2, ADC_SAMPLES / 2); // apply Hamming window
    arm_fir_f32(&fir_instance, fir_in + ADC_SAMPLES / 2, fir_out + ADC_SAMPLES / 2, ADC_SAMPLES / 2); // perform half of FIR filtering
}

/**
  * @brief  Apply a Hamming window to reduce spectral leakage
  * @retval void
  */
void Hamming_Window(float32_t fir_out[], uint16_t blocks) {
    const float32_t alpha = 0.54f;
    const float32_t beta = 1.0f - alpha;

    for (uint16_t i = 0; i < blocks; i++) {
        fir_out[i] = alpha - beta * cosf(2.0f * PI * i / (blocks - 1));
    }
}