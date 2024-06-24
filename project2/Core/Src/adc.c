#include "adc.h"
#include "stm32l476xx.h"
#include "fft.h"

extern float32_t fft_input[FFT_SIZE];
uint16_t adc_input[FFT_SIZE];
uint16_t adc_count = 0;

/**
  * @brief  Initialize ADC and ADC interrupt
  * @retval void
  */
void ADC_init() {
  /* PA0 -> ADC12_IN5 */
  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
  // set clock for synchronous hclk
  ADC123_COMMON->CCR = (1 << ADC_CCR_CKMODE_Pos);

  // wake up ADC and enable power
  ADC1->CR &= ~(ADC_CR_DEEPPWD);
  ADC1->CR |= ADC_CR_ADVREGEN;

  // wait 20us for ADC to power up
  for (uint8_t i = 0; i < ADC_WAKEUP_DELAY; i++);

  ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF); // disable ADC and differential mode
  ADC1->CR |= ADC_CR_ADCAL; // start calibration
  while (ADC1->CR & ADC_CR_ADCAL); // wait for calibration

  // configure channels for single ended mode
  ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);

  // enable ADC
  ADC1->ISR |= ADC_ISR_ADRDY; // clear with a 1
  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY)); // wait for bit to go high
  ADC1->ISR |= ADC_ISR_ADRDY; // clear with a 1

  // ADC single mode, 12-bit resolution, right aligned
  ADC1->CFGR = 0;
  ADC1->SMPR1 = (7 << ADC_SMPR1_SMP5_Pos); // max clocks per sample
  // Sequence has length of 1, channel 5
  ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);

  // Enable interrupts
  ADC1->IER |= ADC_IER_EOCIE; // enable end of conversion interrupt
  NVIC->ISER[0] = (1 << (ADC1_2_IRQn & IRQ_MASK)); // enable ADC1_2 interrupt

  // Configure PA0 for analog in
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // enable GPIOA clock
  GPIOA->MODER |= GPIO_MODER_MODE0; // analog mode
  GPIOA->ASCR |= GPIO_ASCR_ASC0; // connect PA0 to ADC

  // start a conversion
  ADC1->CR |= ADC_CR_ADSTART; // start ADC conversion
}

/**
  * @brief  Disable ADC and ADC interrupt
  * @retval void
  */
void ADC_stop() {
  // Stop the ADC conversion
  if (ADC1->CR & ADC_CR_ADSTART) {
    ADC1->CR |= ADC_CR_ADSTP; // Stop ongoing conversion
    while (ADC1->CR & ADC_CR_ADSTP); // Wait until it is confirmed stopped
  }

  // Disable the ADC interrupt
  ADC1->IER &= ~ADC_IER_EOCIE; // Disable end of conversion interrupt
  NVIC->ICER[0] = (1 << (ADC1_2_IRQn & IRQ_MASK)); // Disable ADC1_2 interrupt in NVIC
}

/**
  * @brief  ADC interrupt handler - read ADC value to FFT/ADC arrays
  * @retval void
  */
void ADC1_2_IRQHandler(void) {
  if (ADC1->ISR & ADC_ISR_EOC) {
    adc_input[adc_count] = ADC1->DR; // Read ADC value to Vpp array
    fft_input[adc_count] = (float32_t) adc_input[adc_count]; // Convert to float for FFT
    adc_count++; // Increment count
    for (uint16_t i = 0; i < ADC_CALIBRATION_DELAY; i++); // ADC calibration delay
  }
  ADC1->ISR &= ~ADC_ISR_EOC;
}