#include "adc.h"

/**
  * @brief  Initialize the ADC peripheral
  * @retval void
  */
void ADC_Init(void) {
    /* PA0 -> ADC12_IN5 */
  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

  // apply / 2 prescaler to ADC clock
  ADC123_COMMON->CCR = (2 << ADC_CCR_CKMODE_Pos);

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
  ADC1->SMPR1 = (7 << ADC_SMPR1_SMP5_Pos); // lowest clocks per sample
  // Sequence has length of 1, channel 5
  ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);

  ADC1->CFGR |= ADC_CFGR_DMACFG; // DMA circular mode 
  ADC1->CFGR |= ADC_CFGR_DMAEN; // enable DMA

  // Configure PA0 for analog in
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // enable GPIOA clock
  GPIOA->MODER |= GPIO_MODER_MODE0; // analog mode
  GPIOA->ASCR |= GPIO_ASCR_ASC0; // connect PA0 to ADC

  // start a conversion
  ADC1->CR |= ADC_CR_ADSTART; // start ADC conversion
}

/**
  * @brief  Stop the ADC conversion
  * @retval void
  */
void ADC_Stop(void) {
  // Stop the ADC conversion
  if (ADC1->CR & ADC_CR_ADSTART) {
    ADC1->CR |= ADC_CR_ADSTP; // Stop ongoing conversion
    while (ADC1->CR & ADC_CR_ADSTP); // Wait until it is confirmed stopped
  }
}