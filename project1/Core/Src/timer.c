#include "timer.h"
#include "dac.h"
#include "wave.h"
#include "buffer.h"

uint8_t duty_cycle = 50;
waveform wave = SQUARE;
uint16_t frequency = 1;
uint16_t counter = 0;
uint16_t val;
uint16_t threshold;

/**
  * @brief  Timer Interrupt Handler. Handles DAC output based on waveform and frequency.
  * @retval void
  */
void TIM2_IRQHandler() {
  if (counter + frequency >= 1000) {
    counter = 0;
  }
  counter += frequency;
  switch (wave) {
    case SINE:
      val = sine_wave[counter];
      DAC_write(0x7000 | val);
      break;
    case SQUARE:
      if (counter < (duty_cycle * 1000) / 100) {
        GPIOA->ODR |= GPIO_ODR_OD1;
        DAC_write(0x7000 | MAX_DAC_VAL);
      }
      else {
        GPIOA->ODR &= ~GPIO_ODR_OD1;
        DAC_write(0x7000);
      }
      break;
    case SAWTOOTH:
      val = sawtooth_wave[counter];
      DAC_write(0x7000 | val);
      break;
    case TRIANGLE:
      val = triangle_wave[counter];
      DAC_write(0x7000 | val);
  }
  TIM2->SR &= ~TIM_SR_UIF; // Clear interrupt flags
}

/**
  * @brief  Initializes interrupt debugging pin.
  * @retval void
  */
void TIM2_GPIO_init() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // Enable clock for GPIOA
  GPIOA->MODER &= ~GPIO_MODER_MODE1_Msk; // Clear mode register
  GPIOA->MODER |= GPIO_MODER_MODE1_0; // Set mode to output
}

/**
  * @brief  Initializes TIM2 registers for function generator output
  * @retval void
  */
void TIM2_init() {
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable clock for TIM2

  TIM2->PSC = 0; // Prescaler
  TIM2->ARR = 804 - 1; // Auto reload
  TIM2->CR1 &= ~TIM_CR1_DIR; // Downcount

  TIM2->DIER |= (TIM_DIER_UIE); // Enable interrupt flags
  TIM2->SR &= ~(TIM_SR_UIF); // Clear UIF flag
  TIM2->CR1 |= TIM_CR1_CEN; // Enable counter
  NVIC->ISER[0] |= 1 << TIM2_IRQn; // Enable interrupts
}