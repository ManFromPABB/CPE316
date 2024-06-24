#include "gpio.h"

/**
  * @brief  Initialize the GPIOB pins for the LED display
  * @retval void
  */
void LED_Init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // Enable GPIOB clock

    GPIOB->MODER = 0; // Clear MODER register

    for (int i = 0; i < 16; i++) {
        GPIOB->MODER |= GPIO_MODER_MODE0_0 << (i * 2); // Set all 16 pins to output mode
    }

    GPIOB->OTYPER = 0; // Set output type to push-pull

    GPIOB->PUPDR = 0; // Clear PUPDR register

    GPIOB->OSPEEDR = 0; // Set to low speed
}

/**
  * @brief  Update the LED display based on the measured frequency magnitudes
  * @retval void
  */
void LED_On(uint8_t measured_frequencies[]) {
    for (int i = 0; i < 16; i++) {
        if (measured_frequencies[i] > GPIO_FREQUENCY_MIN) {
            GPIOB->ODR |= 1 << i; // toggle on if frequency is strong
        }
        else {
            GPIOB->ODR &= ~(1 << i); // toggle off if frequency is weak
        }
    }
}