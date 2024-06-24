#include "dac.h"

/**
  * @brief  Initialize DAC peripheral
  * @retval void
  */
void DAC_init() {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Clock enable for SPI1
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  GPIOA->AFR[0] |= (5 << 20) | (5 << 28); // Set alternate function for SPI1 (0101 for SPI1) (4 * 5 bit shift for PA5, 4 * 7 bit shift for PA7)
  GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE7); // Clear mode register
  GPIOA->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1); // Configure PA4 as output, PA5 and PA7 as alternate function
  GPIOA->OTYPER &= ~(GPIO_OTYPER_ODR_4 | GPIO_OTYPER_ODR_5 | GPIO_OTYPER_ODR_7); // Push-pull output
  GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED7); // Very high speed
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD7); // No pull-up, no pull-down
  GPIOA->ODR |= GPIO_ODR_OD4; // Enable chip select

  SPI1->CR1 = 0; // Reset SPI1 config register
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR; // Enable master mode and slave select
  SPI1->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; // 16-bit data size
  SPI1->CR1 &= ~(SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2); // Set baud rate to fclk/2
  SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}

/**
  * @brief  Write data to DAC
  * @param  data: 16-bit data to write to DAC
  * @retval void
  */
void DAC_write(uint16_t data) {
  while (!(SPI1->SR & SPI_SR_TXE)); // Check if data FIFO is empty
  GPIOA->ODR &= ~(GPIO_ODR_OD4);  // Set CS low
  SPI1->DR = data; // Send data
  while (SPI1->SR & SPI_SR_BSY); // Wait for transmission to complete
  GPIOA->ODR |= GPIO_ODR_OD4;    // Set CS high
}