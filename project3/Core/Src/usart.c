#include "usart.h"

extern float32_t fft_mag[ADC_SAMPLES / 2];

/**
  * @brief  Initialize USART2 for UART communication
  * @retval void
  */
void USART_Init() {
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // enable peripheral clocks
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  NVIC->ISER[1] |= (1 << (USART2_IRQn & IRQ_MASK)); // enable USART2 interrupt

  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
  GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos); // alternate function 7 for USART2 TX and RX
  GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3); // clear mode
  GPIOA->MODER |= GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1; // alternate function
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3; // very high speed
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3); // no pull-up, no pull-down
  GPIOA->OTYPER &= ~(GPIO_OTYPER_ODR_2 | GPIO_OTYPER_ODR_3); // push-pull

  USART2->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);
  USART2->BRR = (__sysc_clk / baudrate); // set baudrate
  USART2->CR1 |= USART_CR1_RXNEIE; // enable receive interrupt
  USART2->CR1 |= USART_CR1_TE | USART_CR1_UE | USART_CR1_RE; // enable transmitter, receiver, and USART
}

/**
  * @brief  Transmit a string over UART
  * @param uint8_t *data - pointer to string to transmit
  * @retval void
  */
void USART_send(uint8_t *data) {
  while (*data != '\0') { // send data until null character
    while (!(USART2->ISR & USART_ISR_TXE)); // wait for transmit buffer to be empty
    USART2->TDR = *data; // write data to transmit buffer
    data++; // increment string pointer
  }
}

/**
  * @brief  Determine how many bars to display for each frequency band
  * @param uint8_t measured_frequencies[] - array to store measured frequencies
  * @param float32_t max_value - maximum value of the FFT output
  * @retval void
  */
void determine_freq_spectrum(uint8_t measured_frequencies[], float32_t max_value) {
  const uint16_t frequency_bands[16] = {20, 40, 80, 160, 320, 500, 700, 1000, 1500, 2000, 3000, 4000, 5000, 6000, 8000, 10000};
  uint16_t curr_frequency, freq_chars = 0;
  for (uint8_t i = 0; i < 16; i++) {
    curr_frequency = (uint16_t) frequency_bands[i] / BIN_FACTOR;
    freq_chars = (uint16_t) (fft_mag[curr_frequency] * 70 / max_value);
    measured_frequencies[i] = freq_chars;
  }
}

/**
  * @brief  Set up the terminal interface
  * @retval void
  */
void USART_Init_Terminal(void) {
  USART_send((uint8_t *)"\033[2J"); // clear terminal
  USART_send((uint8_t *)"\033[H"); // move cursor to top left
  USART_send((uint8_t *)"\033[27C"); // move cursor 27 characters to the right
  USART_send((uint8_t *)"Audio Spectrum Visualizer");
  USART_send((uint8_t *)"\033[2B"); // move cursor 2 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"\033[31C"); // move cursor 31 characters to the right
  USART_send((uint8_t *)"By Justin Carlson");
  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line

  for (uint8_t i = 0; i < 80; i++) {
    USART_send((uint8_t *)"—");
  }

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"  20Hz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"  40Hz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"  80Hz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)" 160Hz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  for (uint8_t i = 0; i < 80; i++) {
    USART_send((uint8_t *)"—");
  }

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)" 320Hz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)" 500Hz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)" 700Hz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"  1kHz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  for (uint8_t i = 0; i < 80; i++) {
    USART_send((uint8_t *)"—");
  }

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"1.5kHz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"  2kHz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"  3kHz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"  4kHz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  for (uint8_t i = 0; i < 80; i++) {
    USART_send((uint8_t *)"—");
  }

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"  5kHz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"  6kHz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)"  8kHz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  USART_send((uint8_t *)" 10kHz |");

  USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  USART_send((uint8_t *)"\r"); // move cursor to beginning of line
  for (uint8_t i = 0; i < 80; i++) {
    USART_send((uint8_t *)"—");
  }
}

/**
  * @brief  Update the terminal interface with the measured frequencies
  * @param uint8_t measured_frequencies[] - array of measured frequencies
  * @retval void
  */
void USART_Update(uint8_t measured_frequencies[]) {
  USART_send((uint8_t *)"\033[H"); // move cursor to top left
  USART_send((uint8_t *)"\033[4B"); // move cursor 4 lines down
  USART_send((uint8_t *)"\033[8C"); // move cursor 8 characters to the right
  uint8_t frequency_index = 0;
  uint8_t escape_buffer[11];
  for (uint8_t i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      USART_send((uint8_t *)"\033[0K"); // clear line to right
      sprintf((char *) escape_buffer, "□\033[%db", measured_frequencies[frequency_index]); // create escape sequence to display number of bars
      USART_send(escape_buffer); // transmit measured_frequencies[frequency_index] bars
      USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
      USART_send((uint8_t *)"\r"); // move cursor to beginning of line
      USART_send((uint8_t *)"\033[8C"); // move cursor 8 characters to the right
      frequency_index++;
    }
    USART_send((uint8_t *)"\033[1B"); // move cursor 1 lines down
  }
}