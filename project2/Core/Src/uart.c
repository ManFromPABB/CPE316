#include "uart.h"
#include "stm32l476xx.h"
#include "arm_math.h"

/**
  * @brief  Initialize UART and UART interrupt
  * @retval void
  */
void UART_init() {
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
void UART_send(uint8_t *data) {
  while (*data != '\0') { // send data until null character
    while (!(USART2->ISR & USART_ISR_TXE)); // wait for transmit buffer to be empty
    USART2->TDR = *data; // write data to transmit buffer
    data++; // increment string pointer
  }
}

/**
  * @brief  Setup terminal for displaying AC/DC voltage, frequency, and bar graph
  * @retval void
  */
void UART_init_terminal(void) {
  UART_send((uint8_t *)"\033[2J"); // clear terminal
  UART_send((uint8_t *)"\033[H"); // move cursor to top left
  UART_send((uint8_t *)"AC/DC Voltage: "); // print AC/DC Voltage
  UART_send((uint8_t *)"\r"); // move cursor to beginning of line
  UART_send((uint8_t *)"\x1B[1B");   // Move cursor 1 lines down
  UART_send((uint8_t *)"Frequency: "); // print Frequency
  UART_send((uint8_t *)"\r"); // move cursor to beginning of line
  UART_send((uint8_t *)"\x1B[3B");   // Move cursor 2 lines down
  UART_send((uint8_t *)"|----|----|----|----|----|----|---");
  UART_send((uint8_t *)"\r"); // move cursor to beginning of line
  UART_send((uint8_t *)"\x1B[1B");   // Move cursor 1 lines down
  UART_send((uint8_t *)"0   0.5  1.0  1.5  2.0  2.5  3.0"); // print bar graph
}

/**
  * @brief  Update terminal with current AC/DC voltage, frequency, and bar graph values
  * @param uint8_t *freq - pointer to frequency string
  * @param uint8_t *rms - pointer to rms voltage string
  * @param uint8_t num_bars - number of bars to display
  * @param uint8_t *vpp - pointer to Vpp voltage string
  * @retval void
  */
void UART_update_terminal(uint8_t *freq, uint8_t *rms, uint8_t num_bars, uint8_t *vpp) {
  UART_send((uint8_t *)"\033[H"); // move cursor to top left
  UART_send((uint8_t *)"\033[15C"); // move cursor 15 characters to the right
  UART_send((uint8_t *)"\033[0K"); // clear line to right
  UART_send(rms); // print rms voltage
  UART_send((uint8_t *)" Vpp: "); // print Vpp
  UART_send(vpp); // print vpp voltage
  UART_send((uint8_t *)"\r"); // move cursor to beginning of line
  UART_send((uint8_t *)"\x1B[1B");   // Move cursor 1 lines down
  UART_send((uint8_t *)"\033[11C"); // move cursor 11 characters to the right
  UART_send((uint8_t *)"\033[0K"); // clear line to right
  UART_send(freq); // print frequency
  UART_send((uint8_t *)"\r"); // move cursor to beginning of line
  UART_send((uint8_t *)"\x1B[2B"); // Move cursor 2 lines down
  UART_send((uint8_t *)"\033[2K"); // clear line
  for (uint8_t i = 0; i < num_bars + 1; i++) {
    UART_send((uint8_t *)"*"); // transmit new bar graph
  }
}

/**
  * @brief  Convert float to string
  * @param uint8_t *string - pointer to string to store float
  * @param float32_t f - float to convert
  * @retval void
  */
void float_to_string(uint8_t *string, float32_t f) {
  uint8_t integer = (uint8_t) f; // get integer part from float
  float32_t fracPart = f - integer; // get fractional part
  fracPart = fracPart * FLOAT_TO_STRING_FRAC + FLOAT_TO_STRING_ROUNDOFF; // round to 2 decimal places
  uint32_t roundedFracPart = (uint32_t)fracPart; // get rounded fractional part
  *string = (uint8_t) integer + '0'; // integer char
  *(string + 1) = '.';
  *(string + 2) = (uint8_t) (roundedFracPart / FLOAT_TO_STRING_TENS) + '0'; // tenths place
  *(string + 3) = (uint8_t) (roundedFracPart % FLOAT_TO_STRING_TENS) + '0'; // hundredths place
  *(string + 4) = '\0'; // null character
}

/**
  * @brief  Convert integer to string
  * @param uint32_t num - integer to convert
  * @param uint8_t *str - pointer to string to store integer
  * @retval uint8_t * - pointer to string
  */
uint8_t *int_to_string(uint32_t num, uint8_t str[]) {
    uint32_t i = 0;
    uint32_t start = 0;

    if (num >= MAX_FREQUENCY) {
        str = (uint8_t *)"1000\0"; // max allowable frequency
        return str;
    }

    if (num >= INT_TO_STRING_HUNDREDS) { // check if 100s conversion needed
        str[i++] = (num / INT_TO_STRING_HUNDREDS) + '0'; // convert 100s place
        num %= INT_TO_STRING_HUNDREDS; // remove 100s place
        start = 1;
    }
    if (num >= FLOAT_TO_STRING_TENS || start) { // check if 10s conversion needed
        str[i++] = (num / FLOAT_TO_STRING_TENS) + '0'; // convert 10s place
        num %= FLOAT_TO_STRING_TENS; // remove 10s place
    }
    str[i++] = num + '0'; // convert 1s place
    str[i] = '\0'; // null character
    return str;
}