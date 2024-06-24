/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#define __sysc_clk 80000000
#define baudrate 115200
#define MAX_ADC 4095
#define MAX_VOLTAGE 3.3
#define FLOAT_STRING_SIZE 5
#define CALIBRATION_SCALE 1
#define CALIBRATION_OFFSET 18
#define IRQ_MASK 0x1F
#define FFT_SIZE 4096
#define FFT_CAL_FACTOR 1.86
#include "main.h"
#include <stdlib.h> // ONLY USED FOR MEMORY ALLOCATION (NO STRING OPERATIONS)
#include "arm_math.h"

void SystemClock_Config(void);
void ADC_init(void);
void ADC1_2_IRQHandler(void);
void FFT_init(void);
float32_t FFT_calc(void);
void UART_init(void);
void UART_send(char *data);
void UART_update_adc(char *freq);
char *int_to_string(int num, char str[]);

uint16_t adc_count = 0;

arm_rfft_fast_instance_f32 fft_instance;
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t fft_mag[FFT_SIZE / 2];

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();

  SystemClock_Config();
  __enable_irq(); // Enable global interrupts
  ADC_init(); // Initialize ADC
  UART_init(); // Initialize UART
  arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

  while (1) {
    ADC1->CR |= ADC_CR_ADSTART; // start ADC conversion
    if (adc_count == FFT_SIZE) {
      ADC1->CR = ADC_CR_ADVREGEN; // stop adc
      ADC1->CR &= ~ADC_CR_ADEN; // stop adc
      uint64_t sum = 0;
      for (int i = 0; i < FFT_SIZE; i++) {
        sum += fft_input[i];
      }
      float32_t avg = (float32_t) sum / FFT_SIZE;
    	uint32_t frequency = (uint32_t) FFT_calc();
      char freq[5];
      char *freq_s = int_to_string((int) frequency, freq);
      UART_update_adc(freq_s);
    	adc_count = 0;
      ADC_init();
    }
  }
}

float32_t FFT_calc(void) {
  float32_t max;
  uint32_t max_index;
  arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);
  arm_cmplx_mag_f32(fft_output, fft_mag, FFT_SIZE / 2);
  fft_mag[0] = 0;
  arm_max_f32(fft_mag, (FFT_SIZE / 2), &max, &max_index);
  return (float32_t) max_index * FFT_CAL_FACTOR;
}

void ADC_init() {
  /* PA0 -> ADC12_IN5 */
  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
  // set clock for synchronous hclk
  ADC123_COMMON->CCR = (1 << ADC_CCR_CKMODE_Pos);

  // wake up ADC and enable power
  ADC1->CR &= ~(ADC_CR_DEEPPWD);
  ADC1->CR |= ADC_CR_ADVREGEN;

  // wait 20us for ADC to power up
  for (int i = 0; i < 80; i++);

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
  ADC1->SMPR1 = (7 << ADC_SMPR1_SMP5_Pos); // 47.5 clocks per sample
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

void ADC1_2_IRQHandler(void) {
  if (ADC1->ISR & ADC_ISR_EOC) {
      fft_input[adc_count] = (float32_t) ADC1->DR * MAX_VOLTAGE / MAX_ADC;
      adc_count++; // Increment count
      for (int i = 0; i < 1000; i++);
  }
  ADC1->ISR &= ~ADC_ISR_EOC;
}

void UART_init() {
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
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

void UART_send(char *data) {
  while (*data != '\0') { // send data until null character
    while (!(USART2->ISR & USART_ISR_TXE)); // wait for transmit buffer to be empty
    USART2->TDR = *data; // write data to transmit buffer
    data++; // increment string pointer
  }
}

void UART_update_adc(char *freq) {
  UART_send("\033[2J"); // clear terminal
  UART_send("\033[H"); // move cursor to top left
  UART_send("Frequency: ");
  UART_send(freq);
}

char *int_to_string(int num, char str[]) {
    int i = 0;
    int start = 0;

    if (num >= 1000) {
        str = "1000\0";
        return str;
    }

    if (num >= 100) {
        str[i++] = (num / 100) + '0';
        num %= 100;
        start = 1;
    }
    if (num >= 10 || start) {
        str[i++] = (num / 10) + '0';
        num %= 10;
    }
    str[i++] = num + '0';
    str[i] = '\0';
    return str;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */