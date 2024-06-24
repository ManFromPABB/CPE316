/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

#include "main.h"
#define KEYPAD_STAR 10
#define KEYPAD_POUND 12
#define KEYPAD_NO_PRESS 0xFF
#define KEYPAD_NUM_ROWS 4
#define KEYPAD_NUM_COLS 3
#define KEYPAD_ROW_OFF  0
#define KEYPAD_COL_OFF KEYPAD_NUM_ROWS
#define LED_OFF KEYPAD_NUM_ROWS + KEYPAD_NUM_COLS
#define MAX_DAC_VAL 4095
#define MAX_VOLTAGE 330
#define OUTPUT_GAIN_POS 13
#define SHDN_POS 12
#define DEBOUNCE_DELAY 1500000

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void DAC_init();
void DAC_write(uint16_t data);
void Keypad_init();
uint8_t Keypad_read();
uint16_t DAC_volt_conv(uint8_t *keys);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();

  SystemClock_Config();

  DAC_init();
  Keypad_init();

  while (1)
  {
    uint8_t keys[3]; // Create a buffer for 3 digit input
    uint16_t data; // Variable for transmission data with flags
    uint16_t value; // Variable for DAC value
    uint8_t key; // Last key pressed
    uint8_t read = 0; // Debouncing flag
    for (int i = 0; i < 3; i++) { // Read 3 digits
      key = Keypad_read(); // Read from keypad
      for (int j = 0; j < DEBOUNCE_DELAY; j++); // Wait for debouncing
      if (key != KEYPAD_NO_PRESS & key != KEYPAD_STAR & key != KEYPAD_POUND & read == 0) { // Make sure key is valid
        keys[i] = key;
        read = 1; // Enable to make sure key isn't double pressed
      }
      else { // If key is invalid, decrement i and reset read flag
        i--;
        read = 0;
      }
    }
    value = DAC_volt_conv(keys); // Convert 0-330 value to 0-4095
    if (value > MAX_DAC_VAL) { // Do not transmit if above 3.3V
      continue;
    }
    data = value | (1 << OUTPUT_GAIN_POS) | (1 << SHDN_POS); // Add transmit flags
    DAC_write(data); // Write to DAC
  }
}

void DAC_init() {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Clock enable for SPI1
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  GPIOA->AFR[0] |= (GPIO_AF5_SPI1 << 20) | (GPIO_AF5_SPI1 << 28); // Set alternate function for SPI1
  GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE7); 
  GPIOA->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1); // Configure PA4 as output, PA5 and PA7 as alternate function
  GPIOA->OTYPER &= ~(GPIO_OTYPER_ODR_4 | GPIO_OTYPER_ODR_5 | GPIO_OTYPER_ODR_7); // Push-pull output
  GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED7); // Very high speed
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD7); // No pull-up, no pull-down
  GPIOA->ODR |= GPIO_ODR_OD4; // Enable CS

  SPI1->CR1 = 0;
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR; // Enable master mode and slave select
  SPI1->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; // 16-bit data size
  SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2; // Set baud rate to lowest value
  SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}

void DAC_write(uint16_t data) {
  while (!(SPI1->SR & SPI_SR_TXE)); // Check if data FIFO is empty
  GPIOA->ODR &= ~(GPIO_ODR_OD4);  // Set CS low
  SPI1->DR = data; // Send data
  while (SPI1->SR & SPI_SR_BSY); // Wait for transmission to complete
  GPIOA->ODR |= GPIO_ODR_OD4;    // Set CS high
}

uint16_t DAC_volt_conv(uint8_t *keys) {
  return (keys[0] * 100 + keys[1] * 10 + keys[2]) * MAX_DAC_VAL / MAX_VOLTAGE; // Convert 0-330 value to 0-4095
}

void Keypad_init() {

	// initialize clock for GPIO port C
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOCEN);

	/*
	 * Pin 0-3: keypad rows
	 * Pin 4-6: keypad cols
	 * Pin 7-10: LEDs
	 */

	// input mode for keypad rows, output mode for all other used pins

	GPIOC->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 |
			GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5 |
			GPIO_MODER_MODE6);

	GPIOC->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 |
			GPIO_MODER_MODE6_0);

	// output type PP for keypad col and LED pins (outputs)
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 |
			GPIO_OTYPER_OT6);

	// pull down resistor on for keypad row pins (inputs), PUPD off for keypad col and LED pins (outputs)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2 |
			GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 |
			GPIO_PUPDR_PUPD6);

	GPIOC->PUPDR |= (GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1 | GPIO_PUPDR_PUPD2_1 |
				GPIO_PUPDR_PUPD3_1);

	// slow speed for keypad col and LED pins (outputs)
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5 |
			GPIO_OSPEEDR_OSPEED6);

	// initialize keypad cols outputs to '1' and LEDs to 0
	GPIOC->ODR |= (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6);
}

uint8_t Keypad_read() {

  GPIOC->ODR |= (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6); // set all cols to '1'

	if (!(GPIOC->IDR & 0xF)) {
    return KEYPAD_NO_PRESS;
  }

	int32_t col=0;
	while (col < KEYPAD_NUM_COLS) {

		GPIOC->ODR &= ~(GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6);
		GPIOC->ODR |= 1 << (KEYPAD_COL_OFF+col);

		if (GPIOC->IDR & 0xF) {
			GPIOC->ODR |= (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6); // set all cols to '1'
			if (GPIOC->IDR & 1 ) { // row 0
				return col+1;
			}
      else if (GPIOC->IDR & 2) { // row 1
        return col+KEYPAD_NUM_COLS+1;
      }
      else if (GPIOC->IDR & 4) { // row 2
        return col+2*KEYPAD_NUM_COLS+1;
      }
      else { // row 3
        if(!col) {
          return KEYPAD_STAR;
        }
        else if (col == 1) {
          return 0;
        }
        else {
          return KEYPAD_POUND;
        }
      }
    }
		col++;
	}
	return KEYPAD_NO_PRESS;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);


  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

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
