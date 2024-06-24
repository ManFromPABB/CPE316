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
#include "main.h"
#define __sysc_clk 4000000
#define baudrate 115200

void SystemClock_Config(void);
void UART_init(void);
void UART_send(char *data);
void UART_ESC_code(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  __enable_irq();
  UART_init();

  UART_ESC_code();

  while (1)
  {
    
  }
}

void UART_init() {
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  NVIC->ISER[1] |= (1 << 6);

  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
  GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);
  GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
  GPIOA->MODER |= GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3;
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);
  GPIOA->OTYPER &= ~(GPIO_OTYPER_ODR_2 | GPIO_OTYPER_ODR_3);

  USART2->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);
  USART2->BRR = (__sysc_clk / baudrate);
  USART2->CR1 |= USART_CR1_RXNEIE;
  USART2->CR1 |= USART_CR1_TE | USART_CR1_UE | USART_CR1_RE;
}

void UART_send(char *data) {
  while (*data != '\0') {
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = *data;
    data++;
  }
}

void UART_ESC_code(void) {
  UART_send("\x1B[37m"); // White text
  UART_send("\x1B[3B");   // Move cursor 3 lines down
  UART_send("\x1B[5C");   // Move cursor 5 columns forward
  UART_send("All good students read the ");
  UART_send("\x1B[1B");   // Move cursor 1 line down
  UART_send("\x1B[21D");  // Move cursor 21 columns backward
  UART_send("\x1B[5m");   // Set text to blink
  UART_send("Reference Manual");
  UART_send("\x1B[H");    // Move cursor to top-left corner (home)
  UART_send("\x1B[m");    // Reset text attributes
  UART_send("Input: ");
}

void USART2_IRQHandler(void) {
  char data;
  if (USART2->ISR & USART_ISR_RXNE) {
    data = USART2->RDR;
    switch (data) {
      case 'R':
        UART_send("\x1B[31m"); // Red text
        break;
      case 'G':
        UART_send("\x1B[32m"); // Green text
        break;
      case 'B':
        UART_send("\x1B[34m"); // Blue text
        break;
      case 'W':
        UART_send("\x1B[37m"); // White text
        break;
      default:
        USART2->TDR = data;
    }
    while(!(USART1->ISR & USART_ISR_TXE));
  }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
