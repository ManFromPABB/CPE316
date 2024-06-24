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
#define I2C_CR2_SADD_NADD10_Pos 1
#define I2C_TIMING 0x10909CEC  // Timing configuration (100 kHz standard mode)
#define I2C_MASTER_ADDRESS 0x51
#define I2C_SLAVE_ADDRESS 0x52

void SystemClock_Config(void);
void I2C_GPIO_init(void);
void EEPROM_init(void);
void EEPROM_write(uint8_t *data, uint8_t i2c_address, uint8_t *write_address, uint8_t size);
void EEPROM_read(uint8_t data[], uint8_t i2c_address, uint8_t *read_address, uint8_t size);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  I2C_GPIO_init();
  EEPROM_init();

  uint8_t write_address[2] = {0xe2, 0xf9};

  uint8_t *data = "a";
  EEPROM_write(data, I2C_SLAVE_ADDRESS, write_address, 1);
  for (uint32_t i = 0; i < 1000000; i++);
  uint8_t data_r;
  EEPROM_read(&data_r, I2C_SLAVE_ADDRESS, write_address, 1);

  while (1) {

  }

}

void I2C_GPIO_init(void) {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // Enable GPIOB clock
  GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
  GPIOB->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1; // Set PB6 and PB7 to alternate function mode
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7; // Set PB6 and PB7 to very high speed
  GPIOB->OTYPER |= GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7; // Set PB6 and PB7 to open-drain
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
  GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7); // Enable AF4 for I2C1 on PB6 and PB7
  GPIOB->AFR[0] |= 4 << GPIO_AFRL_AFSEL6_Pos | 4 << GPIO_AFRL_AFSEL7_Pos; // Enable AF4 for I2C1 on PB6 and PB7
}

void EEPROM_init() {
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; // Enable I2C1 clock
  I2C1->CR1 = 0;
  I2C1->CR2 = 0;
  I2C1->OAR1 = I2C_MASTER_ADDRESS << I2C_OAR1_OA1_Pos; // Set own address to 0x51
  I2C1->CR2 &= ~I2C_CR2_ADD10; // Enable 7-bit addressing
  I2C1->TIMINGR = I2C_TIMING; // Set timing configuration
  I2C1->CR2 |= I2C_CR2_AUTOEND; // Enable automatic end mode
  I2C1->CR1 |= I2C_CR1_PE; // Enable I2C1 peripheral
}

void EEPROM_write(uint8_t *data, uint8_t i2c_address, uint8_t *write_address, uint8_t size) {
  // while (!(I2C1->ISR & I2C_ISR_BUSY)); // Wait for BUSY flag to be cleared
  I2C1->CR2 &= ~I2C_CR2_NBYTES;
  I2C1->CR2 |= (size + 2) << I2C_CR2_NBYTES_Pos; // Set number of bytes to write
  I2C1->CR2 |= (i2c_address << 1); // Set slave address
  I2C1->CR2 &= ~I2C_CR2_RD_WRN;  // Set to write mode
  I2C1->CR2 |= I2C_CR2_START; // Generate start condition

  // Transmit the address
  for (uint8_t i = 0; i < 2; i++) {
    while (!(I2C1->ISR & I2C_ISR_TXIS)); // Wait for TXIS flag to be set
    I2C1->TXDR = write_address[i]; // Write data to TXDR
  }

  // Transmit the data
  for (uint8_t i = 0; i < size; i++) {
    while (!(I2C1->ISR & I2C_ISR_TXIS)); // Wait for TXIS flag to be set
    I2C1->TXDR = data[i]; // Write data to TXDR
  }
}

void EEPROM_read(uint8_t data[], uint8_t i2c_address, uint8_t *read_address, uint8_t size) {
  // while (!(I2C1->ISR & I2C_ISR_BUSY)); // Wait for BUSY flag to be cleared
  I2C1->CR2 &= ~I2C_CR2_NBYTES;
  I2C1->CR2 |= 2 << I2C_CR2_NBYTES_Pos; // Set number of bytes to read
  I2C1->CR2 |= (i2c_address << 1); // Set slave address
  I2C1->CR2 &= ~I2C_CR2_RD_WRN;  // Set to read mode
  I2C1->CR2 |= I2C_CR2_START; // Generate start condition

  for (uint8_t i = 0; i < 2; i++) {
    while (!(I2C1->ISR & I2C_ISR_TXIS)); // Wait for TXIS flag to be set
    I2C1->TXDR = read_address[i]; // Write data to TXDR
  }

  for (uint32_t i = 0; i < 1000000; i++);
  
  I2C1->CR2 &= ~I2C_CR2_NBYTES;
  I2C1->CR2 |= size << I2C_CR2_NBYTES_Pos; // Set number of bytes to read
  I2C1->CR2 |= I2C_CR2_RD_WRN;  // Set to read mode
  I2C1->CR2 |= I2C_CR2_START; // Generate start condition

  for (uint8_t i = 0; i < size; i++) {
    while (!(I2C1->ISR & I2C_ISR_RXNE)); // Wait for RXNE flag to be set
    data[i] = I2C1->RXDR; // Read data from RXDR
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
