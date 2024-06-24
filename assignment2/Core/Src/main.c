#include "main.h"
#define KEYPAD_STAR 10
#define KEYPAD_POUND 12
#define KEYPAD_NO_PRESS 0xFF
#define KEYPAD_NUM_ROWS 4
#define KEYPAD_NUM_COLS 3
#define KEYPAD_ROW_OFF  0
#define KEYPAD_COL_OFF KEYPAD_NUM_ROWS
#define LED_OFF KEYPAD_NUM_ROWS + KEYPAD_NUM_COLS


void SystemClock_Config(void);
void GPIO_Init(void);
uint8_t Keypad_Read(void);
void LED_Write(uint8_t);

int main(void) {
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  GPIO_Init();

  /* Infinite loop */
  uint8_t key;
  while (1) {
	  key = Keypad_Read();

	  if (key != KEYPAD_NO_PRESS) {
		  LED_Write(key);
	  }
  }
}

void GPIO_Init() {

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
			GPIO_MODER_MODE6 | GPIO_MODER_MODE7 | GPIO_MODER_MODE8 |
			GPIO_MODER_MODE9 | GPIO_MODER_MODE10);

	GPIOC->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 |
			GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0 | GPIO_MODER_MODE8_0 |
			GPIO_MODER_MODE9_0 | GPIO_MODER_MODE10_0);

	// output type PP for keypad col and LED pins (outputs)
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 |
			GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8 |
			GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10);

	// pull down resistor on for keypad row pins (inputs), PUPD off for keypad col and LED pins (outputs)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2 |
			GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 |
			GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8 |
			GPIO_PUPDR_PUPD9 | GPIO_PUPDR_PUPD10);

	GPIOC->PUPDR |= (GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1 | GPIO_PUPDR_PUPD2_1 |
				GPIO_PUPDR_PUPD3_1);

	// slow speed for keypad col and LED pins (outputs)
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5 |
			GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7 | GPIO_OSPEEDR_OSPEED8 |
			GPIO_OSPEEDR_OSPEED9 | GPIO_OSPEEDR_OSPEED10);

	// initialize keypad cols outputs to '1' and LEDs to 0
	GPIOC->ODR &= ~(GPIO_ODR_OD7 | GPIO_ODR_OD8 | GPIO_ODR_OD9 | GPIO_ODR_OD10);
	GPIOC->ODR |= (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6);
}


uint8_t Keypad_Read() {

  GPIOC->ODR |= (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6); // set all cols to '1'

	if (!(GPIOC->IDR&0xF)) {
    return KEYPAD_NO_PRESS;
  }

	int32_t col=0;
	while (col < KEYPAD_NUM_COLS) {

		GPIOC->ODR &= ~(GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6);
		GPIOC->ODR |= 1 << (KEYPAD_COL_OFF+col);

		if (GPIOC->IDR&0xF) {
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

void LED_Write(uint8_t key) {
	GPIOC->ODR &= ~((0xF) << GPIO_ODR_OD7_Pos); // clear these 4 pins
	GPIOC->ODR |= (key & 0xF) << GPIO_ODR_OD7_Pos; // set these pins to 4 LSB of key
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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