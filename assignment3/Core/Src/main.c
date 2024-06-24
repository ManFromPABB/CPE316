#include "main.h"
#define DUTY_CYCLE 50

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void TIM2_init();
void GPIO_init();

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  GPIO_init();
  TIM2_init();

  __enable_irq();

  while (1);


}


void TIM2_init() {
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable clock for TIM2

  TIM2->PSC = 20 - 1; // Prescaler
  TIM2->ARR = 20 - 1; // Auto reload
  TIM2->CR1 &= ~TIM_CR1_DIR; // Downcount

  TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk);
  TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // Enable PWM mode 1
  TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk); // Select output compare mode

  TIM2->DIER |= (TIM_DIER_CC1IE | TIM_DIER_UIE); // Enable interrupt flags
  TIM2->CCR1 = 10 - 1; // Calculate duty cycle
  TIM2->SR &= ~(TIM_SR_CC1IF); // Clear CC1IF flag
  TIM2->SR &= ~(TIM_SR_UIF); // Clear UIF flag
  TIM2->CR1 |= TIM_CR1_CEN; // Enable counter
  NVIC->ISER[0] |= 1 << TIM2_IRQn; // Enable interrupts
}


void GPIO_init() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; // Enable clock for GPIOC

  GPIOC->MODER &= ~(GPIO_MODER_MODE0_Msk);
  GPIOC->MODER |= GPIO_MODER_MODE0_0; // Output mode
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT0); // Push-pull
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_Msk); // Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk); // No pull-up, no pull-down

  GPIOA->MODER &= ~(GPIO_MODER_MODE0_Msk);
  GPIOA->MODER |= (GPIO_MODER_MODE0_1);
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT0); // Push-pull
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_Msk); // Low speed
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk); // No pull-up, no pull-down
  GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL1_Msk); // Select AF1

  GPIOC->MODER &= ~(GPIO_MODER_MODE1_Msk);
  GPIOC->MODER |= GPIO_MODER_MODE1_0; // Output mode
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT1); // Push-pull
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED1_Msk); // Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1_Msk); // No pull-up, no pull-down

  // Enable MCO, select MSI (4 MHz source)
  RCC->CFGR = ((RCC->CFGR & ~(RCC_CFGR_MCOSEL)) | (RCC_CFGR_MCOSEL_0));

  // Configure MCO output on PA8
  RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
  GPIOA->MODER   &= ~(GPIO_MODER_MODE8);		// alternate function mode
  GPIOA->MODER   |=  (2 << GPIO_MODER_MODE8_Pos);
  GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT8);		// Push-pull output
  GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD8);		// no resistor
  GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8);		// high speed
  GPIOA->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);		// select MCO function
}


void TIM2_IRQHandler() {
  GPIOC->ODR |= 1 << 1; // Toggle ISR duration pin
  if (TIM2->SR & TIM_SR_CC1IF) { // Set high->low at duty cycle
    GPIOC->ODR &= 0;
  }
  if (TIM2->SR & TIM_SR_UIF) {
    GPIOC->ODR |= 1;
  }
  TIM2->SR &= ~TIM_SR_UIF; // Clear interrupt flags
  TIM2->SR &= ~TIM_SR_CC1IF;
  GPIOC->ODR &= ~(1 << 1); // Toggle ISR duration pin
}


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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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