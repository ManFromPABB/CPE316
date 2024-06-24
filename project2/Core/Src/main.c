#include "main.h"
#include "fft.h"
#include "uart.h"
#include "adc.h"

void SystemClock_Config(void);

extern arm_rfft_fast_instance_f32 fft_instance;
extern float32_t fft_input[FFT_SIZE];
extern uint16_t adc_count;

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
  UART_init_terminal(); // Initialize terminal
  arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE); // Initialize FFT

  while (1) {
    ADC1->CR |= ADC_CR_ADSTART; // start ADC conversion
    if (adc_count == FFT_SIZE) { // check when ADC conversion complete
      ADC_stop(); // disable ADC

      float64_t mag_square_sum = 0;
      for (int i = 0; i < FFT_SIZE; i++) {
        mag_square_sum += fft_input[i] * fft_input[i]; // calculate sum of squares of input samples
      }
      float64_t mean_squared = mag_square_sum / FFT_SIZE; // divide by number of samples to get mean squared
      float64_t rms = sqrt(mean_squared) * MAX_VOLTAGE / MAX_ADC; // sqrt to get RMS value and scale to voltage

      uint32_t frequency = (uint32_t) FFT_calc(); // calculate frequency of input signal

      uint32_t max = find_max(); // find max value in input samples
      uint32_t min = find_min(); // find min value in input samples
      float32_t vpp = (max - min) * MAX_VOLTAGE / MAX_ADC; // calculate peak to peak voltage

      uint8_t rms_string[5];
      uint8_t vpp_string[5];
      float_to_string(vpp_string, vpp); // convert Vpp value to string  
      uint8_t freq_buff[5];
      float_to_string(rms_string, rms); // convert RMS value to string

      uint8_t *freq_string;
      if (frequency) {
        freq_string = int_to_string(frequency, freq_buff); // convert frequency to string
      } else {
        freq_string = (uint8_t *) "DC\0"; // display DC if signal is DC
      }

      uint8_t num_bars = (uint8_t) round(rms / 0.1); // calculate number of bars to display on terminal
      UART_update_terminal(freq_string, rms_string, num_bars, vpp_string); // update terminal
      
      ADC_init(); // restart ADC
      adc_count = 0; // reset ADC sample count
    }
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
