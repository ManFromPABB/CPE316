#include "keypad.h"

/**
  * @brief  Initialize keypad GPIO pins.
  * @retval void
  */
void Keypad_init() {
	// initialize clock for GPIO port C
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOCEN);

	/*
	 * Pin 0-3: keypad rows
	 * Pin 4-6: keypad cols
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

/**
  * @brief  Read pressed value on keypad.
  * @retval uint8_t
  */
uint8_t Keypad_read() {

  	GPIOC->ODR |= (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6); // set all cols to '1'

	if (!(GPIOC->IDR & 0xF)) { // Check if any key is pressed
    	return KEYPAD_NO_PRESS;
  	}

	int32_t col = 0;
	while (col < KEYPAD_NUM_COLS) {

		GPIOC->ODR &= ~(GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6);

		GPIOC->ODR |= 1 << (KEYPAD_COL_OFF+col); // Enable column to check its status

		if (GPIOC->IDR & 0xF) {
			GPIOC->ODR |= (GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6); // set all cols to '1'
			if (GPIOC->IDR & 1 ) { // row 0
				return col + 1;
			}
			else if (GPIOC->IDR & 2) { // row 1
				return col + KEYPAD_NUM_COLS + 1;
			}
			else if (GPIOC->IDR & 4) { // row 2
				return col + 2 * KEYPAD_NUM_COLS + 1;
			}
			else { // row 3
				if (!col) {
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
	col++; // Go to next column if press not found
	}
	return KEYPAD_NO_PRESS;
}

/**
  * @brief  Changes device settings based on keypad input.
  * @param  key: keypad input
  * @param  duty_cycle: pointer to duty cycle
  * @param  frequency: pointer to frequency
  * @param  waveform: pointer to type of waveform
  * @retval void
  */
void Keypad_parse(uint8_t key, uint8_t *duty_cycle, uint16_t *frequency, enum waveform *waveform) {
	if ((key == KEYPAD_STAR) & (*duty_cycle > 10) & (*waveform == SQUARE)) { // If in square wave mode, decrease duty cycle
		*duty_cycle -= 10;
		return;
	}
	else if ((key == KEYPAD_POUND) & (*duty_cycle < 90) & (*waveform == SQUARE)) { // If in square wave mode, increase duty cycle
		*duty_cycle += 10;
		return;
	}
	else if ((key == 0)) { // Restore default duty cycle
		*duty_cycle = 50;
		return;
	}
	else if ((key > 0) & (key < 6)) {
		*frequency = key; // Change frequency
		return;
	}
	switch (key) {
		case 6:
			*waveform = SINE; // Change wave to sine
			return;
		case 7:
			*waveform = TRIANGLE; // Change wave to triangle
			return;
		case 8:
			*waveform = SAWTOOTH; // Change wave to sawtooth
			return;
		case 9:
			*waveform = SQUARE; // Change wave to square
			return;
	}
}