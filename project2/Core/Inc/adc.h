#ifndef PROJECT2_ADC_H
#define PROJECT2_ADC_H

#define CALIBRATION_SCALE 1
#define CALIBRATION_OFFSET 18
#define MAX_ADC 4095
#define IRQ_MASK 0x1F
#define MAX_VOLTAGE 3.3
#define ADC_WAKEUP_DELAY 80
#define ADC_CALIBRATION_DELAY 1000

void ADC_init(void);
void ADC_stop(void);
void ADC1_2_IRQHandler(void);

#endif