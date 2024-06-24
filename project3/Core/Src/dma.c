#include "dma.h"

extern float32_t adc[ADC_SAMPLES];
uint8_t adc_access_complete = 0;
uint8_t adc_half_access_complete = 0;

/**
  * @brief  Set up DMA1 channel 1 to transfer ADC data to memory
  * @retval void
  */
void DMA_Init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // enable DMA1 clock

    NVIC->ISER[0] = 1 << (DMA1_Channel1_IRQn & IRQ_MASK); // enable DMA1 channel 1 interrupt

    DMA1_Channel1->CCR = 0; // reset DMA1 channel 1 control register
    DMA1_Channel1->CMAR = (uint32_t) adc; // map memory address to adc buffer
    DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR; // map peripheral address to ADC1 data register
    DMA1_Channel1->CNDTR = ADC_SAMPLES; // set number of data to transfer (ADC_SAMPLES = 2048)
    DMA1_Channel1->CCR = DMA_CCR_PL; // set priority level to high
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0; // set memory size to 16 bits
    DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0; // set peripheral size to 16 bits
    DMA1_Channel1->CCR |= DMA_CCR_MINC; // enable memory increment mode
    DMA1_Channel1->CCR |= DMA_CCR_CIRC; // enable circular mode
    DMA1_Channel1->CCR |= DMA_CCR_HTIE; // enable half transfer interrupt
    DMA1_Channel1->CCR |= DMA_CCR_TCIE; // enable transfer complete interrupt
    DMA1_Channel1->CCR |= DMA_CCR_EN; // enable DMA1 channel 1
}

/**
  * @brief  Enable half transfer and transfer complete interrupt flags for FIR lowpass filter processing
  * @retval void
  */
void DMA1_Channel1_IRQHandler() {
    if (DMA1->ISR & DMA_ISR_HTIF1) { // if half transfer interrupt flag is set
        DMA1->IFCR |= DMA_IFCR_CHTIF1; // clear half transfer interrupt flag
        adc_half_access_complete = 1;
    }
    if (DMA1->ISR & DMA_ISR_TCIF1) { // if transfer complete interrupt flag is set
        DMA1->IFCR |= DMA_IFCR_CTCIF1; // clear transfer complete interrupt flag
        adc_access_complete = 1;
    }
}