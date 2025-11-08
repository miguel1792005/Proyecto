#include "LPC17xx.h"


#define	F_CCLK	(SystemCoreClock)
#define F_PCLK	(SystemCoreClock/4)

/*

void adc_cfg_int_cnv(void) {
	uint32_t div;
	LPC_SC->PCLKSEL0 &= ~(0x3<<24); // Clear bits 25:24 ...
	LPC_SC->PCLKSEL0 |= 0x00;	// PCLK_ADC = CCLK/4
	LPC_SC->PCONP |= 0x1<<12;	// ADC 0 power on
	LPC_PINCON->PINSEL1 |= (0x01 << 14); // P0.23 pin as AD0.0 input
	LPC_PINCON->PINMODE1 |= (0x2 << 14);	// P0.25: disable pull-up & down
	div = (F_PCLK+FREQ_ADC_MAX-1)/(FREQ_ADC_MAX)            DIV_ROUND_UP((uint32_t)F_PCLK, (uint32_t)FREQ_ADC_MAX);
	LPC_ADC->ADCR = (0x01U << 0) |
	ADC_CLK_DIV(div-1)
	|
	ADC_MAT1_1_START |
	ADC_NOT_PDN; //	channel select | power on
	LPC_ADC->ADINTEN = ADGINTEN;
	// JUST Global interrupt enable
	NVIC_SetPriority(ADC_IRQn, 5);
	NVIC_EnableIRQ(ADC_IRQn);
}

*/