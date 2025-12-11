#include "LPC17xx.h"
#define	F_CCLK	(SystemCoreClock)
#define F_PCLK	(SystemCoreClock/4)
#define FREQ

void adc_cfg(void) {
	//uint32_t div;   REVISAR Y BORRAR SI NO ES NECESARIO
	LPC_SC->PCLKSEL0&=~(0x3<<24); // Clear bits 25:24 ...
	LPC_SC->PCLKSEL0|=0x00;	// PCLK_ADC = CCLK/4
	LPC_SC->PCONP|=0x1<<12;	// ADC 0 power on
	LPC_PINCON->PINSEL1|=(0x01<<16); // P0.24 pin as AD0.1 input
	LPC_PINCON->PINMODE1|=(0x2<<16);	// P0.24: disable pull-up & down
	LPC_ADC->ADCR=(0x01U<<1)|(0x01U<<8)|(0x04U<<24) ; //	channel select | power on | START on MAT0.1
	LPC_ADC->ADINTEN=0x1<<1;
	// JUST AD0.1 interrupt enable
	LPC_ADC->ADCR|=(0x01U<<21); //	power on
}

