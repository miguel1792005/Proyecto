#include "LPC17xx.h"

void dac_cfg() {
		LPC_SC->PCLKSEL0&=~0x00C00000U;
		LPC_SC->PCLKSEL0|=0x00000000U;	// PCLK_DAC=CCLK/4 (25 MHz)
		LPC_PINCON->PINSEL1|=(0x2<<20); // DAC output=P0.26 (AOUT)
		LPC_PINCON->PINMODE1|=(0x2<<20); // disable pull up & down
		LPC_DAC->DACCTRL=0;
}
