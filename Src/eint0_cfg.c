#include "LPC17xx.h"

void eint0_cfg(){
	
	LPC_SC->EXTMODE|=0x1; //EINT0 edge sensitive
	
	LPC_SC->EXTPOLAR|=0x1; //EINT0 rising-edge sensitive
	
	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_SetPriority(EINT0_IRQn,0);
	
}
