#include "LPC17xx.h"

void eint0_cfg(){
	LPC_SC->EXTMODE|=0x2; //EINT1 edge sensitive
	LPC_SC->EXTPOLAR&=~(0x2); //EINT1 rising-edge sensitive
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_SetPriority(EINT1_IRQn,0);
}
