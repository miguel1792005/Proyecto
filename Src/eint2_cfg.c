#include "LPC17xx.h"

void eint2_cfg(){
	LPC_SC->EXTMODE|=0x4; //EINT2 edge sensitive
	LPC_SC->EXTPOLAR&=~(0x4); //EINT2 rising-edge sensitive
	NVIC_EnableIRQ(EINT2_IRQn);
	NVIC_SetPriority(EINT2_IRQn,0);
}
