#include "LPC17xx.h"

void set_distance(uint16_t D){
	
	LPC_TIM3->IR=0xFF;
	LPC_TIM3->MR0=(uint32_t)(((11*2*33.5)/(3.141592654*6.7))*D);
	LPC_TIM3->TCR=(0x1);		//Timer counter and prescaler enable to counting

}
