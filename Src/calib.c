#include <LPC17xx.h>

void calib(float gain1,float gain2, uint32_t velocidad){
	
	LPC_PWM1->MR2 = (uint32_t)(gain1*((12500*velocidad)/100));	
	LPC_PWM1->LER |= (0x1<<2);		//Enable to change the value of MR2	
	LPC_PWM1->MR4 = (uint32_t)(gain2*((12500*velocidad)/100));	
	LPC_PWM1->LER |= (0x1<<4);		//Enable to change the value of MR2

	
}