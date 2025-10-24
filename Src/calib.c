#include <LPC17xx.h>

void calib(float gain, uint32_t speed){
	
	LPC_PWM1->MR4 = (uint32_t)(gain*((12500*speed)/100));	// Increase or Decrease the value 
	LPC_PWM1->LER |= (0x1<<4);		//Enable to change the value of MR2
	
}