#include <LPC17xx.h>

void calib(float gain1,float gain2,uint32_t speed){
	
	LPC_PWM1->MR4=(uint32_t)(((2000*speed)/100)*gain2);	// Increase or Decrease the value 
	LPC_PWM1->LER|=(0x1<<4);		//Enable to change the value of MR4
	LPC_PWM1->MR2=(uint32_t)(((2000*speed)/100)*gain1);	// Increase or Decrease the value
	LPC_PWM1->LER|=(0x1<<2);		//Enable to change the value of MR2	
	
}