#include <LPC17xx.h>

uint8_t Fc_speed_control(uint8_t velocidad){
	
	LPC_PWM1->MR2=((12500*velocidad)/100);		
	LPC_PWM1->MR4=((12500*velocidad)/100);	
	LPC_PWM1->LER=(0x1<<2)|(0x1<<4);		//Enable to change the value of MR2
	
	return 3;
}