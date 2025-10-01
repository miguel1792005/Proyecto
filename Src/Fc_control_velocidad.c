#include <LPC17xx.h>

uint8_t Fc_control_velocidad(uint8_t velocidad){
	
	LPC_PWM1->MR2=((500000*velocidad)/100);		
	LPC_PWM1->LER=(0x1<<2);		//Enable to change the value of MR1
	
	return 3;
}