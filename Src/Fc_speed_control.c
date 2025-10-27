#include <LPC17xx.h>

uint8_t Fc_speed_control(uint8_t speed){
	
	LPC_PWM1->MR2 = (uint32_t)((2000*speed)/100);	//When speed is 99 we have problems, MR2>MR0 thats why the speed *1.0101 (we need more speed on motor 2 is more slower)
	LPC_PWM1->LER |= (0x1<<2);		//Enable to change the value of MR2	
	LPC_PWM1->MR4 = (uint32_t)((2000*speed)/100);		
	LPC_PWM1->LER |= (0x1<<4);		//Enable to change the value of MR2
	
	return 3;
}