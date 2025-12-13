#include <LPC17xx.h>

void calib(float gain1,float gain2,uint32_t speed){
	
	LPC_PWM1->MR4=(uint32_t)(6250+(((6250*speed)/100)*gain2*0.98));	// INCREASE OR DECREASE THE VALUE 
	LPC_PWM1->LER|=(0x1<<4);	//	ENABLE TO CHANGE THE VALUE OF MR4
	LPC_PWM1->MR2=(uint32_t)(6250+(((6250*speed)/100)*gain1*1.017));	// INCREASE OR DECREASE THE VALUE
	LPC_PWM1->LER|=(0x1<<2);	//	ENABLE TO CHANGE THE VALUE OF MR2	
	
}
