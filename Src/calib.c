#include <LPC17xx.h>

void calib(pasos1,pasos2){
	
	uint32_t MR4=LPC_PWM1->MR4;
	
	if((pasos1)>(pasos2)){
		LPC_PWM1->MR4=(uint32_t)(MR4*1.01);
		LPC_PWM1->LER=(0x1<<4);		//Enable to change the value of MR4
	}
	if((pasos1)<(pasos2)){
		LPC_PWM1->MR4=(uint32_t)(MR4*0.99);
		LPC_PWM1->LER=(0x1<<4);		//Enable to change the value of MR4
	}
}