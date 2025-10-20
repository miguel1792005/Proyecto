#include <LPC17xx.h>

void calib(uint32_t pasos1,uint32_t pasos2){
	
	uint32_t MR4=LPC_PWM1->MR4;
	uint32_t gain=pasos1/pasos2;
	
	MR4*=MR4;
	LPC_PWM1->MR4=MR4;	
	LPC_PWM1->LER=(0x1<<2)|(0x1<<4);		//Enable to change the value of MR2
	
}