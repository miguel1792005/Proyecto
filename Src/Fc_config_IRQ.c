#include <LPC17xx.h>

void Fc_config_IRQ(void){
	
	//Prigroup
	NVIC_SetPriorityGrouping(3);
	//Priority
	NVIC_SetPriority(TIMER1_IRQn, 4);		
	NVIC_SetPriority(TIMER2_IRQn, 5);
	NVIC_SetPriority(TIMER3_IRQn, 0);
	NVIC_SetPriority(PWM1_IRQn, 1);
	//Enable
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);
	NVIC_EnableIRQ(PWM1_IRQn);
	
	
	LPC_PWM1->TCR=(0x9<<0); //PWM counter and prescaler enable to counting
	
}