#include <LPC17xx.h>

void Fc_config_IRQ(void){
	
	//Prigroup
	NVIC_SetPriorityGrouping(3);
	//Priority
	NVIC_SetPriority(TIMER1_IRQn, 4);		
	NVIC_SetPriority(TIMER2_IRQn, 5);
	NVIC_SetPriority(TIMER3_IRQn, 0);
	//Enable
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);
	
}