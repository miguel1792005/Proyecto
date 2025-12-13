#include <LPC17xx.h>

void Fc_reset_Tc(){
	LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
	LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);
}
