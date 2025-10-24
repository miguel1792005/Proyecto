#include <LPC17xx.h>

void Fc_config_TIMER(void){
	LPC_SC->PCONP|=(1<<2)|(1<<22);		//start Timer1 and Timer2
	//TIMER1
	LPC_TIM1->TCR=(0x1<<1);		//Reset Timer1 counter and prescaler and disable
	LPC_TIM1->CTCR=(LPC_TIM1->CTCR&~(0xF))|(0x3);		//Increment TC with both edges (counter mode) and used CAP1.0 pin input
	LPC_TIM1->CCR=0x3;		//When some edge comes, generate interrupt on TIMER1
	LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
	//TIMER2
	LPC_TIM2->TCR=(0x1<<1);		//Reset Timer2 counter and prescaler and disable
	LPC_TIM2->CTCR=(LPC_TIM1->CTCR&~(0xF))|(0x3);		//Increment TC with both edges (counter mode) and used CAP2.0 pin input
	LPC_TIM2->CCR=0x3;		//When some edge comes, generate interrupt on TIMER2
	LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
	
	
	
	LPC_TIM1->MR0 = 2; // Set MR0 according 1/11 revolution to calibrate the speed
	LPC_TIM1->MCR = (0x1 << 0); // Interrupt on MR0 match
	
	
	
	
	
}