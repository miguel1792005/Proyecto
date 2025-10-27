#include <LPC17xx.h>

void Fc_config_TIMER(void){
	
	LPC_SC->PCONP|=(1<<2)|(1<<22)|(1<<23);		//start Timer1 and Timer2
	//TIMER1 take the freqency of motor1
	LPC_SC->PCLKSEL0=(LPC_SC->PCLKSEL0&~(0x3<<4))|(0x2<<4);		//Ftimer1 fclk/2
	LPC_TIM1->TCR=(0x1<<1);		//Reset Timer1 counter and prescaler and disable
	LPC_TIM1->PR=0x1388;		//5000=1TC
	LPC_TIM1->CTCR&=~(0xF);		//Increment TC with pck freqency (timer mode) and used CAP1.0 pin input
	LPC_TIM1->CCR|=(0x5);		//Load the value of TC on CR0 and generate interrupt rising edge
	LPC_TIM1->IR=0x3F;

	//TIMER2 take the freqency of motor2
	LPC_SC->PCLKSEL1=(LPC_SC->PCLKSEL1&~(0x3<<12))|(0x2<<12);		//Ftimer2 fclk/2
	LPC_TIM2->TCR=(0x1<<1);		//Reset Timer1 counter and prescaler and disable
	LPC_TIM2->PR=0x1388;		//5000=1TC
	LPC_TIM2->CTCR&=~(0xF);		//Increment TC with pck freqency (timer mode) and used CAP1.0 pin input
	LPC_TIM2->CCR|=(0x5);		//Load the value of TC on CR0 and generate interrupt rising edge
	LPC_TIM2->IR=0x3F;

	//TIMER3	Set distance
	LPC_SC->PCLKSEL1=(LPC_SC->PCLKSEL1&~(0x3<<14))|(0x2<<14);		//Ftimer2 fclk/2
	LPC_TIM3->TCR=(0x1<<1);		//Reset Timer3 counter and prescaler and disable
	LPC_TIM3->CTCR=(LPC_TIM2->CTCR&~(0xF))|(0x3);		//Increment TC with both edges (counter mode) and used CAP3.0 pin input
	LPC_TIM3->MCR=(0x3); // Clear counter trigger interrupt on MR0 match 
	LPC_TIM3->IR=0x3F;
	LPC_TIM3->TCR=(0x1);		//Timer counter and prescaler enable to counting
	
}