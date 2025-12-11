#include <LPC17xx.h>
#define T_ADC 0.1f

void Fc_config_TIMER(void){
	
	//TIMER0 for ADC conversions and signal generator DAC
	LPC_SC->PCONP|=(1<<1);		//start Timer0 
	LPC_SC->PCLKSEL0&=~(0x3<<2);		//Ftimer1 fclk/4
	LPC_TIM0->TCR=(0x1<<1);		//Reset Timer0 counter and prescaler and disable
	LPC_TIM0->PR=0;
	LPC_TIM0->CTCR&=0x0;		//Increment TC in timer mode
	LPC_TIM0->MCR|=(0x3<<6)|(0x1);		//Reset TC when MR2 and interrupt on MR2 and MR0
	LPC_TIM0->MR2=12499999;		//0.5 secs with each letter
	LPC_TIM0->MR1=12499999;	//Conversion ADC 1 sec(toggle 0.5-0.5)
	LPC_TIM0->MR0=4788;		//Start with DO	20 samples
	LPC_TIM0->EMR|=(0x2|(0x3<<6));
	LPC_TIM0->IR=0x3F;
	LPC_TIM0->TCR=0x1;//Start timer

	//TIMER1 take the freqency of motor1
	LPC_SC->PCONP|=(1<<2)|(1<<22)|(1<<23);		//start Timer1 and Timer2
	LPC_SC->PCLKSEL0=(LPC_SC->PCLKSEL0&~(0x3<<4))|(0x2<<4);		//Ftimer1 fclk/2
	LPC_TIM1->TCR=(0x1<<1);		//Reset Timer1 counter and prescaler and disable
	LPC_TIM1->PR=0x1388;		//5000=1TC
	LPC_TIM1->CTCR&=~(0xF);		//Increment TC with pck freqency (timer mode) and used CAP1.0 pin input
	LPC_TIM1->CCR|=0x5;		//Load the value of TC on CR0 and generate interrupt rising edge
	LPC_TIM1->IR=0x3F;
	LPC_TIM1->TCR=0x1;//Start timer

	//TIMER2 take the freqency of motor2
	LPC_SC->PCLKSEL1=(LPC_SC->PCLKSEL1&~(0x3<<12))|(0x2<<12);		//Ftimer2 fclk/2
	LPC_TIM2->TCR=(0x1<<1);		//Reset Timer1 counter and prescaler and disable
	LPC_TIM2->PR=0x1388;		//5000=1TC
	LPC_TIM2->CTCR&=~(0xF);		//Increment TC with pck freqency (timer mode) and used CAP1.0 pin input
	LPC_TIM2->CCR|=0x5;		//Load the value of TC on CR0 and generate interrupt rising edge
	LPC_TIM2->IR=0x3F;
	LPC_TIM2->TCR=0x1;//Start timer

	//TIMER3	Set distance
	LPC_SC->PCLKSEL1=(LPC_SC->PCLKSEL1&~(0x3<<14))|(0x2<<14);		//Ftimer2 fclk/2
	LPC_TIM3->TCR=(0x1<<1);		//Reset Timer3 counter and prescaler and disable
	LPC_TIM3->CTCR=(LPC_TIM2->CTCR&~(0xF))|(0x3);		//Increment TC with both edges (counter mode) and used CAP3.0 pin input
	LPC_TIM3->MCR=0x3; // Clear counter trigger interrupt on MR0 match 
	LPC_TIM3->IR=0x3F;
	LPC_TIM3->TCR=0x1;		//Timer counter and prescaler enable to counting	
}
