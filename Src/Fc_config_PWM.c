#include <LPC17xx.h>

void Fc_config_PWM(void){
	
	LPC_SC->PCLKSEL0=(LPC_SC->PCLKSEL0&~(0x3<<12))|(0x2<<12);		//Clear bits 3:2 and pclk_peripheral pwm1=cclk/2
	LPC_SC->PCONP|=(1<<6);		//start PWM1
	LPC_PWM1->TCR=(0x1<<1); //Reset PWM counter and prescaler
	LPC_PWM1->CTCR&=~(0x3);		//PWM to Counting mode
	LPC_PWM1->MCR=(0x1<<1);		//Reset PWM counter when the value of MR0=PWM Counter
	LPC_PWM1->MR0=25000;  //1000=1ms
	LPC_PWM1->LER=(0x1<<0);		//Enable to change the value of MR0 
	LPC_PWM1->MR2=0;	
	LPC_PWM1->LER|=(0x1<<2);		//Enable to change the value of MR2
	LPC_PWM1->MR4=0;
	LPC_PWM1->LER|=(0x1<<4);		//Enable to change the value of MR4
	LPC_PWM1->TCR=(0x9<<0); //PWM counter and prescaler enable to counting
	LPC_PWM1->PCR|=((1<<10)|(1<<12)); //Enable source
	
}