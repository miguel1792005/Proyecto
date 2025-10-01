#include "LPC17xx.h"
#include "PWM.h"

#define FCLK SystemCoreClock
#define FMotor 1000000

void configPWM(uint16_t duty){
	
	LPC_PINCON->PINSEL3 &= ~(0x3<<8);
	LPC_PINCON->PINSEL3 |= (0x2<<8); //Set as PWM1.2 P1.20
	
	
	LPC_SC->PCONP |= (0x1<<6); //Power on PWM1
	
	LPC_PWM1->MR0 = (uint32_t)(FCLK/FMotor)-1; //Set frequency of PWM1
	LPC_PWM1->PCR |= 0x1<<10;
	LPC_PWM1->MCR |= 0x1<<1;
	LPC_PWM1->TCR |= (0x1)|(0x1<<3);
	
	
	LPC_PWM1->MR2 = ((uint32_t)(duty*(FCLK/FMotor))-1)/100;
	
	LPC_PWM1->LER |= (0x1<<2)|(0x1<<0);
	
	
}