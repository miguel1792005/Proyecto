#include <LPC17xx.h>

void Fc_config_pines(void){
	//Register PINSEL
	LPC_PINCON->PINSEL3 = (LPC_PINCON->PINSEL3&~((0x3<<8)|(0x3F)|(0x3<<14))) | (0x2<<8)|(0x3<<4)|(0x2<<14);		//Clear bit 15:14/9:8/7:0, PWM1.4 pin1.23 PWM1.2 pin1.20, pin 1.16/1.17 GPIO and CAP1.0 pin 1.18
	LPC_PINCON->PINSEL2 &= ~0xF;		//Bit 1:0 GPIO pin 1.0 and 1.1
	LPC_PINCON->PINSEL0 = (LPC_PINCON->PINSEL0&~((0x3<<8)|(0x3<<2))) | (0x3<<8)|(0x2<<2);		//CAP2.0 pin0.4, RXD3 pin0.1
	
	//Register FIODIR
	LPC_GPIO1->FIODIR|=(0x3)|(0x3)<<16;		//pin 1.0 and 1.1/ pin 1.16 and 1.17 output
}