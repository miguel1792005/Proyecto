#include "LPC17xx.h"

void set_distance(uint16_t D){

	LPC_TIM2->MR0 = (uint32_t)(((11*2*33.5)/(3.141592654*6.7))*D); // Set MR0 according distance    
	LPC_TIM2->MCR = (0x3 << 0); // Clear counter and trigger interrupt on MR0 match

	LPC_TIM1->TC=0;
	LPC_TIM1->MR0 = 2; // Set MR0 according 1/11 revolution to calibrate the speed
	LPC_TIM2->TC=0; // Possible error after the stop of the wheels

}
