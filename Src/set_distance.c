#include "LPC17xx.h"

void set_distance(uint8_t D){


	LPC_TIM2->MR0 = (uint32_t)(((2*11)/(3.1415*6.7))*D); // Set MR0 according distance
	LPC_TIM2->MCR = (0x3 << 0); // Clear counter and trigger interrupt on MR0 match



}
