#include "LPC17xx.h"

#define PI 3.1415926535897932384626433832795f
#define K1 ((11*2*33.5)/(PI*6.7))

void set_distance(uint16_t D){
	
	LPC_TIM3->IR=0xFF;
	LPC_TIM3->MR0=(uint32_t)(K1*D);
	LPC_TIM3->TCR=(0x1);		//Timer counter and prescaler enable to counting

}
