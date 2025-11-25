#include "LPC17xx.h"
#include <math.h>

#define N_POINTS 16

void sound(uint16_t a) {
		LPC_DAC->DACR=(uint32_t) a;
}
