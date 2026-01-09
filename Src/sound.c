#include "LPC17xx.h"

void sound(uint16_t a) {
		LPC_DAC->DACR=(uint32_t) a;
}
