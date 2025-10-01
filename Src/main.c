#include "LPC17xx.h"
#include "PWM.h"


int main(){
	
	uint16_t velocity;
	
	velocity=90;
	
	configPWM(velocity);
	
	
	while(1);
	
	return 0;
	
}
