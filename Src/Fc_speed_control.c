#include <LPC17xx.h>

uint8_t Fc_speed_control(uint8_t speed){
	
	LPC_PWM1->MR2=(uint32_t)(6250+((6250*speed*1.017)/100));	//	WE NEED MORE SPEED ON MOTOR 2 IS MORE SLOWER
	LPC_PWM1->LER|=(0x1<<2);	//	ENABLE TO CHANGE THE VALUE OF MR2	
	LPC_PWM1->MR4=(uint32_t)(6250+((6250*speed*0.98)/100));		
	LPC_PWM1->LER|=(0x1<<4);	//	ENABLE TO CHANGE THE VALUE OF MR2
	
	return 3;
}
