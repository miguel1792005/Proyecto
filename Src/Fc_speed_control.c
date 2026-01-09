#include <LPC17xx.h>

void Fc_speed_control(uint8_t duty_1,uint8_t duty_2,float *dprev1,float *dprev2,float *refvelocity_1,float *refvelocity_2){

	*refvelocity_1=(((0.2*duty_1)/98)+0.197959);
	*refvelocity_2=(((0.2*duty_2)/98)+0.197959);
	
	LPC_PWM1->MR2=(uint32_t)(6250+(((6250*(*dprev1))/100)));
	LPC_PWM1->LER|=(0x1<<2);	//	ENABLE TO CHANGE THE VALUE OF MR2
	LPC_PWM1->MR4=(uint32_t)(6250+(((6250*(*dprev2))/100)));
	LPC_PWM1->LER|=(0x1<<4);	//	ENABLE TO CHANGE THE VALUE OF MR4
	
}
