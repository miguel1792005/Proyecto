#include "LPC17xx.h"

#define	Kp	300.0f
#define	Ki	300.0f
#define	Kd	50.0f
#define	Ts	0.1f

float pid1(float error, float errorprev, float *accumulated_integra_1){
	
	float dnew=0;
	*accumulated_integra_1=*accumulated_integra_1+Ki*error*Ts;
	
	dnew=Kp*error+*accumulated_integra_1+Kd*((error-errorprev)/Ts);
	
	if(dnew>100) dnew=100;
	
	LPC_PWM1->MR2=(uint32_t)(6250+((6250*dnew)/100));
	LPC_PWM1->LER|=(0x1<<2);	//	ENABLE TO CHANGE THE VALUE OF MR2
	
	return dnew;
	
}
