#ifndef FC_MOTOR1
#define FC_MOTOR1

#include <LPC17xx.h>

void Fc_speed_control(uint8_t duty_1,uint8_t duty_2,float *dprev1,float *dprev2,float *refvelocity_1,float *refvelocity_2);

#endif
