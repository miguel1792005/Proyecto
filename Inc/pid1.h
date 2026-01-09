#ifndef PID1
#define PID1

#include <LPC17xx.h>

float pid1(float error, float errorprev, float *accumulated_integra_1);

#endif
