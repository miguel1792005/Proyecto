#ifndef PID2
#define PID2

#include <LPC17xx.h>

float pid2(float error, float errorprev, float *accumulated_integra_2);

#endif
