#include <LPC17xx.h>
#include <math.h>	//	LIBRARY FOR SINF

#define PI 3.1415926535897932384626433832795f
#define N_POINTS 20
#define DAC_N_BITS 10
#define DAC_MID_RANGE (1U << (DAC_N_BITS-1))


uint16_t tone_init_samples(uint8_t i) {
	
	float x;
	uint16_t sample_table=0;
	
	x=DAC_MID_RANGE+(DAC_MID_RANGE-1)*sinf((2*PI/(N_POINTS))*i);
	sample_table=((uint16_t)x)<<6;
	
	return sample_table;
}
