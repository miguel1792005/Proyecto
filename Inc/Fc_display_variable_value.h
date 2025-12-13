#ifndef FC_DISPLAY_VARIABLE_VALUE
#define FC_DISPLAY_VARIABLE_VALUE

#include <LPC17xx.h>

void Fc_display_variable_value(uint32_t speed1,uint32_t speed2,float voltage,uint16_t Xpos,uint16_t Ypos,int line,int dist_cm,int angle,char *lcd_buffer);

#endif
