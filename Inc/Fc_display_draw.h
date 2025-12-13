#ifndef FC_DISPLAY_DRAW
#define FC_DISPLAY_DRAW

#include <LPC17xx.h>

void Fc_display_draw(uint8_t sel_song,uint16_t Xpos,uint16_t Ypos,int line,int dist_cm,int angle,char *lcd_buffer);

#endif
