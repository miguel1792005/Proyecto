#ifndef FC_DISPLAY_MESSAGE
#define FC_DISPLAY_MESSAGE

#include <LPC17xx.h>

void Fc_display_message(const char *text,uint16_t Xpos,uint16_t Ypos,char *lcd_buffer,uint8_t distance);

#endif
