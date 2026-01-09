#include <LPC17xx.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "AsciiLib.h"
#include "GLCD.h" 

void Fc_display_message(const char *text,uint16_t Xpos,uint16_t Ypos,char *lcd_buffer,uint8_t distance){
	sprintf(lcd_buffer,text,distance);
	GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, Black, Cyan);
}