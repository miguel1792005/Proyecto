#include <LPC17xx.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "AsciiLib.h"
#include "GLCD.h" 

#define FONT_W  8
#define FONT_H  16

void Fc_display_variable_value(uint32_t speed1,uint32_t speed2,float voltage,uint16_t Xpos,uint16_t Ypos,int line,int dist_cm,int angle,char *lcd_buffer){
  line = 0;
	Xpos = 80; Ypos = line*FONT_H;
	sprintf(lcd_buffer, "Velocity MOTOR 1: %.2f m/s", (float)(5.5/speed1));//5.5/speed1= velocity in m/s
	GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, Black, Cyan);

	line = 1;
	Xpos = 80; Ypos = line*FONT_H;
	sprintf(lcd_buffer, "Velocity MOTOR 2: %.2f m/s", (float)(5.5/speed2));//5.5/speed2=velocity in m/s
	GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, Black, Cyan);

  line = 2;
  Xpos = 80; Ypos = line*FONT_H;
  sprintf(lcd_buffer, "VOLTAGE: %.2fV", voltage);
  GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, Black, Cyan);
}
