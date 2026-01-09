#include <LPC17xx.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "AsciiLib.h"
#include "GLCD.h" 

#define FONT_H  16

void Fc_display_variable_value(float velocity1,float velocity2,float voltage, uint8_t distance,char *lcd_buffer){
	uint16_t Xpos;
	uint16_t Ypos;
	
	Xpos = 40; Ypos = 0*FONT_H;
	sprintf(lcd_buffer, "Velocity MOTOR 1: %.2f m/s", (float)(velocity1));	//	5.5/SPEED1= VELOCITY IN M/S 
	GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, Black, Cyan);

	Xpos = 40; Ypos = 1*FONT_H;
	sprintf(lcd_buffer, "Velocity MOTOR 2: %.2f m/s", (float)(velocity2));	//	5.5/SPEED2=VELOCITY IN M/S
	GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, Black, Cyan);

	Xpos = 40; Ypos = 2*FONT_H;
  sprintf(lcd_buffer, "DISTANCE: %d cm      ", distance);	//	DISTANCE 
  GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, Black, Cyan);
	
  Xpos = 40; Ypos = 3*FONT_H;
  sprintf(lcd_buffer, "VOLTAGE: %.2fV", voltage);	//	VOLTAGE 
  GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, Black, Cyan);
	
	
}
