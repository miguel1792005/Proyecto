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

void Fc_display_draw(uint8_t p,uint8_t sel_song,uint16_t Xpos,uint16_t Ypos,int line,int dist_cm,int angle,char *lcd_buffer){
	for(p=0;p<100;p++){
		switch(sel_song){
		case 0:
			LCD_DrawLine(160-(p/2), 60+p, 160+(p/2), 60+p , Green);
			LCD_DrawLine(150, 160+(p/4),170, 160+(p/4), 0xA52F);
			Xpos = 80; Ypos = 200;
			sprintf(lcd_buffer, "MERRY CHRISTMAS!!!!!!!" );
			GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, Red, White);
		break;
		case 1:
			LCD_DrawLine(60, 80+(p/2), 260, 80+(p/2) , Red);
			LCD_DrawLine(60, 130+(p/2), 260, 130+(p/2) , Yellow);
			LCD_DrawLine(60, 180+(p/2), 260, 180+(p/2) , Red);
		break;
		case 2:
			LCD_DrawLine(160-(p/2), 60+p, 160+(p/2), 60+p , Green);
			LCD_DrawLine(150, 160+(p/4),170, 160+(p/4), 0xA52F);
			Xpos = 80; Ypos = 200;
			sprintf(lcd_buffer, "MERRY CHRISTMAS!!!!!!!" );
			GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, Red, White);					
		break;
		default:
		break;
		}
	}	
}
