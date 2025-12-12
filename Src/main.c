#include <LPC17xx.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include <stdint.h>


#include "AsciiLib.h"
#include "GLCD.h" 
#include <stdio.h>
#include <stdint.h>


uint16_t Xpos, Ypos;
  int line;
  int dist_cm, angle;
  


#define F_CPU       (SystemCoreClock)
#define F_PCLK      (F_CPU/4)


static char lcd_buffer[256];
#define FONT_W  8
#define FONT_H  16



#include "Fc_config_pines.h"
#include "Fc_config_PWM.h"
#include "Fc_speed_control.h"
#include "Fc_config_TIMER.h"
#include "Fc_config_IRQ.h"
#include "Fc_bluetooth_communication.h"
#include "calib.h"
#include "set_distance.h"
#include "sound.h"
#include "dac_cfg.h"
#include "adc_cfg.h"
#include "eint1_cfg.h"
#include "eint2_cfg.h"
#include "usb_cfg.h"


#define FCPU 25000000

//#define size_of_array 9
#define END1 '\n'
#define END2 '\r'

#define N_OFFSETADC	0
#define GAIN_ADC 2.33f

#define PI 3.1415926535897932384626433832795f
#define K2 8*(PI/180)


#define N_POINTS 20
#define DAC_N_BITS 10
#define DAC_N_LEVELS (1U << DAC_N_BITS)
#define DAC_MID_RANGE (1U << (DAC_N_BITS-1))
#define PI 3.1415926535897932384626433832795f

#define DO	260
#define RE	290
#define MI	330
#define FA	350
#define SOL	390
#define LA	440
#define SI	490
#define notese	24
#define notesr	30
#define notesn	20
#define notesb	12



volatile uint8_t *rx_buffer=NULL;	//	DYNAMIC BUFFER FOR SAVING THE MESSAGE
volatile uint16_t current_size=0;	//	VAR. TO SAVE THE SIZE OF THE DYNAMIC BUFFER
volatile uint16_t pointer_to_data=0;	//	POINTER FOR READING THE RECEIVED MESSAGE
uint8_t message=0; //	FLAG ACTIVE WHEN THE MESSAGE HAS BEEN RECEIVED

volatile uint8_t token=0; // FLAG TO START THE MOVEMENT AFTER PRESSING KEY1
volatile uint8_t tokencalib_1=0; //	FLAG THAT ACTIVATES THE CALIBRATION OF WHEEL 1
volatile uint8_t tokencalib_2=0; //	FLAG THAT ACTIVATES THE CALIBRATION OF WHEEL 2

volatile uint8_t speed;	//	VAR. THAT STORES THE PORCENTUAL SPEED RECEIVED (CHAR->UINT8_T)
volatile uint8_t distance;	// VAR. THAT STORES THE DISTANCE RECEIVED (CHAR->UINT8_T)

volatile float gain1=1;	//	GAIN TO CALIBRATE SPEED OF WHEEL 1
volatile float gain2=1;	//	GAIN TO CALIBRATE SPEED OF WHEEL 2

volatile uint32_t speed1=0;	//	TIME BETWEEN EDGES OF ENCODER OF MOTOR 1 (SPEED WHEEL 1)
volatile uint32_t speed2=0;	//	TIME BETWEEN EDGES OF ENCODER OF MOTOR 2 (SPEED WHEEL 2)
volatile uint32_t CAP1_0=0;	//	VAR. TO STORE THE VALUE OF TC OF LAST CAPTURE (MOTOR 1)
volatile uint32_t CAP2_0=0;	//	VAR. TO STORE THE VALUE OF TC OF LAST CAPTURE (MOTOR 2)

float voltage=0;	//	VOLTAGE OF BATTERY MEASURE FROM ADC

uint8_t end_move=0;	// FLAG TO DETECT THE STATE OF THE ROBOT

uint8_t sel_song=0;	//	VAR. TO SELECT A SONG
uint16_t Espana[notese]={DO*2,DO*2,SOL,SOL,MI*2,MI*2,DO*2,SOL*2,FA*2,MI*2,RE*2,DO*2,DO*2,SI,LA,SOL,FA,FA,FA,FA,FA,FA,FA,FA};	 //ESPAÑA
uint16_t navidad[notesn]={SOL,DO*2,DO*2,SI,DO*2,LA,LA,LA,LA,0,LA,RE*2,RE*2,DO*2,LA,SOL,SOL,SOL,SOL,0};	//	FELIZ NAVIDAD
uint16_t blanca[notesr]={LA,LA,LA,LA,LA+20,LA,SOL+20,LA,LA+20,LA+20,LA+20,LA+20,SI,DO*2,DO*2,DO*2,DO*2,RE,MI,FA,SOL,FA,MI,RE,DO,DO,DO,DO,DO,DO};	//	OH BLANCA NAVIDAD
uint16_t Beep[notesb]={600,500,600,500,600,500,600,500,600,500,600,500}; //Beep while backwards moving is performed
uint16_t Silence=0;	//	Silence
uint8_t index_song=0;	//	POINTER TO CHORD OF THE SONG


uint8_t p;

static uint16_t sample_table[N_POINTS];	//	ARRAY OF VALUES OF A SINE
static int sample_idx;	//	POINTER TO THE ARRAY OF SINES

void tone_init_samples() {
		int i;
		float x;
	
		for(i=0;i<N_POINTS;i++) {
				x=DAC_MID_RANGE+(DAC_MID_RANGE-1)*sinf((2*PI/(N_POINTS))*i);
				sample_table[i]=((uint16_t)x)<<6;
		}
		sample_idx = 0;
}

//__________________________________________EINT1|BUTTON|KEY1______________________________________________
void EINT1_IRQHandler(){
	LPC_SC->EXTINT=0x2;	//	CLEAR FLAG OF IRQ
	Fc_config_IRQ();	//	CONFIGURATION OF THE REST OF IRQ
	
	
		token=1;
	
}


//__________________________________________EINT2|BUTTON|KEY2______________________________________________
void EINT2_IRQHandler(){
	
	LPC_SC->EXTINT=0x4;	//	CLEAR FLAG OF IRQ
	
	sel_song++;	//	SELECT THE NEXT SONG
	
	index_song=0;	// POINT TO THE START OF THE SONG
	sample_idx=0;	//	POINT TO THE FIRST VALUE OF THE SINE
	
	LCD_Clear(Cyan);
	
	if(sel_song>=2 && rx_buffer[pointer_to_data]!='R'){
		
		sel_song=0;	//	CIRCULAR SELECTION OF SONG
	
	}
		
	
}

//__________________________________________IRQ______________________________________________
void TIMER0_IRQHandler(){	//	GENERATE THE SOUND SIGNAL WITH DAC
	if(LPC_TIM0->IR&((0x1))){	//	MATCH MR0 -> SHOW DAC VALUE
		LPC_TIM0->IR=(0x1);	//	CLEAR FLAG OF MR0 MATCH
		sound(sample_table[sample_idx]);	//	OUTPUT VALUES OF SINE
		sample_idx=(sample_idx==N_POINTS-1)?0:sample_idx+1;	//	CIRCULAR ARRAY -> CONTINUOUS SINE
		
		if(rx_buffer[pointer_to_data]!='R' && end_move!=0){
			switch(sel_song){
				case 0:
					LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*navidad[index_song]))-1);
					break;
				case 1:
					LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Espana[index_song]))-1);
					break;
				case 2:
					LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*blanca[index_song]))-1);
					break;
				default:
					LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Silence))-1);

				break;
				

		}
	}
		
		if(end_move==0){
					
			LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Silence))-1);
		}
			
	
	
		
		if(rx_buffer[pointer_to_data]=='R'){
					
			LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Beep[index_song]))-1);
		}
		
	}
	
	if(LPC_TIM0->IR&((0x1<<2))){		//Interrupt MR2 change the letter
		LPC_TIM0->IR=(0x1<<2);		//Clear flag of MR2 interrupt
		
		
		if(rx_buffer[pointer_to_data]!='R' && end_move!=0){
		switch(sel_song){
			case 0:
				LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*navidad[index_song]))-1);		//Start with DO	20 samples
				index_song=(index_song>=(notesn-1))?0:index_song+1;
			break;
			
			case 1:			
				LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*Espana[index_song]))-1);		//Start with DO	20 samples
				index_song=(index_song>=(notese-1))?0:index_song+1;
			break;
			
			case 2:
				LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*blanca[index_song]))-1);		//Start with DO	20 samples
				index_song=(index_song>=(notesr-1))?0:index_song+1;
			break;
			
			default:
				LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Silence))-1);

			break;
			
			}
		
		}
		
		if(end_move==0){
					
			LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Silence))-1);
		}
		
		
		if(rx_buffer[pointer_to_data]=='R'){
					
			LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*Beep[index_song]))-1);		//Start with DO	20 samples
			index_song=(index_song>=(notesb-1))?0:index_song+1;
			
		}
	}
}	
	
void TIMER1_IRQHandler(){	//Motor (1) Fastest, right side if you see the front of the car, PWM1.2 P1.20 / CAP1.0 P1.18 / Every 2 edges to calibrate

	if(LPC_TIM1->IR&((0x1<<4))){		//Interrupt CAP0 (Calib speed)
		LPC_TIM1->IR=(0x1<<4);		//Clear flag of CAP0 interrupt
		
		speed1=LPC_TIM1->CR0-CAP1_0;
		CAP1_0=LPC_TIM1->CR0;
		
		tokencalib_1=1;
		/*
		if((speed1)<(speed2)){
			gain1=gain1*0.9999;
			gain2=gain2*1.0001;
			calib(gain1,gain2,speed);
		}
		if((speed1)>(speed2)){
			gain1=gain1*1.0001;
			gain2=gain2*0.9999;
			calib(gain1,gain2,speed);
		}
		
		
  line = 0;
  Xpos = 0; Ypos = line*FONT_H;
  sprintf(lcd_buffer, "Velocity MOTOR 1: %4d", speed1);
  GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);

  line = 1;
  Xpos = 0; Ypos = line*FONT_H;
  sprintf(lcd_buffer, "Velocity MOTOR 2: %4d", speed2);
  GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);	
	*/	
	
	}
}
void TIMER2_IRQHandler(){	//Motor (2) Slowest, left side if you see the front of the car, PWM1.4 P1.23 / CAP2.0 P0.4 / When distance is completed
	
	if(LPC_TIM2->IR&((0x1<<4))){		//Interrupt CAP0	(Calib speed)
		LPC_TIM2->IR=(0x1<<4);		//Clear flag of CAP0 interrupt
		
		speed2=LPC_TIM2->CR0-CAP2_0;
		CAP2_0=LPC_TIM2->CR0;
		
		
		tokencalib_2=1;
		
		/*
		if((speed1)<(speed2)){
			gain1=gain1*0.9999;
			gain2=gain2*1.0001;
			calib(gain1,gain2,speed);
		}
		if((speed1)>(speed2)){
			gain1=gain1*1.0001;
			gain2=gain2*0.9999;
			calib(gain1,gain2,speed);
		}
		
		*/
	}
	
}

void TIMER3_IRQHandler(){	//Reached the value of distance
	if(LPC_TIM3->IR&((0x1))){		//Interrupt MR0 (The distance it was reached)
		LPC_TIM3->IR=(0x1);		//Clear flag of MR0 interrupt
		
		LPC_TIM1->TC=0;
		LPC_TIM2->TC=0;
		LPC_TIM3->TC=0;
		
		LPC_TIM1->PC=0;
		LPC_TIM2->PC=0;
		
		CAP1_0=0;		
		CAP2_0=0;		
		
		
		LPC_TIM1->TCR=(0x1<<1);		//Reset Timer1 counter and prescaler and disable
		LPC_TIM2->TCR=(0x1<<1);		//Reset Timer2 counter and prescaler and disable
		LPC_TIM3->TCR=(0x1<<1);		//Reset Timer3 counter and prescaler and disable
		
		NVIC_DisableIRQ(TIMER1_IRQn);
		NVIC_DisableIRQ(TIMER2_IRQn);
		
		LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16))); // Security Stop
		
		
		
		pointer_to_data+=3;
		token=1;
	}
}

void UART0_IRQHandler(){
	//uint8_t data_index;
	if((LPC_UART0->IIR&0xE)==(0x04)){		//At least 1 interruption is pending and recive data available RDA
		
		uint8_t received_char0 = LPC_UART0->RBR;
		uint8_t *temp_pointer0;

		if((received_char0 != END1)&(received_char0 != END2)){
			
			temp_pointer0 = (uint8_t *)realloc((void*) rx_buffer, current_size + 1);

			if(temp_pointer0 != NULL){
				
				rx_buffer = temp_pointer0;
				
				rx_buffer[current_size] = received_char0;
				current_size++;
				
				
			}
			
		}
			
		if( ((received_char0 == END1)||(received_char0 == END2))){
			
					message=1;
		
		}
		
		
		
		
		/*
		rx_buffer[rx_index++]=LPC_UART0->RBR;		//Save the charapter on rx_buffer
		
		if(rx_index>=size_of_array){
			
			rx_index=0;
			for(data_index=0;data_index<size_of_array;data_index++){
				
				data[data_index]=rx_buffer[data_index];		// Save in array of data
				
			}
		}
		
		*/
	}
	
}

void UART3_IRQHandler(){
//	uint8_t data_index;
	if((LPC_UART3->IIR&0xE)==(0x04)){		//At least 1 interruption is pending and recive data available RDA
		
		uint8_t received_char3 = LPC_UART3->RBR;
		uint8_t *temp_pointer3;

		if((received_char3 != END1)&(received_char3 != END2)){
			
			temp_pointer3 = (uint8_t *)realloc((void*) rx_buffer, current_size + 1);

			if(temp_pointer3 != NULL){
				
				rx_buffer = temp_pointer3;
				
				rx_buffer[current_size] = received_char3;
				current_size++;
				
				
			}
			
		}
			
		if( ((received_char3 == END1)||(received_char3 == END2))){
			
					message=1;
		
		}
		
		
		
		
		
		
		
		/*rx_buffer[rx_index++]=LPC_UART3->RBR;		//Save the charapter on rx_buffer
		if(rx_index>=size_of_array){
			rx_index=0;
			for(data_index=0;data_index<size_of_array;data_index++){
				data[data_index]=rx_buffer[data_index];		// Save in array of data
			}
		}
		*/
		
		
	}
}

void ADC_IRQHandler(){
int8_t i=2;
uint16_t uint16voltage;
	
	voltage=(float)(GAIN_ADC*(N_OFFSETADC+(float)3.3*(((float)((LPC_ADC->ADDR1>>4)&0xFFF))/(float)0xFFF)));		//Obtain value of voltage
	uint16voltage=(uint16_t)(voltage*100);		//Convert in to decimal
	
	if(end_move==0){
		while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
		LPC_UART3->THR='A'; //Indicate Labview the end of the array to send other character**********************************************
	}
	else{
		while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
		LPC_UART3->THR='O';
	}
	
	
	
	if(rx_buffer[pointer_to_data]=='R')	{
		
		while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
		LPC_UART3->THR='F';
		
	}
	
	else{
		
		if(end_move!=0){
			
			switch(sel_song){
			case 0:
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
				LPC_UART3->THR='C';
			break;
			
			case 1:
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
				LPC_UART3->THR='B';

			break;
			
			case 2:
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
				LPC_UART3->THR='D';

			break;
			
			default:
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
				LPC_UART3->THR='H';

			break;
			
			}
		
			
		}
		
		else{
			
			
			while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
			LPC_UART3->THR='A'; 
			
		}
		
	
	
	}
	
		
	
	for(i=2;i>=0;i--){
		while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);		//Waiting to THR empty
		LPC_UART3->THR=(char)((uint16voltage%10)+'0');		//Convert each number in to character (the message will be inverted on lavbiew)
		uint16voltage/=10;
	}
	

}
//___________________________________________________________________________________________
int main(){
	Fc_config_pines();
	Fc_config_TIMER();
	Fc_config_PWM();
  Fc_bluetooth_communication();
	dac_cfg();
	adc_cfg();
	eint1_cfg();
	eint2_cfg();
	usb_cfg();
	
	
	
	
	LCD_Initialization();
	LCD_Clear(Cyan);
	
	
	
	
	dist_cm = 1234;
  angle = 180;

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
	
	
	
	
	NVIC_EnableIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART3_IRQn);
	
	tone_init_samples();
	
	while(1){
		
		
		
		if(tokencalib_1==1){
			
			
		
			if((speed1)<(speed2)){
				gain1=gain1*0.9999;
				gain2=gain2*1.0001;
				calib(gain1,gain2,speed);
			}
			if((speed1)>(speed2)){
				gain1=gain1*1.0001;
				gain2=gain2*0.9999;
				calib(gain1,gain2,speed);
			}
		
				
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
			
			tokencalib_1=0;
				
	}	
		
	if (tokencalib_2==1){
		
			if((speed1)<(speed2)){
				gain1=gain1*0.9999;
				gain2=gain2*1.0001;
				calib(gain1,gain2,speed);
			}
			if((speed1)>(speed2)){
				gain1=gain1*1.0001;
				gain2=gain2*0.9999;
				calib(gain1,gain2,speed);
			}
			
		
		tokencalib_2=0;
		
		
	}
		
		
		
		
		
		
		if(token && message){ //KEY1 has been pressed and a message has reached 
			
			if(pointer_to_data>=current_size){
				end_move=0;
				token=0;
				message=0;
				
				free((void*)rx_buffer);
				rx_buffer = NULL;
				
				current_size = 0;
				pointer_to_data = 0;
				
				
			}
			
			
			
			else if(pointer_to_data + 2 < current_size){
			
				
				switch(rx_buffer[pointer_to_data]){

				case 'V':		//Define speed
					speed=(((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0'));
					pointer_to_data=pointer_to_data+Fc_speed_control(speed);
					end_move=1;

				break;
				
				case 'D':		//Define right movement
					angle=(uint16_t)((((uint16_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint16_t)(rx_buffer[pointer_to_data+2]-'0')));
					set_distance(K2*angle);
					LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
					LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
					NVIC_EnableIRQ(TIMER1_IRQn);
					NVIC_EnableIRQ(TIMER2_IRQn);
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x1<<16);
					token=0;

				break;
				
				case 'I':		//Define left movement
					angle=(uint16_t)((((uint16_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint16_t)(rx_buffer[pointer_to_data+2]-'0')));
					set_distance(K2*angle);
					LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
					LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
					NVIC_EnableIRQ(TIMER1_IRQn);
					NVIC_EnableIRQ(TIMER2_IRQn);
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x2<<16);
					token=0;

				break;
				
				case 'A':		//Define forward movement
					distance=(uint8_t)((((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0')));
					set_distance(distance);
					LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
					LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
					NVIC_EnableIRQ(TIMER1_IRQn);
					NVIC_EnableIRQ(TIMER2_IRQn);
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);
					token=0;

				break;
				
				case 'R':		//Define backwards movement
					distance=(uint8_t)((((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0')));
					set_distance(distance);
					LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
					LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
					NVIC_EnableIRQ(TIMER1_IRQn);
					NVIC_EnableIRQ(TIMER2_IRQn);
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x2<<16);
					token=0;
					

				break;
				
				default:
						LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16))); // Stop
				break;
			
				
				
			}
				
			
				
			}
		}
	}
}

