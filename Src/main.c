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

#define DO	261
#define RE	293
#define MI	329
#define FA	349
#define SOL	392
#define LA	440
#define SI	493
#define notese	24
#define notesr	53


//SE DEBE MEJORAR LA MR0 DE AMBOS TEMPORIZADORES LA FUNCION CALIB Y LA FUNCION SPEED SE AÑADIO UNA GANANCIA PARA CONTRARRESTAR LAS GRANDES DIFERENCIAS
//ENTRE AMBOS MOTORES ACTUALES

//volatile char rx_buffer[size_of_array]={0};		// Reception Buffer 
//volatile char data[]={0};		// Real comunication
//volatile uint8_t rx_index=0;
//volatile uint8_t pointer_to_data=0; // Pointer to the array of data
volatile uint8_t *rx_buffer=NULL;
volatile uint16_t current_size=0;
volatile uint16_t pointer_to_data=0;
uint8_t message=0; //message reached

volatile uint8_t token=0; // Start the movement after pressing the KEY1

volatile uint8_t speed;
volatile uint16_t distance;

volatile float gain1=1;
volatile float gain2=1;

volatile uint32_t speed1=0;		//speed motor1
volatile uint32_t speed2=0;		//speed motor2
volatile uint32_t CAP1_0=0;		//Value of last cap
volatile uint32_t CAP2_0=0;		//Value of last cap

float voltage=0;

uint32_t contador=0;
uint8_t end_move=0;

uint8_t sel_song=0;
uint16_t Espana[notese]={DO*2,DO*2,SOL,SOL,MI*2,MI*2,DO*2,SOL*2,FA*2,MI*2,RE*2,DO*2,DO*2,SI,LA,SOL,FA,FA,FA,FA,FA,FA,FA,FA};	 //ESPAÑA
uint16_t Cucaracha[notese]={DO,FA,DO,FA,FA,FA,LA,LA,DO,FA,DO,FA,FA,FA,LA,LA,FA,SOL,MI,FA,RE,MI,DO,DO}; //CUCARACHA
uint16_t Rocky[notesr]={MI,SOL,SOL,LA,LA,LA,LA,LA,LA*2,SI*2,SI*2,MI,MI,MI,MI,MI,MI,SOL,SOL,LA,LA,LA,LA,LA,LA*2,SI*2,SI*2,MI,MI,MI,MI,MI,RE,DO,RE,RE,DO,RE,MI,MI,DO,DO*2,SI,SI*2,LA,SI,SOL,SOL,SOL,DO,SI,SI,SI}; //ROCKY
uint16_t Beep=DO*2; //Beep while backwards moving is performed
uint8_t index_song=0;

static uint16_t sample_table[N_POINTS];		//array of values of a sine signal
static int sample_idx;										//index pointing to the last output through DAC

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
	LPC_SC->EXTINT=0x2; // Clear flag of IRQ
	Fc_config_IRQ();
	
	//if(rx_buffer != NULL && current_size > 0){
		token=1;
		end_move=1;
	//}
}


//__________________________________________EINT2|BUTTON|KEY2______________________________________________
void EINT2_IRQHandler(){
	
	LPC_SC->EXTINT=0x3; //Clear flag of IRQ
	
	sel_song++;
	
	
	if(sel_song>=3 && rx_buffer[pointer_to_data]!='R'){
		
		sel_song=0;
	
	}
		
	
}

//__________________________________________IRQ______________________________________________
void TIMER0_IRQHandler(){	//Generate the sound signal with DAC
	if(LPC_TIM0->IR&((0x1))){		//Interrupt MR0 Show the DAC value
		LPC_TIM0->IR=(0x1);		//Clear flag of MR0 interrupt
		sound(sample_table[sample_idx]);
		sample_idx=(sample_idx==N_POINTS-1)?0:sample_idx+1;
		
		
		switch(sel_song){
			case 0:
				LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Espana[index_song]))-1);
				break;
			case 1:
				LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Cucaracha[index_song]))-1);
				break;
			case 2:
				LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Rocky[index_song]))-1);
				break;
			case 3:
				LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Beep))-1);
				break;
			

		}
		
	}
	
	if(LPC_TIM0->IR&((0x1<<2))){		//Interrupt MR2 change the letter
		LPC_TIM0->IR=(0x1<<2);		//Clear flag of MR2 interrupt
		
		switch(sel_song){
			case 0:
				LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*Espana[index_song]))-1);		//Start with DO	20 samples
				index_song=(index_song==(notese-1))?0:index_song+1;
			break;
			
			case 1:
				LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*Cucaracha[index_song]))-1);		//Start with DO	20 samples
				index_song=(index_song==(notese-1))?0:index_song+1;
			break;
			
			case 2:
				LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*Rocky[index_song]))-1);		//Start with DO	20 samples
				index_song=(index_song==(notesr-1))?0:index_song+1;
			break;
			
			case 3:
				LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*Beep))-1);		//Start with DO	20 samples
			break;
		}
	
	}
}	
	
void TIMER1_IRQHandler(){	//Motor (1) Fastest, right side if you see the front of the car, PWM1.2 P1.20 / CAP1.0 P1.18 / Every 2 edges to calibrate

	if(LPC_TIM1->IR&((0x1<<4))){		//Interrupt CAP0 (Calib speed)
		LPC_TIM1->IR=(0x1<<4);		//Clear flag of CAP0 interrupt
		
		speed1=LPC_TIM1->CR0-CAP1_0;
		CAP1_0=LPC_TIM1->CR0;
		
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
		
	}
}
void TIMER2_IRQHandler(){	//Motor (2) Slowest, left side if you see the front of the car, PWM1.4 P1.23 / CAP2.0 P0.4 / When distance is completed
	
	if(LPC_TIM2->IR&((0x1<<4))){		//Interrupt CAP0	(Calib speed)
		LPC_TIM2->IR=(0x1<<4);		//Clear flag of CAP0 interrupt
		
		speed2=LPC_TIM2->CR0-CAP2_0;
		CAP2_0=LPC_TIM2->CR0;
		
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
	
	for(i=2;i>=0;i--){
		while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);		//Waiting to THR empty
		LPC_UART3->THR=(char)((uint16voltage%10)+'0');		//Convert each number in to character (the message will be inverted on lavbiew)
		uint16voltage/=10;
	}
	while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
	LPC_UART3->THR='A';		//Indicate lavbiew is the end of the number we could used other caracter


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
  Xpos = 0; Ypos = line*FONT_H;
  sprintf(lcd_buffer, "Velocity MOTOR 1: %4d", speed1);
  GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);

  line = 1;
  Xpos = 0; Ypos = line*FONT_H;
  sprintf(lcd_buffer, "Velocity MOTOR 2: %4d", speed2);
  GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);	
	
  line = 2;
  Xpos = 0; Ypos = line*FONT_H;
  sprintf(lcd_buffer, "VOLTAGE: %f", voltage);
  GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);
	
	
	
	
	NVIC_EnableIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART3_IRQn);
	
	tone_init_samples();
	
	while(1){
		
		
		
		if(token && message){ //KEY1 has been pressed and a message has reached 
			
			if(pointer_to_data>=current_size){
				end_move=0;
				token=0;
				
				free((void*)rx_buffer);
				rx_buffer = NULL;
				
				current_size = 0;
				pointer_to_data = 0;
				
				
				
				line = 2;
				Xpos = 0; Ypos = line*FONT_H;
				sprintf(lcd_buffer, "VOLTAGE: %f", voltage);
				GUI_Text(Xpos, Ypos, (uint8_t *)lcd_buffer, White, Blue);

			}
			
			
			
			else if(pointer_to_data + 2 < current_size){
			
				
				switch(rx_buffer[pointer_to_data]){

				case 'V':		//Define speed
					speed=(((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0'));
					pointer_to_data=pointer_to_data+Fc_speed_control(speed);

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
					distance=(uint16_t)((((uint16_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint16_t)(rx_buffer[pointer_to_data+2]-'0')));
					set_distance(distance);
					LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
					LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
					NVIC_EnableIRQ(TIMER1_IRQn);
					NVIC_EnableIRQ(TIMER2_IRQn);
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);
					token=0;

				break;
				
				case 'R':		//Define backwards movement
					distance=(uint16_t)((((uint16_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint16_t)(rx_buffer[pointer_to_data+2]-'0')));
					set_distance(distance);
					LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
					LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
					NVIC_EnableIRQ(TIMER1_IRQn);
					NVIC_EnableIRQ(TIMER2_IRQn);
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x2<<16);
					token=0;
					sel_song=3; //Beep sound

				break;
				
				default:
						LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16))); // Stop
				break;
			
				
				
			}
				
			
				
			}
		}
	}
}

