//	----------------------------------------------------------------------------------------------------
//	1. SYSTEM AND STANDARD LIBRARIES
//	----------------------------------------------------------------------------------------------------

#include <LPC17xx.h>	//	DEVICE LIBRARY
#include <stdlib.h>	//	LIBRARY FOR DYNAMIC MEMORY (MALLOC, FREE, NULL)

//	----------------------------------------------------------------------------------------------------
//	2. LCD AND GRAPHICS LIBRARIES
//	----------------------------------------------------------------------------------------------------

#include "AsciiLib.h"
#include "GLCD.h" 

//	----------------------------------------------------------------------------------------------------
//	3. FUNCTIONS LIBRARIES
//	----------------------------------------------------------------------------------------------------

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
#include "Fc_display_variable_value.h"
#include "Fc_display_draw.h"
#include "Fc_reset_Tc.h"
#include "tone_samples.h"

//	----------------------------------------------------------------------------------------------------
//	4. SYSTEM DEFINITIONS
//	----------------------------------------------------------------------------------------------------

#define F_CPU       (SystemCoreClock)
#define F_PCLK      (F_CPU/4)
#define FCPU 25000000

//	----------------------------------------------------------------------------------------------------
//	5. MATH AND CONVERSION CONSTANTS
//	----------------------------------------------------------------------------------------------------

#define PI 3.1415926535897932384626433832795f
#define K2 (8.0f * (PI/180.f))	//	CONVERSION FROM ANGLE TO DISTANCE OF WHEELS

//	----------------------------------------------------------------------------------------------------
//	6. UART COMMUNICATION CONSTANTS
//	----------------------------------------------------------------------------------------------------

#define END1 '\n'
#define END2 '\r'

//	----------------------------------------------------------------------------------------------------
//	7. ADC AND WHEEL CALIBRATION CONSTANTS
//	----------------------------------------------------------------------------------------------------

#define N_OFFSETADC	0
#define GAIN_ADC 2.8f //2.33

//	----------------------------------------------------------------------------------------------------
//	8. LCD DISPLAY CONSTANTS
//	----------------------------------------------------------------------------------------------------

#define FONT_W  8
#define FONT_H  16
static char lcd_buffer[256];	//	BUFFER FOR LCD TEXT

//	----------------------------------------------------------------------------------------------------
//	9. AUDIO(DAC) CONSTANTS
//	----------------------------------------------------------------------------------------------------

#define N_POINTS 20
#define DAC_N_LEVELS (1U << DAC_N_BITS)


//	NOTE FREQUENCIES AND ARRAY SIZE OF SONGS

#define DO	260
#define RE	290
#define MI	330
#define FA	350
#define SOL	390
#define LA	440
#define SI	490

#define notesnav	20
#define notesesp	24
#define notesblan	30
#define notesb	12

//	----------------------------------------------------------------------------------------------------
//	10. COMMUNICATION VARIABLES
//	----------------------------------------------------------------------------------------------------

volatile uint8_t *rx_buffer=NULL;	//	DYNAMIC BUFFER FOR SAVING THE MESSAGE
volatile uint16_t current_size=0;	//	VAR. TO SAVE THE SIZE OF THE DYNAMIC BUFFER
volatile uint16_t pointer_to_data=0;	//	POINTER FOR READING THE RECEIVED MESSAGE
volatile uint8_t message=0; //	FLAG ACTIVE WHEN THE MESSAGE HAS BEEN RECEIVED
int8_t loop=2;	//	COUNTER TO SEND 3 CHARACTERS OF VOLTAGE

//	----------------------------------------------------------------------------------------------------
//	11. CONTROL FLAGS
//	----------------------------------------------------------------------------------------------------

volatile uint8_t token=0; // FLAG TO START THE MOVEMENT AFTER PRESSING KEY1
volatile uint8_t tokencalib_1=0; //	FLAG THAT ACTIVATES THE CALIBRATION OF WHEEL 1
volatile uint8_t tokencalib_2=0; //	FLAG THAT ACTIVATES THE CALIBRATION OF WHEEL 2
volatile uint8_t tokendisplay=0;	//	COUNTER OF 100 PULSES OF THE ENCODER TO UPDATE VARIABLES ON DISPLAY
volatile uint8_t end_move=0;	// FLAG TO DETECT THE STATE OF THE ROBOT
volatile uint8_t adc_ready=0;	//	FLAG TO DETECT THE READ OF ADC
volatile uint8_t key2=0;	//	FLAG TO DETECT SELECTION OF SONG CHANGED

//	----------------------------------------------------------------------------------------------------
//	12. MOTOR CALIBRATION VARIABLES
//	----------------------------------------------------------------------------------------------------

volatile uint8_t speed;	//	VAR. THAT STORES THE PORCENTUAL SPEED RECEIVED (CHAR->UINT8_T)
volatile uint8_t distance;	// VAR. THAT STORES THE DISTANCE RECEIVED (CHAR->UINT8_T)
volatile float gain1=1.0f;	//	GAIN TO CALIBRATE SPEED OF WHEEL 1
volatile float gain2=1.0f;	//	GAIN TO CALIBRATE SPEED OF WHEEL 2
volatile uint32_t speed1=0;	//	TIME BETWEEN EDGES OF ENCODER OF MOTOR 1 (SPEED WHEEL 1)
volatile uint32_t speed2=0;	//	TIME BETWEEN EDGES OF ENCODER OF MOTOR 2 (SPEED WHEEL 2)
volatile uint32_t CAP1_0=0;	//	VAR. TO STORE THE VALUE OF TC OF LAST CAPTURE (MOTOR 1)
volatile uint32_t CAP2_0=0;	//	VAR. TO STORE THE VALUE OF TC OF LAST CAPTURE (MOTOR 2)

//	----------------------------------------------------------------------------------------------------
//	13. DISPLAY VARIABLES
//	----------------------------------------------------------------------------------------------------

volatile float voltage=0;	//	VOLTAGE OF BATTERY MEASURE FROM ADC
volatile uint16_t uint16voltage;	//	TEMPORARY VARIABLE TO SEND 3 DIGITS OF VOLTAGE THROUGH UART
uint16_t Xpos, Ypos;	//	LCD (X,Y) COORDINATES
int line;
int dist_cm, angle;	//	MOVEMENT PARAMETERS

//	----------------------------------------------------------------------------------------------------
//	14. SONG VARIABLES
//	----------------------------------------------------------------------------------------------------

static const uint16_t navidad[notesnav]={SOL,DO*2,DO*2,SI,DO*2,LA,LA,LA,LA,0,LA,RE*2,RE*2,DO*2,LA,SOL,SOL,SOL,SOL,0};	//	FELIZ NAVIDAD
static const uint16_t Espana[notesesp]={DO*2,DO*2,SOL,SOL,MI*2,MI*2,DO*2,SOL*2,FA*2,MI*2,RE*2,DO*2,DO*2,SI,LA,SOL,FA,FA,FA,FA,FA,FA,FA,FA};	 //ESPAÑA
static const uint16_t blanca[notesblan]={LA,LA,LA,LA,LA+20,LA,SOL+20,LA,LA+20,LA+20,LA+20,LA+20,SI,DO*2,DO*2,DO*2,DO*2,RE,MI,FA,SOL,FA,MI,RE,DO,DO,DO,DO,DO,DO};	//	OH BLANCA NAVIDAD
static const uint16_t Beep[notesb]={600,500,600,500,600,500,600,500,600,500,600,500}; //Beep while backwards moving is performed
static const uint16_t Silence=1;	//	SILENCE
uint8_t index_song=0;	//	POINTER TO CHORD OF THE SONG
static uint16_t sample_table[N_POINTS];	//	ARRAY OF VALUES OF A SINE
static int sample_idx;	//	POINTER TO THE ARRAY OF SINES
volatile uint8_t sel_song=0; // VAR. TO SELECT A SONG
volatile uint8_t i=0;	//	COUNTER TO PRODUCE ARRAY OF SINE VALUES

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
	key2=1;	
	if(sel_song>=2&&rx_buffer[pointer_to_data]!='R'){		
		sel_song=0;	//	CIRCULAR SELECTION OF SONG	
	}
		
	
}

//__________________________________________IRQ______________________________________________
void TIMER0_IRQHandler(){	//	GENERATE THE SOUND SIGNAL WITH DAC
	if(LPC_TIM0->IR&((0x1))){	//	MATCH MR0 -> SHOW DAC VALUE
		LPC_TIM0->IR=(0x1);	//	CLEAR FLAG OF MR0 MATCH
		sound(sample_table[sample_idx]);	//	OUTPUT VALUES OF SINE
		sample_idx=(sample_idx==N_POINTS-1)?0:sample_idx+1;	//	CIRCULAR ARRAY -> CONTINUOUS SINE
		
		if(rx_buffer[pointer_to_data]!='R'&&end_move!=0){
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
		
		if(rx_buffer[pointer_to_data]!='R'&&end_move!=0){
		switch(sel_song){
		case 0:
			LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*navidad[index_song]))-1);		//Start with DO	20 samples
			index_song=(index_song>=(notesnav-1))?0:index_song+1;
		break;
		case 1:			
			LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*Espana[index_song]))-1);		//Start with DO	20 samples
			index_song=(index_song>=(notesesp-1))?0:index_song+1;
		break;			
		case 2:
			LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*blanca[index_song]))-1);		//Start with DO	20 samples
			index_song=(index_song>=(notesblan-1))?0:index_song+1;
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
	}
}
void TIMER2_IRQHandler(){	//Motor (2) Slowest, left side if you see the front of the car, PWM1.4 P1.23 / CAP2.0 P0.4 / When distance is completed	
	if(LPC_TIM2->IR&((0x1<<4))){		//Interrupt CAP0	(Calib speed)
		LPC_TIM2->IR=(0x1<<4);		//Clear flag of CAP0 interrupt		
		speed2=LPC_TIM2->CR0-CAP2_0;
		CAP2_0=LPC_TIM2->CR0;				
		tokencalib_2=1;
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
	if((LPC_UART0->IIR&0xE)==(0x04)){		//At least 1 interruption is pending and recive data available RDA	
		uint8_t received_char0 = LPC_UART0->RBR;
		uint8_t *temp_pointer0;

		if((received_char0!=END1)&(received_char0!=END2)){		
			temp_pointer0=(uint8_t *)realloc((void*)rx_buffer,current_size+1);
			if(temp_pointer0!=NULL){	
				rx_buffer=temp_pointer0;			
				rx_buffer[current_size] = received_char0;
				current_size++;							
			}		
		}			
		if( ((received_char0==END1)||(received_char0==END2))){			
					message=1;		
		}
	}
	
}

void UART3_IRQHandler(){
	if((LPC_UART3->IIR&0xE)==(0x04)){		//At least 1 interruption is pending and recive data available RDA	
		uint8_t received_char3 = LPC_UART3->RBR;
		uint8_t *temp_pointer3;

		if((received_char3!=END1)&(received_char3!=END2)){
			temp_pointer3=(uint8_t *)realloc((void*)rx_buffer,current_size+1);
			if(temp_pointer3!= NULL){			
				rx_buffer=temp_pointer3;			
				rx_buffer[current_size]=received_char3;
				current_size++;				
			}			
		}			
		if(((received_char3==END1)||(received_char3==END2))){			
					message=1;		
		}
	}
}

void ADC_IRQHandler(){
	
	voltage=(float)(GAIN_ADC*(N_OFFSETADC+(float)3.3*(((float)((LPC_ADC->ADDR1>>4)&0xFFF))/(float)0xFFF)));	//	OBTAINE VALUE OF VOLTAGE
	uint16voltage=(uint16_t)(voltage*100);	//	CONVERSION TO DECIMAL
	adc_ready=1;
	
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
//------------------------------------------------DISPLAY------------------------------------------------		
	LCD_Initialization();
	LCD_Clear(Cyan);
	Fc_display_variable_value(speed1,speed2,voltage,Xpos,Ypos,line,dist_cm,angle,lcd_buffer);
//-------------------------------------------------------------------------------------------------------			
	NVIC_EnableIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART3_IRQn);
	
	for(i=0;i<N_POINTS;i++){
	sample_table[i]=tone_init_samples(i);
	}
	sample_idx = 0;
	
	while(1){
		
		if(adc_ready==1){
			
			if(end_move==0){
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
				LPC_UART3->THR='A'; //INDICATE LABVIEW MODE WAITING AND SILENCE
			}
			else{
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
				LPC_UART3->THR='O';	//INDICATE LABVIEW MODE WORKING
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
			for(loop=2;loop>=0;loop--){
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);		//Waiting to THR empty
				LPC_UART3->THR=(char)((uint16voltage%10)+'0');		//Convert each number in to character (the message will be inverted on lavbiew)
				uint16voltage/=10;
			}
			
			
			adc_ready=0;
			
		}
		
		if(key2==1){
			
			LCD_Clear(Cyan);
			Fc_display_draw(sel_song,Xpos,Ypos,line,dist_cm,angle,lcd_buffer);
			key2=0;
			
		}
		
		
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
			
//------------------------------------------------DISPLAY------------------------------------------------		
			if(tokendisplay>=99){	//	UPDATE VARIABLES SHOWN ON THE SCREEN EACH 100 PULSES OF THE ENCODER
				Fc_display_variable_value(speed1,speed2,voltage,Xpos,Ypos,line,dist_cm,angle,lcd_buffer);
				tokendisplay=0;
			}
//-------------------------------------------------------------------------------------------------------
			
			tokendisplay++;
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
		if(token&&message){ //KEY1 has been pressed and a message has reached 		
			if(pointer_to_data>=current_size){
				end_move=0;
				token=0;
				message=0;			
				free((void*)rx_buffer);
				rx_buffer = NULL;			
				current_size=0;
				pointer_to_data=0;			
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
					Fc_reset_Tc();
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x1<<16);
					token=0;
				break;				
				case 'I':		//Define left movement
					angle=(uint16_t)((((uint16_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint16_t)(rx_buffer[pointer_to_data+2]-'0')));
					set_distance(K2*angle);
					Fc_reset_Tc();
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x2<<16);
					token=0;
				break;				
				case 'A':		//Define forward movement
					distance=(uint8_t)((((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0')));
					set_distance(distance);
					Fc_reset_Tc();
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);
					token=0;
				break;				
				case 'R':		//Define backwards movement
					distance=(uint8_t)((((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0')));
					set_distance(distance);
					Fc_reset_Tc();
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

