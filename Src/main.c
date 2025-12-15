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
static const uint16_t Espana[notesesp]={DO*2,DO*2,SOL,SOL,MI*2,MI*2,DO*2,SOL*2,FA*2,MI*2,RE*2,DO*2,DO*2,SI,LA,SOL,FA,FA,FA,FA,FA,FA,FA,FA};	//	ESPAÑA
static const uint16_t blanca[notesblan]={LA,LA,LA,LA,LA+20,LA,SOL+20,LA,LA+20,LA+20,LA+20,LA+20,SI,DO*2,DO*2,DO*2,DO*2,RE,MI,FA,SOL,FA,MI,RE,DO,DO,DO,DO,DO,DO};	//	OH BLANCA NAVIDAD
static const uint16_t Beep[notesb]={600,500,600,500,600,500,600,500,600,500,600,500}; //	BEEP WHILE BACKWARDS MOVING IS PERFORMED
static const uint16_t Silence=1;	//	SILENCE
uint8_t index_song=0;	//	POINTER TO CHORD OF THE SONG
static uint16_t sample_table[N_POINTS];	//	ARRAY OF VALUES OF A SINE
static int sample_idx;	//	POINTER TO THE ARRAY OF SINES
volatile uint8_t sel_song=0; // VAR. TO SELECT A SONG
volatile uint8_t i=0;	//	COUNTER TO PRODUCE ARRAY OF SINE VALUES

//__________________________________________EINT1|BUTTON|KEY1______________________________________________
void EINT1_IRQHandler(){	//	START MOVEMENTS
	LPC_SC->EXTINT=0x2;	//	CLEAR FLAG OF IRQ
	Fc_config_IRQ();	//	CONFIGURATION OF THE REST OF IRQ	
	token=1;	
}

//__________________________________________EINT2|BUTTON|KEY2______________________________________________
void EINT2_IRQHandler(){	//CHANGE THE SOUND	
	LPC_SC->EXTINT=0x4;	//	CLEAR FLAG OF IRQ	
	index_song=0;	// POINT TO THE START OF THE SONG
	sample_idx=0;	//	POINT TO THE FIRST VALUE OF THE SINE
	key2=1;
	if(sel_song>=2&&rx_buffer[pointer_to_data]!='R'){		
		sel_song=0;	//	CIRCULAR SELECTION OF SONG	
	}else if(sel_song!=2){
		sel_song++;	//	SELECT THE NEXT SONG
	}
}

//______________________________________________IRQ______________________________________________
void TIMER0_IRQHandler(){	//	GENERATE THE SOUND SIGNAL WITH DAC
	if(LPC_TIM0->IR&((0x1))){	//	MATCH MR0 -> SHOW DAC VALUE
		LPC_TIM0->IR=(0x1);	//	CLEAR FLAG OF MR0 MATCH
		sound(sample_table[sample_idx]);	//	OUTPUT VALUES OF SINE
		sample_idx=(sample_idx==N_POINTS-1)?0:sample_idx+1;	//	CIRCULAR ARRAY -> CONTINUOUS SINE
		
		if(rx_buffer[pointer_to_data]!='R'&&end_move!=0){	//	CHANGE THE FRECUENCY OF THE SINE DAC WITH SEL_SONG 
			switch(sel_song){
				case 0:
					LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((F_PCLK)/(N_POINTS*navidad[index_song]))-1);	//	MERRY CHRISTMAS, EACH POINT OF SINE, WORKING MODE
					break;
				case 1:
					LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((F_PCLK)/(N_POINTS*Espana[index_song]))-1);	//	ESPAÑA, EACH POINT OF SINE, WORKING MODE
					break;
				case 2:
					LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((F_PCLK)/(N_POINTS*blanca[index_song]))-1);	//	WHITE CHRISTMAS, EACH POINT OF SINE, WORKING MODE
					break;
				default:
					LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((F_PCLK)/(N_POINTS*Silence))-1);	//	SILENCE 
					break;
			}
		}		
		if(end_move==0){					
			LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((F_PCLK)/(N_POINTS*Silence))-1);	//	SILENCE, WAITING MODE
		}	
		if(rx_buffer[pointer_to_data]=='R'){					
			LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((F_PCLK)/(N_POINTS*Beep[index_song]))-1);	//	BEEP WHEN BACKWARD MOVEMENT, EACH POINT OF SINE, WORKING MODE
		}
	}	
	
	if(LPC_TIM0->IR&((0x1<<2))){	//	MATCH MR2 CHANGE THE LETTER
		LPC_TIM0->IR=(0x1<<2);	//	CLEAR FLAG OF MR2 INTERRUPT
		
		if(rx_buffer[pointer_to_data]!='R'&&end_move!=0){	//	CHANGE THE FRECUENCY OF THE SINE DAC WITH SEL_SONG 
			switch(sel_song){
				case 0:
					LPC_TIM0->MR0=(uint16_t)(((F_PCLK)/(N_POINTS*navidad[index_song]))-1);	//	MERRY CHRISTMAS, CHANGE THE NOTE OF THE SIGN, WORKING MODE		
					index_song=(index_song>=(notesnav-1))?0:index_song+1;
					break;
				case 1:			
					LPC_TIM0->MR0=(uint16_t)(((F_PCLK)/(N_POINTS*Espana[index_song]))-1);	//	ESPAÑA, CHANGE THE NOTE OF THE SIGN, WORKING MODE
					index_song=(index_song>=(notesesp-1))?0:index_song+1;
					break;			
				case 2:
					LPC_TIM0->MR0=(uint16_t)(((F_PCLK)/(N_POINTS*blanca[index_song]))-1);	//	WHITE CHRISTMAS, CHANGE THE NOTE OF THE SIGN, WORKING MODE
					index_song=(index_song>=(notesblan-1))?0:index_song+1;
					break;			
				default:
					LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((F_PCLK)/(N_POINTS*Silence))-1);	//	SILENCE 
					break;			
			}		
		}		
		if(end_move==0){					
			LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((F_PCLK)/(N_POINTS*Silence))-1);	//	SILENCE, WAITING MODE
		}				
		if(rx_buffer[pointer_to_data]=='R'){				
			LPC_TIM0->MR0=(uint16_t)(((F_PCLK)/(N_POINTS*Beep[index_song]))-1);	//	BEEP WHEN BACKWARD MOVEMENT, CHANGE THE NOTE OF THE SIGN, WORKING MODE
			index_song=(index_song>=(notesb-1))?0:index_song+1;			
		}
	}
}	
	
void TIMER1_IRQHandler(){	//	CONTROL TIME EACH RISING EDGE OF THE ENCODER_1 WITH CAP (CONTROL SPEED1)
	if(LPC_TIM1->IR&((0x1<<4))){	//	INTERRUPT CAP0 (CALIB SPEED)
		LPC_TIM1->IR=(0x1<<4);	//	CLEAR FLAG OF CAP0 INTERRUT	
		speed1=LPC_TIM1->CR0-CAP1_0;	//	TIME MEASURED BETWEEN RISING EDGES
		CAP1_0=LPC_TIM1->CR0;		
		tokencalib_1=1;	//	FLAG TO CHANGE THE SPEED ON MAIN				
	}
}
void TIMER2_IRQHandler(){	//	CONTROL TIME EACH RISING EDGE OF THE ENCODER_2	WITH CAP (CONTROL SPEED2)
	if(LPC_TIM2->IR&((0x1<<4))){	//	INTERRUPT CAP0 (CALIB SPEED)
		LPC_TIM2->IR=(0x1<<4);	//	CLEAR FLAG OF CAP0 INTERRUT	
		speed2=LPC_TIM2->CR0-CAP2_0;	//	TIME MEASURED BETWEEN RISING EDGES
		CAP2_0=LPC_TIM2->CR0;				
		tokencalib_2=1;	//	FLAG TO CHANGE THE SPEED ON MAIN
	}
}

void TIMER3_IRQHandler(){	//	REACH THE DESIRED DISTANCE/ANGLE
	if(LPC_TIM3->IR&((0x1))){	//	MATCH MR0 (THE DISTANCE IT WAS REACHED)
		LPC_TIM3->IR=(0x1);	//	CLEAR FLAG OF MR0 INTERRUPT
		
		LPC_TIM1->TC=0;	//	RESET OF ALL VARIABLES AND INTERRUPTIONS
		LPC_TIM2->TC=0;
		LPC_TIM3->TC=0;
		LPC_TIM1->PC=0;
		LPC_TIM2->PC=0;	
		CAP1_0=0;		
		CAP2_0=0;		
		LPC_TIM1->TCR=(0x1<<1);	//	RESET TIMER1 COUNTER AND PRESCALER AND DISABLE
		LPC_TIM2->TCR=(0x1<<1);	//	RESET TIMER2 COUNTER AND PRESCALER AND DISABLE
		LPC_TIM3->TCR=(0x1<<1);	//	RESET TIMER3 COUNTER AND PRESCALER AND DISABLE
		NVIC_DisableIRQ(TIMER1_IRQn);
		NVIC_DisableIRQ(TIMER2_IRQn);
		
		LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)));	// SECURITY STOP
		pointer_to_data+=3;	//	NEXT COMMAND OF MOVEMMENT
		token=1;	//	ENNABLE FOR A NEW COMMAND MOVEMMENT ON WHILE(1)
	}
}

void UART0_IRQHandler(){	//	COMMUNICATION WITH LAVBIEW WITH USB
	if((LPC_UART0->IIR&0xE)==(0x04)){	//	AT LEAST 1 INTERRUPTION IS PENDING AND RECIVE DATA AVAILABLE RDA	
		uint8_t received_char0=LPC_UART0->RBR;
		uint8_t *temp_pointer0;

		if((received_char0!=END1)&(received_char0!=END2)){	//	IT IS EXECUTED IF THERE IS NO "LINE BREAK" IN THE MESSAGE
			temp_pointer0=(uint8_t *)realloc((void*)rx_buffer,current_size+1);	//	INCREMENT THE DINAMIC MEMMORY WITH FC REALLOC
			if(temp_pointer0!=NULL){	//	SAFE THE CHARACTER ON RX_BUFFER WHEN THERE IS SPACE
				rx_buffer=temp_pointer0;			
				rx_buffer[current_size]=received_char0;
				current_size++;							
			}		
		}			
		if(((received_char0==END1)||(received_char0==END2))){	//	WHEN "LINE BREAKS" COMES THE MESSAGE IS FULL RECIVED		
			message=1;		
		}
	}
}

void UART3_IRQHandler(){	//	COMMUNICATION WITH LAVBIEW WITH BLUETOOTH
	if((LPC_UART3->IIR&0xE)==(0x04)){		//At least 1 interruption is pending and recive data available RDA	
		uint8_t received_char3=LPC_UART3->RBR;
		uint8_t *temp_pointer3;

		if(message==0){
			if((received_char3!=END1)&(received_char3!=END2)){	//	IT IS EXECUTED IF THERE IS NO "LINE BREAK" IN THE MESSAGE
				temp_pointer3=(uint8_t *)realloc((void*)rx_buffer,current_size+1);	//	INCREMENT THE DINAMIC MEMMORY WITH FC REALLOC
				if(temp_pointer3!=NULL){	//	SAFE THE CHARACTER ON RX_BUFFER WHEN THERE IS SPACE			
					rx_buffer=temp_pointer3;			
					rx_buffer[current_size]=received_char3;
					current_size++;				
				}			
			}			
			if(((received_char3==END1)||(received_char3==END2))){	//	WHEN "LINE BREAKS" COMES THE MESSAGE IS FULL RECIVED					
				message=1;		
			}
		}else{
			NVIC_SetPendingIRQ(EINT2_IRQn);	//	CHANGE THE MUSIC FROM LAVBIEW
		}
	}
}

void ADC_IRQHandler(){	//	BATTERY MEASURE
	
	voltage=(float)(GAIN_ADC*(N_OFFSETADC+(float)3.3*(((float)((LPC_ADC->ADDR1>>4)&0xFFF))/(float)0xFFF)));	//	OBTAINE VALUE OF VOLTAGE
	uint16voltage=(uint16_t)(voltage*100);	//	CONVERSION TO DECIMAL
	adc_ready=1;
	
}
//___________________________________________________________________________________________
int main(){
	Fc_config_pines();	// ALL CONFIGURATE FC
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
	Fc_display_variable_value(speed1,speed2,voltage,Xpos,Ypos,line,dist_cm,angle,lcd_buffer);	//	INITIALIZATION VARIABLE VALUES ON DISPLAY
	Fc_display_draw(sel_song,Xpos,Ypos,line,dist_cm,angle,lcd_buffer);	//	INITIALIZATION DRAW
//-------------------------------------------------------------------------------------------------------			
	NVIC_EnableIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART3_IRQn);
	
	for(i=0;i<N_POINTS;i++){	//	GENERATE AND SAFE POINTS OF SINE
	sample_table[i]=tone_init_samples(i);
	}
	sample_idx = 0;
	
	while(1){
		
		if(adc_ready==1){	//	ENABLE TO SEND THE MESSAGE TO LAVIEW				
			if(end_move==0){	// WAITING MODE 	
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);	//	WAITING UNTIL THE BUFFER IS EMPTY 
				LPC_UART3->THR='A';	//	INDICATE LABVIEW MODE WAITING AND SILENCE
			}
			else{	//	WORKING MODE
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
				LPC_UART3->THR='O';	//	INDICATE LABVIEW MODE WORKING
			}
			
			if(rx_buffer[pointer_to_data]=='R')	{		
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
				LPC_UART3->THR='F';	//	INDICATE LABVIEW MODE WORKING DO BEEP	
			}else{		
				if(end_move!=0){	//	WORKING MODE			
					switch(sel_song){	//	SELECT SONG
						case 0:
							while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
							LPC_UART3->THR='C';	//	INDICATE LABVIEW MODE WORKING DO MERRY CHRISTMAS	
							break;		
						case 1:
							while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
							LPC_UART3->THR='B';	//	INDICATE LABVIEW MODE WORKING DO ESPAÑA	
							break;			
						case 2:
							while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
							LPC_UART3->THR='D';	//	INDICATE LABVIEW MODE WORKING DO WHITE CHRISTMAS	
							break;			
						default:
							while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
							LPC_UART3->THR='H';	//	INDICATE LABVIEW MODE WORKING DO ERROR	
							break;						
					}
				}
				else{
					while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
					LPC_UART3->THR='A';	//	INDICATE LABVIEW MODE WAITING DO NOTHING 			
				}
			}
			for(loop=2;loop>=0;loop--){		//	ONLY 3 NUMBERS WE WILL SEND, FOR EXAMPLE 233=2.33
				while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);
				LPC_UART3->THR=(char)((uint16voltage%10)+'0');	//	CONVERT EACH NUMBER IN TO CHARACTER (THE MESSAGE WILL BE INVERTED ON LAVBIEW)
				uint16voltage/=10;
			}		
			adc_ready=0;	//	RESET FLAG			
		}
		
		if(key2==1){	//	CHANGE THE DISPLAY DRAW DEPENDS ON SEL_SONG			
			LCD_Clear(Cyan);
			Fc_display_draw(sel_song,Xpos,Ypos,line,dist_cm,angle,lcd_buffer);
			key2=0;		
		}
		
		if(tokencalib_1==1){	//	CALIBRATE SPEED OF MOTOR1	
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
				Fc_display_variable_value(speed1,speed2,voltage,Xpos,Ypos,line,dist_cm,angle,lcd_buffer);	//SHOW SPEED1, SPEED2 AND VOLTAGE	
				tokendisplay=0;
			}
//-------------------------------------------------------------------------------------------------------		
			tokendisplay++;
			tokencalib_1=0;	//	RESET FLAG				
		}
		
		if (tokencalib_2==1){	//	CALIBRATE SPEED OF MOTOR2			
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
			tokencalib_2=0;	//	RESET FLAG			
		}
		
		if(token&&message){	//	KEY1 HAS BEEN PRESSED AND A MESSAGE HAS REACHED 		
			if(pointer_to_data>=current_size){	//	THE ENTIRE MESSAGE WAS EXECUTED	
				end_move=0;	// RESET ALL THE VARIABLES USED ON THE COMUNICATION
				token=0;
				message=0;			
				free((void*)rx_buffer);	//	FREE UP SPACE FROM DYNAMIC MEMORY
				rx_buffer = NULL;			
				current_size=0;
				pointer_to_data=0;			
			}
			else if(pointer_to_data+2<current_size){				
				switch(rx_buffer[pointer_to_data]){
					case 'V':	//	DEFINE SPEED
						speed=(((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0'));
						pointer_to_data=pointer_to_data+Fc_speed_control(speed);
						end_move=1;
						break;				
					case 'D':	//	DEFINE RIGHT MOVEMENT
						angle=(uint16_t)((((uint16_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint16_t)(rx_buffer[pointer_to_data+2]-'0')));
						set_distance(K2*angle);
						Fc_reset_Tc();
						LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x1<<16);
						token=0;
						break;				
					case 'I':	//	DEFINE LEFT MOVEMENT
						angle=(uint16_t)((((uint16_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint16_t)(rx_buffer[pointer_to_data+2]-'0')));
						set_distance(K2*angle);
						Fc_reset_Tc();
						LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x2<<16);
						token=0;
						break;				
					case 'A':	//	DEFINE FORWARD MOVEMENT
						distance=(uint8_t)((((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0')));
						set_distance(distance);
						Fc_reset_Tc();
						LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);
						token=0;
						break;				
					case 'R':	//	DEFINE BACKWARDS MOVEMENT
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

