//	----------------------------------------------------------------------------------------------------
//	1. SYSTEM AND STANDARD LIBRARIES
//	----------------------------------------------------------------------------------------------------

#include <LPC17xx.h>	//	DEVICE LIBRARY
#include <stdlib.h>	//	LIBRARY FOR DYNAMIC MEMORY (MALLOC, FREE, NULL)
#include <stdio.h>	//	LIBRARY FOR INPUT OUTPUT

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
#include "Send_labview.h"
#include "Fc_display_message.h"
#include "Fc_changed_points_select_song.h"
#include "Fc_changed_letter_select_song.h"
#include "pid1.h"
#include "pid2.h"

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
#define K2 (9.0f * (PI/180.f))	//	CONVERSION FROM ANGLE TO DISTANCE OF WHEELS
#define K2_H ((PI*40.0f)/(180.0f))	//	CONVERSIÓN FROM ANGLE TO DISTANCE OF WHEELS ON NON-HOLONOMIC
#define K3 (9.0f * (PI/180.f))	//	CONVERSION FROM ANGLE TO DISTANCE OF WHEELS
#define K3_H ((PI*20.0f)/(180.0f))	//	CONVERSIÓN FROM ANGLE TO DISTANCE OF WHEELS ON NON-HOLONOMIC

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

#define FONT_H  16
static char lcd_buffer[256];	//	BUFFER FOR LCD TEXT

//	----------------------------------------------------------------------------------------------------
//	9. AUDIO(DAC) CONSTANTS
//	----------------------------------------------------------------------------------------------------

#define N_POINTS 20

//	----------------------------------------------------------------------------------------------------
//	10. COMMUNICATION VARIABLES
//	----------------------------------------------------------------------------------------------------

volatile uint8_t *rx_buffer=NULL;	//	DYNAMIC BUFFER FOR SAVING THE MESSAGE
volatile uint16_t current_size=0;	//	VAR. TO SAVE THE SIZE OF THE DYNAMIC BUFFER
volatile uint16_t pointer_to_data=0;	//	POINTER FOR READING THE RECEIVED MESSAGE
volatile uint8_t message=0; //	FLAG ACTIVE WHEN THE MESSAGE HAS BEEN RECEIVED
int8_t loop=2;	//	COUNTER TO SEND 3 CHARACTERS OF VOLTAGE AND SPEED
uint8_t powers_10[3]={1,10,100};	//	POWERS OF 10 TO SEND SPEED

//	----------------------------------------------------------------------------------------------------
//	11. CONTROL FLAGS
//	----------------------------------------------------------------------------------------------------

volatile uint8_t token=0; // FLAG TO START THE MOVEMENT AFTER PRESSING KEY1
volatile uint8_t tokencalib_1=0; //	FLAG THAT ACTIVATES THE CALIBRATION OF WHEEL 1
volatile uint8_t tokencalib_2=0; //	FLAG THAT ACTIVATES THE CALIBRATION OF WHEEL 2
volatile uint8_t end_move=0;	// FLAG TO DETECT THE STATE OF THE ROBOT
volatile uint8_t adc_ready=0;	//	FLAG TO DETECT THE READ OF ADC
volatile uint8_t key2=0;	//	FLAG TO DETECT SELECTION OF SONG CHANGED
volatile uint8_t tokenhol=0;	//	FLAG FOR HOLONOMIC MOVEMENTS
volatile uint8_t flagmessage=0;	//	FLAG FOR SENDING MESSAGE OF PUSH BUTTON TO DISPLAY
volatile uint8_t flagmessageUART0=0;	//	FLAG FOR SENDING MESSAGE TO LABVIEW UART3
volatile uint8_t flagmessageUART3=0;	//	FLAG FOR SENDING MESSAGE TO LABVIEW UART0

//	----------------------------------------------------------------------------------------------------
//	12. MOTOR CALIBRATION VARIABLES
//	----------------------------------------------------------------------------------------------------

volatile uint8_t duty;	//	VAR. THAT STORES THE PORCENTUAL SPEED RECEIVED (CHAR->UINT8_T)
volatile uint8_t angle;	//	VAR. THAT STORES THE ANGLE RECEIVED (CHAR->UINT8_T)
volatile uint8_t distance;	// VAR. THAT STORES THE DISTANCE RECEIVED (CHAR->UINT8_T)
volatile uint32_t rising_edge_1=0;	//	VAR. TO STORE THE VALUE OF EDGES EACH 10MS (MOTOR 1)
volatile uint32_t rising_edge_2=0;	//	VAR. TO STORE THE VALUE OF EDGES EACH 10MS (MOTOR 2)
volatile float velocity1=0;	//	VAR. TO STORE VELOCITY OF MOTOR 1 (m/s)
volatile float velocity2=0;	//	VAR. TO STORE VELOCITY OF MOTOR 2 (m/s)
volatile float velocity1_1;	//	VAR. AUXILIAR VARIABLE SEND THE VELOCITY TO UART
volatile float velocity2_1;	//	VAR. AUXILIAR VARIABLE SEND THE VELOCITY TO UART
volatile float error1=0;	//	VAR. TO STORE THE ERROR OF VELOCITY OF MOTOR 1
volatile float error2=0;	//	VAR. TO STORE THE ERROR OF VELOCITY OF MOTOR 2
float refvelocity_1=0;	//	VAR. TO STORE THE REFERENCE VELOCITY
float refvelocity_2=0;	//	VAR. TO STORE THE REFERENCE VELOCITY
float dprev1=0;	//	TEMP. VAR. TO STORE THE DUTY CYCLE BETWEEN CYCLES OF CALIBRATION OF MOTOR 1
float dprev2=0;	//	TEMP. VAR. TO STORE THE DUTY CYCLE BETWEEN CYCLES OF CALIBRATION OF MOTOR 2
float accumulated_integra_1=0;	//	VAR. TO STORE ACUMULATED INTEGRAL OF ERROR ON PID 1
float accumulated_integra_2=0;	//	VAR. TO STORE ACUMULATED INTEGRAL OF ERROR ON PID 1
float errorprev1=0;	//	TEMP. VAR. TO STORE THE ERROR BETWEEN CYCLES OF CALIBRATION OF MOTOR 1
float errorprev2=0;	//	TEMP. VAR. TO STORE THE ERROR BETWEEN CYCLES OF CALIBRATION OF MOTOR 2

//	----------------------------------------------------------------------------------------------------
//	13. DISPLAY VARIABLES
//	----------------------------------------------------------------------------------------------------

volatile float voltage=0;	//	VOLTAGE OF BATTERY MEASURE FROM ADC
volatile uint16_t uint16voltage;	//	TEMPORARY VARIABLE TO SEND 3 DIGITS OF VOLTAGE THROUGH UART

//	----------------------------------------------------------------------------------------------------
//	14. SONG VARIABLES
//	----------------------------------------------------------------------------------------------------

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
		Fc_changed_points_select_song(rx_buffer[pointer_to_data],end_move,sel_song,index_song);
	}	
	
	if(LPC_TIM0->IR&((0x1<<2))){	//	MATCH MR2 CHANGE THE LETTER
		LPC_TIM0->IR=(0x1<<2);	//	CLEAR FLAG OF MR2 INTERRUPT
		index_song=Fc_changed_letter_select_song(rx_buffer[pointer_to_data],end_move,sel_song,index_song);
	}	
}
void TIMER1_IRQHandler(){	//	(CONTROL SPEED1)
	if(LPC_TIM1->IR&((0x1<<4))){	//	INTERRUPT CAP0 (CALIB SPEED)
		LPC_TIM1->IR=(0x1<<4);	//	CLEAR FLAG OF CAP0 INTERRUT			
		rising_edge_1++;		
	}
	if(LPC_TIM1->IR&(0x1)){	//	MATCH 0.1S TAKE THE MEASURE OF WHEEL
		LPC_TIM1->IR=(0x1);		
		velocity1=(float)(0.0057119*(rising_edge_1));		
		tokencalib_1=1;	//	FLAG TO CHANGE THE SPEED ON MAIN
		rising_edge_1=0;
		LPC_TIM1->TC=0;
	}
}
void TIMER2_IRQHandler(){	//	(CONTROL SPEED2)
	if(LPC_TIM2->IR&((0x1<<4))){	//	INTERRUPT CAP0 (CALIB SPEED)
		LPC_TIM2->IR=(0x1<<4);	//	CLEAR FLAG OF CAP0 INTERRUT		
		rising_edge_2++;	
	}
	if(LPC_TIM2->IR&(0x1)){	//	MATCH 0.1S TAKE THE MEASURE OF WHEEL
		LPC_TIM2->IR=(0x1);
		velocity2=(float)(0.0057119*(rising_edge_2));
		tokencalib_2=1;	//	FLAG TO CHANGE THE SPEED ON MAIN		
		rising_edge_2=0;
		LPC_TIM2->TC=0;
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
		rising_edge_1=0;		
		rising_edge_2=0;		
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

		if(message==0){
			if((received_char0!=END1)&(received_char0!=END2)){	//	IT IS EXECUTED IF THERE IS NO "LINE BREAK" IN THE MESSAGE
				temp_pointer0=(uint8_t *)realloc((void*)rx_buffer,current_size+1);	//	INCREMENT THE DINAMIC MEMMORY WITH FC REALLOC
				if(temp_pointer0!=NULL){	//	SAVE THE CHARACTER ON RX_BUFFER WHEN THERE IS SPACE
					rx_buffer=temp_pointer0;			
					rx_buffer[current_size]=received_char0;
					current_size++;							
				}		
			}			
			if(((received_char0==END1)||(received_char0==END2))){	//	WHEN "LINE BREAKS" COMES THE MESSAGE IS FULL RECIVED		
				message=1;
				flagmessage=1;
				flagmessageUART0=1;
			}
		}else{
			NVIC_SetPendingIRQ(EINT2_IRQn);	//	CHANGE THE MUSIC FROM LAVBIEW
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
				if(temp_pointer3!=NULL){	//	SAVE THE CHARACTER ON RX_BUFFER WHEN THERE IS SPACE			
					rx_buffer=temp_pointer3;			
					rx_buffer[current_size]=received_char3;
					current_size++;				
				}			
			}			
			if(((received_char3==END1)||(received_char3==END2))){	//	WHEN "LINE BREAKS" COMES THE MESSAGE IS FULL RECIVED					
				message=1;	
				flagmessage=1;
				flagmessageUART3=1;
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
	Fc_config_pines();	// ALL CONFIGURATION FC
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
	Fc_display_variable_value(velocity1,velocity2,voltage,distance,lcd_buffer);	//	INITIALIZATION VARIABLE VALUES ON DISPLAY
	Fc_display_draw(sel_song,lcd_buffer);	//	INITIALIZATION DRAW
	Fc_display_message("      WAIT      ",100,218,lcd_buffer,NULL);	//	PRINT "WAIT" ON DISPLAY
//-------------------------------------------------------------------------------------------------------			
	NVIC_EnableIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART3_IRQn);
	
	for(i=0;i<N_POINTS;i++){	//	GENERATE AND SAVE POINTS OF SINE
	sample_table[i]=tone_init_samples(i);
	}
	sample_idx = 0;
	
	while(1){
//--------------------------------------------------------SEND DATA TO LABVIEW--------------------------------------------------------	
		if(adc_ready==1){	//	ENABLE TO SEND THE MESSAGE TO LAVIEW				
			if(flagmessage==0&&end_move==0){	// WAITING MODE 	
				tokenhol=0;	
				Fc_display_message("      WAIT      ",100,218,lcd_buffer,NULL);	//	PRINT "WAIT" ON DISPLAY
				Fc_display_message("LAST DISTANCE: %d cm",40,2*FONT_H,lcd_buffer,distance);		//	PRINT LAST DISTANCE ON DISPLAY
				Send_labview('A',flagmessageUART0,flagmessageUART3);	//	SEND CHARACTER A TO LABVIEW MODE "WAITING"
			}
			if(flagmessage==0&&end_move==1){	//	WORKING MODE	
				Fc_display_message("    WORKING    ",100,218,lcd_buffer,NULL);	//	PRINT "WORKING" ON DISPLAY							
				Send_labview('O',flagmessageUART0,flagmessageUART3);	//	SEND CHARACTER O TO LABVIEW MODE "WORKING"
			}
			
			if(rx_buffer[pointer_to_data]=='R'){		
				Send_labview('F',flagmessageUART0,flagmessageUART3);	//	SEND CHARACTER F TO LABVIEW MODE "WORKING" DO BEEP
			}else{		
				if(end_move!=0){	//	WORKING MODE			
					switch(sel_song){	//	SELECT SONG
						case 0:
							Send_labview('C',flagmessageUART0,flagmessageUART3);	//	INDICATE LABVIEW MODE WORKING DO MERRY CHRISTMAS
							break;		
						case 1:
							Send_labview('B',flagmessageUART0,flagmessageUART3);	//	INDICATE LABVIEW MODE WORKING DO ESPAÑA	
							break;			
						case 2:
							Send_labview('D',flagmessageUART0,flagmessageUART3);		//	INDICATE LABVIEW MODE WORKING DO WHITE CHRISTMAS	
							break;			
						default:
							Send_labview('H',flagmessageUART0,flagmessageUART3);	//	INDICATE LABVIEW MODE WORKING DO ERROR			
							break;						
					}
				}
				else{
					Send_labview('A',flagmessageUART0,flagmessageUART3);	//	INDICATE LABVIEW MODE WAITING DO NOTHING					
				}
			}
			for(loop=2;loop>=0;loop--){		//	ONLY 3 NUMBERS WE WILL SEND, FOR EXAMPLE 332=2.33 VOLTAGE
				Send_labview((char)((uint16voltage%10)+'0'),flagmessageUART0,flagmessageUART3);	//	CONVERT EACH NUMBER IN TO CHARACTER (THE MESSAGE WILL BE INVERTED ON LAVBIEW)
				uint16voltage/=10;
			}			
			velocity1_1=velocity1*1000;
			velocity2_1=velocity2*1000;					
			for(loop=2;loop>=0;loop--){		//	ONLY 3 NUMBERS WE WILL SEND, FOR EXAMPLE 321=0.321 CM/S SPEED
				Send_labview((char)(((((uint32_t)((velocity1_1)/powers_10[loop]))%10))+'0'),flagmessageUART0,flagmessageUART3);	//	CONVERT EACH NUMBER IN TO CHARACTER 
			}		
			for(loop=2;loop>=0;loop--){		//	ONLY 3 NUMBERS WE WILL SEND, FOR EXAMPLE 333=0.333 CM/S SPEED
				Send_labview((char)(((((uint32_t)((velocity2_1)/powers_10[loop]))%10))+'0'),flagmessageUART0,flagmessageUART3);	//	CONVERT EACH NUMBER IN TO CHARACTER 
			}					
			adc_ready=0;	//	RESET FLAG			
		}
//------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------PRINT PUSH BUTTON ON DISPLAY----------------------------------------------------	
		if(flagmessage==1&&end_move==0){						
			Fc_display_message("  PUSH BUTTON  ",100,218,lcd_buffer,NULL);	//	PRINT "PUSH BUTTON" ON DISPLAY		
		}
//------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------RESET DRAW OF DISPLAY-------------------------------------------------------	
		if(key2==1){	//	CHANGE THE DISPLAY DRAW DEPENDS ON SEL_SONG			
			LCD_Clear(Cyan);
			Fc_display_draw(sel_song,lcd_buffer);
			key2=0;		
		}
//------------------------------------------------------------------------------------------------------------------------------------	
//------------------------------------------------------CALIBRATE SPEED OF WHEELS-----------------------------------------------------				
		if(message==1){	//	MESSAGE HAS REACHED
			if(tokencalib_1||tokencalib_1){	//	UPDATE VARIABLES SHOWN ON THE SCREEN EACH 0.1 SECONDS
				Fc_display_variable_value(velocity1,velocity2,voltage,distance,lcd_buffer);	//SHOW SPEED1, SPEED2 AND VOLTAGE	
			}
			if(tokencalib_1==1){	//	CALIBRATE SPEED OF MOTOR1 IT IS NOT A HOLONOMIC ROTATING MOTION
				error1=refvelocity_1-velocity1;
				dprev1=pid1(error1,errorprev1,&accumulated_integra_1);
				errorprev1=error1;	
				tokencalib_1=0;	//	RESET FLAG
			}
			if(tokencalib_2==1){	//	CALIBRATE SPEED OF MOTOR2	IT IS NOT A HOLONOMIC ROTATING MOTION		
				error2=refvelocity_2-velocity2;
				dprev2=pid2(error2,errorprev2,&accumulated_integra_2);
				errorprev2=error2;
				tokencalib_2=0;	//	RESET FLAG			
			}
//------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------MOVEMENT INSTRUCTIONS-------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
			if(token==1){	//	KEY1 HAS BEEN PRESSED 
				if(pointer_to_data>=current_size){	//	THE ENTIRE MESSAGE WAS EXECUTED	
					end_move=0;	// RESET ALL THE VARIABLES USED 
					token=0;
					message=0;			
					free((void*)rx_buffer);	//	FREE UP SPACE FROM DYNAMIC MEMORY
					rx_buffer = NULL;			
					current_size=0;
					pointer_to_data=0;
					refvelocity_1=0;
					refvelocity_2=0;
					velocity1=0;
					velocity2=0;
					accumulated_integra_1=0;
					accumulated_integra_2=0;			
				}
				else if(pointer_to_data+2<current_size){				
					switch(rx_buffer[pointer_to_data]){
						case 'H':
							tokenhol=1;
							pointer_to_data++;
						case 'V':	//	DEFINE SPEED
							duty=(((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0'));
							dprev1=duty;
							dprev2=duty;
							Fc_speed_control(duty,duty,&dprev1,&dprev2,&refvelocity_1,&refvelocity_2);
							pointer_to_data=pointer_to_data+3;
							if(duty==0){
								pointer_to_data=current_size;	//	SPEED 0% THEN STOP THE PROCESS
							}else{
								end_move=1;
								flagmessage=0;	//	RESET FLAG AFTER WRITE PUSH BUTTON ON DISPLAY
							}
							break;				
						case 'D':	//	DEFINE RIGHT MOVEMENT
							angle=(uint16_t)((((uint16_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint16_t)(rx_buffer[pointer_to_data+2]-'0')));					
							Fc_display_message("DIRECTION: >",220,32,lcd_buffer,NULL);	//	DIRECTION													
							Fc_reset_Tc();
							if(tokenhol==0){
								Fc_speed_control(duty,duty,&dprev1,&dprev2,&refvelocity_1,&refvelocity_2);
								LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x1<<16);
								set_distance((uint8_t)(K2*angle));
							}else{	
								Fc_speed_control(99,1,&dprev1,&dprev2,&refvelocity_1,&refvelocity_2);								
								LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);
								set_distance((uint8_t)(angle*K2_H));				
							}
							token=0;
						break;				
						case 'I':	//	DEFINE LEFT MOVEMENT
							angle=(uint16_t)((((uint16_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint16_t)(rx_buffer[pointer_to_data+2]-'0')));						
							Fc_display_message("DIRECTION: <",220,32,lcd_buffer,NULL);	//	DIRECTION											
							Fc_reset_Tc();
							if(tokenhol==0){
								Fc_speed_control(duty,duty,&dprev1,&dprev2,&refvelocity_1,&refvelocity_2);
								LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x2<<16);
								set_distance((uint8_t)(K3*angle));
							}else{
								Fc_speed_control(1,99,&dprev1,&dprev2,&refvelocity_1,&refvelocity_2);							
								LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);
								set_distance((uint8_t)(angle*K3_H));
							}		
							token=0;
						break;				
						case 'A':	//	DEFINE FORWARD MOVEMENT
							distance=(uint8_t)((((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0')));
							Fc_display_message("DIRECTION: v",220,32,lcd_buffer,NULL);	//	DIRECTION							
							set_distance(distance);
							Fc_reset_Tc();					
							Fc_speed_control(duty,duty,&dprev1,&dprev2,&refvelocity_1,&refvelocity_2);				
							LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);
							token=0;
						break;				
						case 'R':	//	DEFINE BACKWARDS MOVEMENT
							distance=(uint8_t)((((uint8_t)(rx_buffer[pointer_to_data+1]-'0'))*10+(uint8_t)(rx_buffer[pointer_to_data+2]-'0')));					
							Fc_display_message("DIRECTION: ^",220,32,lcd_buffer,NULL);	//	DIRECTION													
							set_distance(distance);
							Fc_reset_Tc();
							Fc_speed_control(duty,duty,&dprev1,&dprev2,&refvelocity_1,&refvelocity_2);					
							LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x2<<16);
							token=0;					
						break;				
						default:
							LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16))); // Stop
						break;				
					}											
				}
			}
//------------------------------------------------------------------------------------------------------------------------------------
		}
	}
}

