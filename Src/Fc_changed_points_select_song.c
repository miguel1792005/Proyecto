#include <LPC17xx.h>

//	----------------------------------------------------------------------------------------------------
//	1. SYSTEM DEFINITIONS
//	----------------------------------------------------------------------------------------------------

#define F_CPU       (SystemCoreClock)
#define FCPU 25000000
#define F_PCLK      (F_CPU/4)

//	----------------------------------------------------------------------------------------------------
//	2. AUDIO(DAC) CONSTANTS
//	----------------------------------------------------------------------------------------------------

#define N_POINTS 20
//	NOTE FREQUENCIES AND ARRAY SIZE OF SONGS
#define DO	260
#define RE	290
#define MI	330
#define FA	350
#define SOL	390
#define LA	440
#define SI	490

#define notesnav	20
#define notesesp	29
#define notesblan	30
#define notesb	12

//	----------------------------------------------------------------------------------------------------
//	3. SONG LOCAL VARIABLES
//	----------------------------------------------------------------------------------------------------

static const uint16_t navidad[notesnav]={SOL,DO*2,DO*2,SI,DO*2,LA,LA,LA,LA,0,LA,RE*2,RE*2,DO*2,LA,SOL,SOL,SOL,SOL,0};	//	FELIZ NAVIDAD
static const uint16_t Espana[notesesp]={DO*2,DO*2,SOL,SOL,MI*2,MI*2,DO*2,SOL*2,FA*2,MI*2,RE*2,DO*2,DO*2,SI,LA,SOL,DO*2,DO*2,RE*2,RE*2,MI*2,MI*2,SOL*2,FA*2,MI*2,RE*2,DO*2,SOL*2,SOL*2};	//	ESPAÑA
static const uint16_t blanca[notesblan]={LA,LA,LA,LA,LA+20,LA,SOL+20,LA,LA+20,LA+20,LA+20,LA+20,SI,DO*2,DO*2,DO*2,DO*2,RE,MI,FA,SOL,FA,MI,RE,DO,DO,DO,DO,DO,DO};	//	OH BLANCA NAVIDAD
static const uint16_t Beep[notesb]={600,500,600,500,600,500,600,500,600,500,600,500}; //	BEEP WHILE BACKWARDS MOVING IS PERFORMED
static const uint16_t Silence=1;	//	SILENCE

void Fc_changed_points_select_song(char rx_buffer,uint8_t end_move,uint8_t sel_song,uint8_t index_song){
	
	if(rx_buffer!='R'&&end_move!=0){	//	CHANGE THE FRECUENCY OF THE SINE DAC WITH SEL_SONG 
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
	if(rx_buffer=='R'){					
		LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((F_PCLK)/(N_POINTS*Beep[index_song]))-1);	//	BEEP WHEN BACKWARD MOVEMENT, EACH POINT OF SINE, WORKING MODE
	}
}	
