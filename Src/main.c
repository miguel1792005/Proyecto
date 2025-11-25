#include <LPC17xx.h>
#include <math.h>


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
#include "eint0_cfg.h"
#include "usb_cfg.h"

#define FCPU 25000000

#define size_of_array 9

#define N_OFFSETADC	0.24f
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
#define notes		24


//SE DEBE MEJORAR LA MR0 DE AMBOS TEMPORIZADORES LA FUNCION CALIB Y LA FUNCION SPEED SE AÑADIO UNA GANANCIA PARA CONTRARRESTAR LAS GRANDES DIFERENCIAS
//ENTRE AMBOS MOTORES ACTUALES

volatile char rx_buffer[size_of_array]={0};		// Reception Buffer 
volatile char data[size_of_array]={0};		// Real comunication
volatile uint8_t rx_index=0;
volatile uint8_t pointer_to_data=0; // Pointer to the array of data
volatile uint8_t token=0; // Start the movement after pressing the KEY1

volatile uint8_t speed;
volatile uint16_t distance;
volatile uint16_t angle;

volatile float gain1=1;
volatile float gain2=1;

volatile uint32_t speed1=0;		//speed motor1
volatile uint32_t speed2=0;		//speed motor2
volatile uint32_t CAP1_0=0;		//Value of last cap
volatile uint32_t CAP2_0=0;		//Value of last cap

float voltage=0;

uint32_t contador=0;

uint16_t Song[notes]={DO*2,DO*2,SOL,SOL,MI*2,MI*2,DO*2,SOL*2,FA*2,MI*2,RE*2,DO*2,DO*2,SI,LA,SOL,FA,FA,FA,FA,FA,FA,FA,FA};	 //ESPAÑA
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

//__________________________________________EINT0|BUTTON|KEY1______________________________________________
void EINT1_IRQHandler(){
	LPC_SC->EXTINT=0x2; // Clear flag of IRQ
	Fc_config_IRQ();
	token=1;
}

//__________________________________________IRQ______________________________________________
void TIMER0_IRQHandler(){	//Generate the sound signal with DAC
	if(LPC_TIM0->IR&((0x1))){		//Interrupt MR0 Show the DAC value
		LPC_TIM0->IR=(0x1);		//Clear flag of MR0 interrupt
		sound(sample_table[sample_idx]);
		sample_idx=(sample_idx==N_POINTS-1)?0:sample_idx+1;
		LPC_TIM0->MR0=(uint32_t)(LPC_TIM0->MR0+((FCPU/4)/(20*Song[index_song]))-1);
	}
	if(LPC_TIM0->IR&((0x1<<2))){		//Interrupt MR2 change the letter
		LPC_TIM0->IR=(0x1<<2);		//Clear flag of MR2 interrupt
		LPC_TIM0->MR0=(uint16_t)(((FCPU/4)/(20*Song[index_song]))-1);		//Start with DO	20 samples
		index_song=(index_song==(notes-1))?0:index_song+1;		
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
		
		if(pointer_to_data>=size_of_array){
			
			token=0;
			
		}

	}
}

/*void UART0_IRQHandler(){
	uint8_t data_index;
	if((LPC_UART0->IIR&0xE)==(0x04)){		//At least 1 interruption is pending and recive data available RDA
		rx_buffer[rx_index++]=LPC_UART0->RBR;		//Save the charapter on rx_buffer
		if(rx_index>=size_of_array){
			rx_index=0;
			for(data_index=0;data_index<size_of_array;data_index++){
				data[data_index]=rx_buffer[data_index];		// Save in array of data
			}
		}
	}
	
}*/

void UART3_IRQHandler(){
	uint8_t data_index;
	if((LPC_UART3->IIR&0xE)==(0x04)){		//At least 1 interruption is pending and recive data available RDA
		rx_buffer[rx_index++]=LPC_UART3->RBR;		//Save the charapter on rx_buffer
		if(rx_index>=size_of_array){
			rx_index=0;
			for(data_index=0;data_index<size_of_array;data_index++){
				data[data_index]=rx_buffer[data_index];		// Save in array of data
			}
		}
	}
}

void ADC_IRQHandler(){
	voltage=(float)(GAIN_ADC*(N_OFFSETADC+(float)3.3*(((float)((LPC_ADC->ADDR1>>4)&0xFFF))/(float)0xFFF)));		//Obtain value of voltage
	LPC_TIM1->MR1=(LPC_TIM1->MR1)+400000;		//Read ADC each 40s*2=80s
}
//___________________________________________________________________________________________
int main(){
	Fc_config_pines();
	Fc_config_TIMER();
	Fc_config_PWM();
  Fc_bluetooth_communication();
	dac_cfg();
	adc_cfg();
	eint0_cfg();
	usb_cfg();
	
	NVIC_EnableIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART3_IRQn);
	
	tone_init_samples();
	
	while(1){
		if(token==1){
			
			switch(data[pointer_to_data]){

			case 'V':		//Define speed
				speed=(((uint8_t)(data[pointer_to_data+1]-'0'))*10+(uint8_t)(data[pointer_to_data+2]-'0'));
				pointer_to_data=pointer_to_data+Fc_speed_control(speed);

			break;
			
			case 'D':		//Define right movement
				angle=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(K2*angle);
				LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
				LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
				NVIC_EnableIRQ(TIMER1_IRQn);
				NVIC_EnableIRQ(TIMER2_IRQn);
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x1<<16);
				token=0;

			break;
			
			case 'I':		//Define left movement
				angle=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(K2*angle);
				LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
				LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
				NVIC_EnableIRQ(TIMER1_IRQn);
				NVIC_EnableIRQ(TIMER2_IRQn);
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x2<<16);
				token=0;

			break;
			
			case 'A':		//Define forward movement
				distance=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(distance);
				LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
				LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
				NVIC_EnableIRQ(TIMER1_IRQn);
				NVIC_EnableIRQ(TIMER2_IRQn);
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);
				token=0;

			break;
			
			case 'R':		//Define backwards movement
				distance=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
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

