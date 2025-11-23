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


#define size_of_array 15

#define N_OFFSETADC 0.24


#define PI 3.1415926535897932384626433832795f
#define K2 8*(PI/180)


#define N_POINTS 20
#define DAC_N_BITS 10
#define DAC_N_LEVELS (1U << DAC_N_BITS)
#define DAC_MID_RANGE (1U << (DAC_N_BITS-1))
#define PI 3.1415926535897932384626433832795f




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


static uint16_t sample_table[N_POINTS];		//array of values of a sine signal
static int sample_idx;										//index pointing to the last output through DAC
uint16_t point;														//value of the array to be reproduce




void tone_init_samples() {
		int i;
		float x;
	
		for(i = 0; i < N_POINTS; i++) {
				x = DAC_MID_RANGE+(DAC_MID_RANGE-1)*sinf((2*PI/(N_POINTS))*i);
				sample_table[i] = ((uint16_t)x) << 6;
		}
		
		sample_idx = 0;
		
}


//__________________________________________EINT0|BUTTON|KEY1______________________________________________
void EINT0_IRQHandler(){
	
	token=1;
		
}


//__________________________________________IRQ______________________________________________
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
			
			LPC_SC->EXTINT=1; // Wait for another push button to start a new cycle
			token=0;
			
		}

	}
}
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
	
	
	voltage = N_OFFSETADC+(float)3.3*(((float)((LPC_ADC->ADDR1 >> 4) & 0xFFF))/(float)0xFFF);//Obtain value of voltage
	
	LPC_TIM1->MR1 = (LPC_TIM1->MR1)+400000; //Read ADC each 40s*2=80s
	
}




void PWM1_IRQHandler(){
	
	if(LPC_PWM1->IR&(0x1<<0)){
		
		LPC_PWM1->IR=(0x1<<0);
			
		contador=contador+1;
		if(contador>500*N_POINTS){//5000=500*10cycles
			
			point=sample_table[0];
			
		}
		
		else{
		
			point=sample_table[sample_idx];
			
			sound(point);
			
			sample_idx = (sample_idx == N_POINTS-1)? 0: sample_idx+1;

		}
		
		
		
		if(contador>1000*N_POINTS){
			
			contador=0;
			
			
		}
		
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
	eint0_cfg();
	
	tone_init_samples();
	
	
	while(1){
		if(token==1){
			
			Fc_config_IRQ();
			
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

