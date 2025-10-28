#include <LPC17xx.h>
#include "Fc_config_pines.h"
#include "Fc_config_PWM.h"
#include "Fc_speed_control.h"
#include "Fc_config_TIMER.h"
#include "Fc_config_IRQ.h"
#include "Fc_bluetooth_communication.h"
#include "calib.h"
#include "set_distance.h"
#define size_of_array 9

//SE DEBE MEJORAR LA MR0 DE AMBOS TEMPORIZADORES LA FUNCION CALIB Y LA FUNCION SPEED SE AÑADIO UNA GANANCIA PARA CONTRARRESTAR LAS GRANDES DIFERENCIAS
//ENTRE AMBOS MOTORES ACTUALES

volatile char rx_buffer[size_of_array]={0};		// Reception Buffer 
volatile char data[size_of_array]={0};		// Real comunication
volatile uint8_t rx_index=0;
volatile uint8_t pointer_to_data=0; // Pointer to the array of data
volatile uint8_t enclav=1;

volatile uint8_t speed;
volatile uint16_t distance;
volatile uint16_t angle;

volatile float gain1=1;
volatile float gain2=1;

volatile uint32_t speed1=0;		//speed motor1
volatile uint32_t speed2=0;		//speed motor2
volatile uint32_t CAP1_0=0;		//Value of last cap
volatile uint32_t CAP2_0=0;		//Value of last cap

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
		enclav=1;

	}
}
void UART3_IRQHandler(void){
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
//___________________________________________________________________________________________

int main(){
	Fc_config_pines();
	Fc_config_IRQ();
	Fc_config_TIMER();
	Fc_config_PWM();
  Fc_bluetooth_communication();
	
	while(1){
		if(enclav==1){
			switch(data[pointer_to_data]){

			case 'V':		//Define speed
				speed=(((uint8_t)(data[pointer_to_data+1]-'0'))*10+(uint8_t)(data[pointer_to_data+2]-'0'));
				pointer_to_data=pointer_to_data+Fc_speed_control(speed);			
			break;
			
			case 'D':		//Define right movement
				angle=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(8*((angle)*(3.141592654/180)));
				LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
				LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
				NVIC_EnableIRQ(TIMER1_IRQn);
				NVIC_EnableIRQ(TIMER2_IRQn);
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x1<<16);
				enclav=0;

			break;
			
			case 'I':		//Define left movement
				angle=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(8*((angle)*(3.141592654/180)));
				LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
				LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
				NVIC_EnableIRQ(TIMER1_IRQn);
				NVIC_EnableIRQ(TIMER2_IRQn);
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x2<<16);
				enclav=0;

			break;
			
			case 'A':		//Define forward movement
				distance=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(distance);
				LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
				LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
				NVIC_EnableIRQ(TIMER1_IRQn);
				NVIC_EnableIRQ(TIMER2_IRQn);
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);
				enclav=0;

			break;
			
			case 'R':		//Define backwards movement
				distance=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(distance);
				LPC_TIM1->TCR=(0x1);		//Timer counter and prescaler enable to counting
				LPC_TIM2->TCR=(0x1);		//Timer counter and prescaler enable to counting
				NVIC_EnableIRQ(TIMER1_IRQn);
				NVIC_EnableIRQ(TIMER2_IRQn);
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x2<<16);
				enclav=0;

			break;
			
			default:
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16))); // Stop
			break;
			
			}
		}
	}
}

