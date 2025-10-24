#include <LPC17xx.h>
#include "Fc_config_pines.h"
#include "Fc_config_PWM.h"
#include "Fc_speed_control.h"
#include "Fc_config_TIMER.h"
#include "Fc_config_IRQ.h"
#include "Fc_bluetooth_communication.h"
#include "calib.h"
#include "set_distance.h"
#define Frase_serial 15

char rx_buffer[Frase_serial]={0};		//Buffer recepción
//char Frase[6]={'V','4','4','R','0','4'};		//Prueba
char Frase1_1[Frase_serial]={0};		//Real comunication
uint8_t rx_index=0;

uint8_t puntero_frase=0;
uint32_t pasos_motor1x2=0;
uint32_t pasos_motor2x2=0;

float gain1=1;
float gain2=1;

uint8_t velocidad;
uint16_t distance; // variable de distancia
uint16_t angle;
uint8_t flag=1;


//__________________________________________IRQ______________________________________________
void TIMER1_IRQHandler(){	//Motor (1) Fastest, right side if you see the front of the car, PWM1.2 P1.20 / CAP1.0 P1.18
	if(LPC_TIM1->IR&((0x1<<0))){		//Interrupt MR0
		LPC_TIM1->IR=(0x1<<0);		//Clear flag of MR0
		
		
		if(!(0 <= ((LPC_TIM1->TC)-(LPC_TIM2->TC)) && ((LPC_TIM1->TC)-(LPC_TIM2->TC)) <= 2)){ //Comprobacion de que uno va mas deprisa
				
			if((LPC_TIM1->TC) > (LPC_TIM2->TC) ){ // Motor 1 mas rapido
				
				//gain1=gain1*0.99; // Decremento de la ganancia del 1

				if((LPC_PWM1->MR4*gain2*1.0001)<LPC_PWM1->MR0){ // Comprobacion de que el tiempo en alto es menor que el periodo
					gain2=gain2*1.0001; // Incremento de la ganancia del 2
					
					calib(gain1,gain2,velocidad); // Funcion calibracion
					
				if((LPC_PWM1->MR4*gain2*1.0001)>LPC_PWM1->MR0){
					
					gain2=gain2*0.9999;
					calib(gain1,gain2,velocidad);
				}
					
				}
			}
			
			/*if((LPC_TIM2->TC) > (LPC_TIM1->TC)){ // Motor 2 mas rapido
			
				//gain2=gain2*0.99; // Decremento de la ganancia del 2
					
				if(LPC_PWM1->MR2<LPC_PWM1->MR0){ // Comprobacion de que el tiempo en alto es menor que el periodo
					gain1=gain1*1.01; // Incremento de la ganancia del 1
				}
				
			}
		*/
			
			
			
		}
		
		LPC_TIM1->MR0 += 2; // Set MR0 according 1/11 revolution to calibrate the speed
		
		
	}
}
void TIMER2_IRQHandler(){	//Motor (2) Slowest, left side if you see the front of the car, PWM1.4 P1.23 / CAP2.0 P0.4
	if(LPC_TIM2->IR&((0x1<<0))){		//Interrupt MR0
		LPC_TIM2->IR=(0x1<<0);		//Clear flag of MR0

		LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)));// Parada de prueba
		
		flag=1;// Pasar a siguiente instruccion

		puntero_frase+=3;
		
		
		
		
	}
}
void UART3_IRQHandler(void) {
	uint8_t indice_frase;
	if((LPC_UART3->IIR&0xE)==(0x04)){		//At least 1 interruption is pending and recive data available RDA
		rx_buffer[rx_index++]=LPC_UART3->RBR;		//Save the charapter on rx_buffer
		if(rx_index>=Frase_serial){
			rx_index=0;
			for(indice_frase=0;indice_frase<Frase_serial;indice_frase++){
				Frase1_1[indice_frase]=rx_buffer[indice_frase];		//Trasladamos a Frase1_1 la que usaremos en nuestro programa
			}
		}
	}
}
//___________________________________________________________________________________________

int main(){
	//WE NEED A FUNCTION TO SET THE PRIORITY AND SUBPRIORITY EACH INTERRUPTIONS
	Fc_config_pines();
	Fc_config_IRQ();
	Fc_config_TIMER();
	Fc_config_PWM();
	
	
  Fc_bluetooth_communication();
	
	
	
	/*
	while(1){
		switch(Frase[puntero_frase]){
			case 'V':		//Define speed
				puntero_frase=puntero_frase+Fc_speed_control((((uint8_t)(Frase[puntero_frase+1]-'0'))*10+(uint8_t)(Frase[puntero_frase+2]-'0')));
			break;
			case 'I':		//Define left movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x1<<16);
			break;
			case 'D':		//Define right movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x2<<16);
			break;
			case 'A':		//Define forward movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);	
			break;
			case 'R':		//Define backwards movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x2<<16);	
			break;


		}
	}
	
	*/
	
	
	
	while(1){
		if(flag==1){
			switch(Frase1_1[puntero_frase]){
			
			case 'V':		//Define speed
				velocidad=(((uint8_t)(Frase1_1[puntero_frase+1]-'0'))*10+(uint8_t)(Frase1_1[puntero_frase+2]-'0'));
				puntero_frase=puntero_frase+Fc_speed_control(velocidad);			
			break;
			case 'D':		//Define left movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x1<<16);
				angle=(uint16_t)((((uint16_t)(Frase1_1[puntero_frase+1]-'0'))*10+(uint16_t)(Frase1_1[puntero_frase+2]-'0')));
				set_distance(8*((angle)*(3.141592654/180)));
				flag=0;
			break;
			case 'I':		//Define right movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x2<<16);
				angle=(uint16_t)((((uint16_t)(Frase1_1[puntero_frase+1]-'0'))*10+(uint16_t)(Frase1_1[puntero_frase+2]-'0')));
				set_distance(8*((angle)*(3.141592654/180)));
				flag=0;
			break;
			case 'A':		//Define forward movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);	
				distance=(uint16_t)((((uint16_t)(Frase1_1[puntero_frase+1]-'0'))*10+(uint16_t)(Frase1_1[puntero_frase+2]-'0')));
				set_distance(distance);
				flag=0;
			break;
			case 'R':		//Define backwards movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x2<<16);
				distance=(uint16_t)((((uint16_t)(Frase1_1[puntero_frase+1]-'0'))*10+(uint16_t)(Frase1_1[puntero_frase+2]-'0')));
				set_distance(distance);
				flag=0;
			break;
			default:
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)));// Parada de prueba
			break;
			
	
			
			}
		}
			

			
			
			
			
			/*
			if(flag1==1 && flag2==1){
				
				paso++;
			
				distanceA=(paso/(11*35))*2*pi*33.5;  //distance in mm
				
			}
			
			else if(flag1==2 && flag2==2){
			
				paso--;
			
				distanceR=(paso/11)*2*pi*33.5;
			
			}
			*/
			
	}
	
}

