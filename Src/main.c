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

char rx_buffer[size_of_array]={0};		// Reception Buffer 
char data[size_of_array]={0};		// Real comunication
uint8_t rx_index=0;
uint8_t pointer_to_data=0; // Pointer to the array of data


uint8_t speed;
uint16_t distance;
uint16_t angle;
uint8_t flag=1; // variable to continue reading code
float gaini=1; // variable to calibrate (increase)
float gaind=1; // Variable to calibrate (decrease)

//__________________________________________IRQ______________________________________________
void TIMER1_IRQHandler(){	//Motor (1) Fastest, right side if you see the front of the car, PWM1.2 P1.20 / CAP1.0 P1.18 / Every 2 edges to calibrate
	if(LPC_TIM1->IR&((0x1<<0))){		//Interrupt MR0
		LPC_TIM1->IR=(0x1<<0);		//Clear flag of MR0 interrupt
			
		if(!(0 <= ((LPC_TIM1->TC)-(LPC_TIM2->TC)) && ((LPC_TIM1->TC)-(LPC_TIM2->TC)) <= 2)){ // Motor 1 has moved 2 steps while Motor 2 has not moved
			if((LPC_TIM1->TC) > (LPC_TIM2->TC) ){ // Motor 1 faster than Motor 2
				if((LPC_PWM1->MR4*gaini*1.0001)<LPC_PWM1->MR0){ // Check that new tH of Motor 2 is less than period of PWM
					gaini=gaini*1.0001; // Increase gain of Motor 2 -> Increase tH
					gaind=gaind*1.0001; // Increase gain of decrease
					calib(gaini,speed); // Calibration function
				}
				if((LPC_PWM1->MR4)>LPC_PWM1->MR0){ // Security function in case new tH is greater than period of PWM
					gaind=gaind*0.9999; // Reduce gain of Motor 2 -> Decrease tH
					gaini=gaini*0.9999; // Reduce gain of increase
					calib(gaind,speed); // Calibration function
				}
				
			}
		}
		
		/* Comprueba esto........................*/
		if(!(0 <= ((LPC_TIM2->TC)-(LPC_TIM1->TC)) && ((LPC_TIM2->TC)-(LPC_TIM1->TC)) <= 2)){ // Motor 2 has moved 2 steps while Motor 1 has not moved
			if((LPC_TIM2->TC) > (LPC_TIM1->TC) ){ // Motor 2 faster than Motor 1
				if((LPC_PWM1->MR4)<LPC_PWM1->MR0){ // Check that tH of Motor 2 is less than period of PWM
					gaind=gaind*0.9999; // Decrease gain of Motor 2 -> Decrease tH
					gaini=gaini*0.9999; // Reduce gain of increase
					calib(gaind,speed); // Calibration function
				}
				if((LPC_PWM1->MR4)>LPC_PWM1->MR0){ // Security function in case tH is greater than period of PWM
					gaind=gaind*0.999; // Reduce a little bit more, so that tH<T, the gain of Motor 2 -> Decrease tH
					gaini=gaini*0.999; // Reduce gain of increase
					calib(gaind,speed); // Calibration function
				}
				
			}
		}
		
		
		
		LPC_TIM1->MR0 += 2; // Set MR0 according 1/11 revolution to calibrate the speed	
	}
}
void TIMER2_IRQHandler(){	//Motor (2) Slowest, left side if you see the front of the car, PWM1.4 P1.23 / CAP2.0 P0.4 / When distance is completed
	if(LPC_TIM2->IR&((0x1<<0))){		//Interrupt MR0
		LPC_TIM2->IR=(0x1<<0);		//Clear flag of MR0
		
		LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16))); // Stop
		flag=1; // Next instruction

		pointer_to_data+=3; // Move the pointer to next instruction
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
		if(flag==1){
			switch(data[pointer_to_data]){
			
			case 'V':		//Define speed
				speed=(((uint8_t)(data[pointer_to_data+1]-'0'))*10+(uint8_t)(data[pointer_to_data+2]-'0'));
				pointer_to_data=pointer_to_data+Fc_speed_control(speed);			
			break;
			
			case 'D':		//Define right movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x1<<16);
				angle=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(8*((angle)*(3.141592654/180)));
				flag=0;
			break;
			
			case 'I':		//Define left movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x2<<16);
				angle=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(8*((angle)*(3.141592654/180)));
				flag=0;
			break;
			
			case 'A':		//Define forward movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x1)|(0x1<<16);	
				distance=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(distance);
				flag=0;
			break;
			
			case 'R':		//Define backwards movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x2<<16);
				distance=(uint16_t)((((uint16_t)(data[pointer_to_data+1]-'0'))*10+(uint16_t)(data[pointer_to_data+2]-'0')));
				set_distance(distance);
				flag=0;
			break;
			
			default:
					LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16))); // Stop
			break;
			
			}
		}	
	}
}

