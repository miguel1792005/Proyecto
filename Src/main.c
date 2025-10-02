#include <LPC17xx.h>
#include "Fc_config_pines.h"
#include "Fc_config_PWM.h"
#include "Fc_control_velocidad.h"
#include "Fc_config_TIMER.h"
#include "Fc_config_IRQ.h"
uint8_t A=0;

//cambio para comprobar
uint16_t C=4;

char Frase[6]={'V','4','4','R','0','4'};
uint8_t puntero_frase=0;
uint32_t pasos_motor1x2=0;
uint32_t pasos_motor2x2=0;

void TIMER1_IRQHandler(){
	if(LPC_TIM1->IR&((0x1<<4))){		//Interrupt?
		LPC_TIM1->IR=(0x1<<4);		//Clear flag
		pasos_motor1x2=LPC_TIM1->TC;
	}
}
void TIMER2_IRQHandler(){
	if(LPC_TIM2->IR&((0x1<<4))){		//Interrupt?
		LPC_TIM2->IR=(0x1<<4);		//Clear flag
		pasos_motor2x2=LPC_TIM2->TC;
	}
}
int main(){
	Fc_config_pines();
	Fc_config_IRQ();
	Fc_config_TIMER();
	Fc_config_PWM();
	while(1){
		switch(Frase[puntero_frase]){
			case 'V':		//Define speed
				puntero_frase=puntero_frase+Fc_control_velocidad((((uint8_t)(Frase[puntero_frase+1]-'0'))*10+(uint8_t)(Frase[puntero_frase+2]-'0')));
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
			case 'R':		//Define recoil movement
				LPC_GPIO1->FIOPIN=(LPC_GPIO1->FIOPIN&~((0x3)|(0x3<<16)))|(0x2)|(0x2<<16);	
			break;		
		}
	}
}