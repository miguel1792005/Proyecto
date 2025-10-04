#include <LPC17xx.h>

void Fc_bluetooth_communication(void){
	LPC_SC->PCONP|=(0x1<<25);
	LPC_SC->PCLKSEL1=(LPC_SC->PCLKSEL1&~(0x3<<18))|(0x1<<18);		//Configurate clock of pheriperal uart3 to =CCLK=100Mhz
	LPC_UART3->LCR=(LPC_UART3->LCR&~(0x7F))|(0x3)|(0x1<<7);		//Word lenght 8=char,stop bit & parity & break control to 0, enable divisor lach
	LPC_UART3->DLM=2;		//Expresion used to take the baud rate in page n323 with reset value of DIVadd=0/Mulvaal=1
	LPC_UART3->DLL=139;
	LPC_UART3->LCR&=~(0x1<<7);		//Deshabilitamos divisor lach
	LPC_UART3->FCR=(LPC_UART3->FCR&~(0xFF))|(0x3);		//(REVISAR TRIGGER LVL) Enable FIFO and reset FIFO RX, RX trigger level 0 Interrupt for Interrupt for each char sent
	
	LPC_UART3->IER|=(0x1);		//Enable interruption
	NVIC_EnableIRQ(UART3_IRQn);
}