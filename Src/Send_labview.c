#include <LPC17xx.h>

void Send_labview(char character,uint8_t flagmessageUART0,uint8_t flagmessageUART3){
	if(flagmessageUART0==1){
		while(((LPC_UART0->LSR&(0x1<<5))>>5)==0);	//	WAITING UNTIL THE BUFFER IS EMPTY
		LPC_UART0->THR=character;
	}
	if(flagmessageUART3==1){
		while(((LPC_UART3->LSR&(0x1<<5))>>5)==0);	//	WAITING UNTIL THE BUFFER IS EMPTY 
		LPC_UART3->THR=character;	//	INDICATE LABVIEW MODE WAITING AND SILENCE
	}
}
