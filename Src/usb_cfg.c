#include <LPC17xx.h>

void usb_cfg(void){
	
	LPC_SC->PCONP|=(0x1<<25);
	LPC_SC->PCLKSEL1=(LPC_SC->PCLKSEL1&~(0x3<<18))|(0x1<<18);		//Configurate clock of pheriperal uart0 to =CCLK=100Mhz
	LPC_UART0->LCR=(LPC_UART0->LCR&~(0x7F))|(0x3)|(0x1<<7);		//Word lenght 8=char,stop bit & parity & break control to 0, enable divisor lach
	LPC_UART0->DLM=2;		//Expresion used to take the baud rate in page n323 with reset value of DIVadd=0/Mulvaal=1
	LPC_UART0->DLL=139;
	LPC_UART0->LCR&=~(0x1<<7);		// Disable divisor lach
	LPC_UART0->FCR=(LPC_UART0->FCR&~(0xFF))|(0x3);		// Enable FIFO and reset FIFO RX, RX trigger level 0 Interrupt for Interrupt for each char sent
	
	LPC_UART0->IER|=(0x1);		//Enable interruption
	
	
}
