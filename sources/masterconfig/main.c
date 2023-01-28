#include "stm32l476xx.h"
#include <stdio.h>

//https://www.youtube.com/watch?v=hyME1osgr7s&t=292s

void USART_Read (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes); 
void USART_Init(USART_TypeDef * USARTx);
void USART_Write (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes);

/*
begin
	define variables
	Loop
		read each byte and store in buffer
	End loop when buffer is full
	if t or T is read
		calculate temp
		write the temp
end
*/

void USART_Read (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes){ 
	int i; 				//define variables
	for (i = 0; i < nBytes; i++) { 	//Loop
		while (!(USARTx->ISR & USART_ISR_RXNE)); // Wait until hardware sets RXNE 
		buffer[i] = USARTx->RDR; // Reading RDR clears RXNE 
	}		
	return;
} 

/*
begin
	define variable
	Loop
		write to buffer
	End loop finshed
	wait for complete
end
*/
void USART_Write (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes) { 
	uint32_t i; 		//define variable
	for (i = 0; i < nBytes; i++) {	//Loop
		while (!(USARTx->ISR & USART_ISR_TXE)); // Wait until hardware sets TXE 
		USARTx->TDR = buffer[i] & 0xFF; // Writing to TOR clears TXE flag 
	} 		//End loop when finished
	// Wait until TC bit is set. TC is set by hardware and cleared by software. 
	while (!(USARTx->ISR & USART_ISR_TC)); // TC: Transmission complete flag 
	// Writing 1 to the TCCF bit in ICR clears the TC bit in ISR 
	USARTx->ICR |= USART_ICR_TCCF; // TCCF: Transmission complete clear flag 
}

/*
begin
	enable ports A and C
	set clock
	initialize LCD
	clear LCD
	set pins as pull up inputs
	set up SysTick interrupt
	set up EXTI interrupts
	
end
*/
int main(void){							//begin
	unsigned char write2[14] = {'A', 'T', '+', 'P', 'S', 'W', 'D', '=', '1','2','3','4', 13, '\n'};
	unsigned char write[11] = {'A', 'T', '+', 'R', 'O', 'L', 'E', '=', '1', 13, '\n'};
	unsigned char write1[12] = {'A', 'T', '+', 'C', 'M', 'O', 'D', 'E', '=', '0', 13, '\n'};
	unsigned char write3[24] = {'A', 'T', '+', 'P', 'A', 'I', 'R', '=', '9', '8', 'd', '3', ',','3','1',',','f','6','6','6', '0','f', 13, '\n'};
	unsigned char write4[24] = {'A', 'T', '+', 'B', 'I', 'N', 'D', '=', '9', '8', 'd', '3', ',','3','1',',','f','6','6','6', '0','f', 13, '\n'};
	unsigned char read[26] = {126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,13,'\n'};
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	//enable port A
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;	//enable port B
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;	//enable port C
	
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);		//set clock
	
	GPIOB->MODER &= 0xFFFFFFFD; //set PB0 as output
	GPIOB->MODER |= 0x00000001;
	
	GPIOB->ODR |= 0x1;		//set PB0 high
	
	
	GPIOC->MODER &= 0xFF0FFFFF;	//set PB10 and PB11 as alternate function
	GPIOC->MODER |= 0x00A00000;
	
	GPIOC->AFR[1] |= 0x77 << 8; //alternate function 7
	
	GPIOC->OSPEEDR |= 0x00F00000;		//high speed
	
	GPIOC->PUPDR &= 0xFF0FFFFF;			//pull-up
	GPIOC->PUPDR &= 0x00A00000;
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;	//enable clock for USART3
	
	RCC->CCIPR &= ~(RCC_CCIPR_USART3SEL);	//select sysclock as usart clock source for uart3
	RCC->CCIPR |= ~(RCC_CCIPR_USART3SEL_0);
	
	
	USART_Init(USART3);
	
	USART_Write(USART3, write, 11);		//write AT+ROLE=1
	USART_Read(USART3, read, 4);
	read[0] = '~';
	read[1] = '~';
	read[2] = '~';
	read[3] = '~';
	
	//USART_Write(USART3, write2, 14);	//AT+PSWD=1234
	//USART_Read(USART3, read, 4);
	
	USART_Write(USART3, write1, 12);	//write AT+CMODE=0
	USART_Read(USART3, read, 4);
	read[0] = '~';
	read[1] = '~';
	read[2] = '~';
	read[3] = '~';
	
	USART_Write(USART3, write4, 24);	//write AT+BIND=(ADDR)
	USART_Read(USART3, read, 4);
	read[0] = '~';
	read[1] = '~';
	read[2] = '~';
	read[3] = '~';
	
	//USART_Write(USART3, write0, 4);
	//USART_Read(USART3, read, 4);
	//USART_Write(USART3, write0, 4);
	//USART_Read(USART3, read, 4);
	
	
	GPIOA->MODER &= 0xFFFFFF0F;	//set PA2 and PA3 as alternate function
	GPIOA->MODER |= 0xA0;
	
		//alternate function 7
	GPIOA->AFR[0] |= 0x77 << 8;
	
	GPIOA->OSPEEDR |= 0x000000F0;		//high speed
	
	GPIOA->PUPDR &= 0xFFFFFF0F;			//pull-up
	GPIOA->PUPDR &= 0x000000A0;
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;	//enable clock for USART2
	
	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);	//select sysclock as usart clock source for uart2
	RCC->CCIPR |= ~(RCC_CCIPR_USART2SEL_0);
	
	
	USART_Init(USART2);
	//read = {0};
	
	USART_Write(USART2, read, 26);	//write original contents from UART
	
	while(1){}
	
}		//never end loop


void USART_Init(USART_TypeDef * USARTx){
	USARTx->CR1 &= ~USART_CR1_UE;	//disable UART
	
	USARTx->CR1 &= ~USART_CR1_M;	//set data length to 8 bits
	
	USARTx->CR2 &= ~USART_CR2_STOP;	//1 stop bit
	
	USARTx->CR1 &= ~USART_CR1_PCE;	//No parity bit
	
	USARTx->CR1 &= ~USART_CR1_OVER8;	//oversampling by 16
	
	
	if(USARTx == USART2){
		//USARTx->BRR = 0x01A1;		//set baud rate to 38400
		USARTx->BRR = 0x0683;		//set baud rate to 9600
	}
	else if(USARTx == USART3){
		USARTx->BRR = 0x01A1;		//set baud rate to 38400
		//USARTx->BRR = 0x0683;		//set baud rate to 9600
	}
	
	USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);		//enable transmission and reception
	
	USARTx->CR1 |= USART_CR1_UE;	//enalbe UART
	
	while((USARTx->ISR & USART_ISR_TEACK) == 0);
	
	while((USARTx->ISR & USART_ISR_REACK) == 0);
	
}






