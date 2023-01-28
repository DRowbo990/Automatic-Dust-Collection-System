#include "stm32l476xx.h"
#include <stdio.h>


void USART_Read (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes); 
void USART_Init(USART_TypeDef * USARTx);
void USART_Write (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes);
void EXTI3_IRQHandler(void);
void EXTI_INIT(void);
void stopWrite(void);
void goWrite(void);
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

static unsigned char state = 0;
static unsigned char stopGo[4] = {'a','a','a','a'};
//static unsigned char current = 0;

void stopWrite(void) // TURNS OFF DUST COLLECOR.
{
	stopGo[0] = 's';
	stopGo[1] = 't';
	stopGo[2] = 'o';
	stopGo[3] = 'p';
	USART_Write(USART3, stopGo, 4);
	//time = 0;
}

void goWrite(void) // TURNS ON DUST COLLECTOR.
{
	stopGo[0] = 'g';
	stopGo[1] = 'o';
	stopGo[2] = 'o';
	stopGo[3] = 'o';
	USART_Write(USART3, stopGo, 4);
}

void EXTI3_IRQHandler(void)
{
	if((EXTI->PR1 & EXTI_PR1_PIF3) == EXTI_PR1_PIF3)
	{	
		
		goWrite();
		while((GPIOC->IDR & 0xF) == 0x0);
		stopWrite();
		EXTI->PR1 |= EXTI_PR1_PIF3;
		
	}
}

void EXTI_INIT(void)
{
	NVIC_EnableIRQ(EXTI3_IRQn);

	// ENABLE SYSCFG CLOCK
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// SELECT PA3
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI3);
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;

	// ENABLE EXTI 3
	EXTI->IMR1 |= EXTI_IMR1_IM3;

	// FALLING TRIGGER EDGE
	EXTI->FTSR1 |= EXTI_FTSR1_FT3;
}

void USART_Read (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes){ 
	int i; 				//define variables
	for (i = 0; i < nBytes; i++) { 	//Loop
		while (!(USARTx->ISR & USART_ISR_RXNE)); // Wait until hardware sets RXNE 
		buffer[i] = USARTx->RDR; // Reading RDR clears RXNE 
	}	
	if(buffer[0] == 's'){
		state = 0;
		//make it stop
	}
	else if(buffer[0] == 'g'){
		state = 1;
		//make it start
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
		//USARTx->BRR = 0x01A1;		//set baud rate to 38400
		USARTx->BRR = 0x0683;		//set baud rate to 9600
	}
	
	USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);		//enable transmission and reception
	
	USARTx->CR1 |= USART_CR1_UE;	//enalbe UART
	
	while((USARTx->ISR & USART_ISR_TEACK) == 0);
	
	while((USARTx->ISR & USART_ISR_REACK) == 0);
	
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
	
	//SET UP GPIOC AND PC3
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	GPIOC->MODER &= 0xFFFFFF3F;
	GPIOC->PUPDR |= 0x00000040;
	
	EXTI_INIT();	//initialize EXTI
	
	while(1); //never end loop
	
}	







