#include "stm32l476xx.h"
#include <stdio.h>


void USART_Read (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes); 
void USART_Init(USART_TypeDef * USARTx);
void USART_Write (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes);
void EXTI3_IRQHandler(void);
void EXTI_INIT(void);
void keyPad(void);
void gpioEnable(void);

void gpioEnable(void)
{
	//SET UP GPIOC AND PC3
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	//SET AS INPUT
	GPIOC->MODER &= 0xFFFFFF3F;
	
	//SET AS PULL UP
	GPIOC->PUPDR |= 0x00000040;
}

void keyPad(void){
	volatile unsigned int butts;
	GPIOB->MODER &= 0xFFFF00FF;						//SETS GPIOB 4-7 input
	GPIOB->MODER &= 0xFF55FFFF;						//SET GPIOB 8-11 AS OUTPUT
	while(1)
	{
		GPIOB->ODR &= 0x00000000;						//RESET ODR
		GPIOB->ODR |= 0x0000700;						//SETS ROW 1 TO 0
		butts = GPIOB->IDR;									//HOLDS IDR VALUE TO BE TESTED
		switch(butts)												//SWITCH TO CASCADE THROUGH EACH ROW TO CHECK FOR PUSH
		{
			case 0x770:
				//LCD_WriteData('1');
				while(GPIOB->IDR == 0x771);
				break;
			
			case 0x7B0:
				//LCD_WriteData('2');
				
				while(GPIOB->IDR == 0x7B2);
				break;
			
			case 0x7D0:
				//LCD_WriteData('3');
				
				while(GPIOB->IDR == 0x7D3);
				break;
			
			case 0x7E0:
				//LCD_WriteData('A');
				
				while(GPIOB->IDR == 0x7E1);
				break;
		}
		
		GPIOB->ODR &= 0x00000000;								//RESET ODR
		GPIOB->ODR |= 0x00000B00;								//SETS ROW 2 TO 0
		butts = GPIOB->IDR;
		switch(butts)
		{
			case 0xB70:
				//LCD_WriteData('4');
				
				while(GPIOB->IDR == 0xB74);
				break;
			
			case 0xBB0:
				//LCD_WriteData('5');
				
				while(GPIOB->IDR == 0xBB5);
				break;
			
			case 0xBD0:
				//LCD_WriteData('6');
				
				while(GPIOB->IDR == 0xBD6);
				break;
			
			case 0xBE0:
				//LCD_WriteData('B');
				
				while(GPIOB->IDR == 0xBE2);
				break;
		}
		
		GPIOB->ODR &= 0x00000000;								//RESET ODR
		GPIOB->ODR |= 0x00000D00;								//SETS ROW 3 TO 0
		butts = GPIOB->IDR;
		switch(butts)
		{
			case 0xD70:
				//LCD_WriteData('7');
				
				while(GPIOB->IDR == 0xD77);
				break;
			
			case 0xDB0:
				//LCD_WriteData('8');
				
				while(GPIOB->IDR == 0xDB8);
				break;
			
			case 0xDD0:
				//LCD_WriteData('9');
				
				while(GPIOB->IDR == 0xDD9);
				break;
			
			case 0xDE0:
				//LCD_WriteData('C');
				
				while(GPIOB->IDR == 0xDE3);
				break;
		}
		
		GPIOB->ODR &= 0x00000000;								//RESET ODR
		GPIOB->ODR |= 0x00000E00;								//SETS ROW 4 TO 0
		butts = GPIOB->IDR;
		switch(butts)
		{
			case 0xE70:
				//LCD_WriteData('*');
				
				while(GPIOB->IDR == 0xE7A);
				break;
			
			case 0xEB0:
				//LCD_WriteData('0');
				
				while(GPIOB->IDR == 0xEB0);
				break;
			
			case 0xED0:
				//LCD_WriteData('#');
				
				while(GPIOB->IDR == 0xED3);
				break;
			
			case 0xEE0:
				//LCD_WriteData('D');
				
				while(GPIOB->IDR == 0xEE4);
				break;
		}
	}
}

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

static unsigned char state = 1;
static unsigned char stopGo[4] = {'a','a','a','a'};
static unsigned char current = 0;

void EXTI3_IRQHandler(void)
{
	if ((EXTI->PR1 & EXTI_PR1_PIF3) == EXTI_PR1_PIF3)
	{
		// TURN ON DUST COLLECTOR
		stopGo[0] = 'g';
		stopGo[1] = 't';
		stopGo[2] = 'o';
		stopGo[3] = 'p';
		USART_Write(USART3, stopGo, 4);
		
		// CHECK FOR DEVICE TO BE OFF
		while ((GPIOB->IDR & 0xF) == 0x0)
			;
		stopGo[0] = 's';
		stopGo[1] = 't';
		stopGo[2] = 'o';
		stopGo[3] = 'p';
		USART_Write(USART3, stopGo, 4);
		EXTI->PR1 |= EXTI_PR1_PIF3;
	}
}

void EXTI_INIT(void)
{
	// ENABLE
	NVIC_EnableIRQ(EXTI3_IRQn);

	// ENABLE SYSCFG CLOCK
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// SELECT PA3
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI3);
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;

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
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	//enable port A
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;	//enable port B
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;	//enable port C
	
	GPIOB->OTYPER |= 0x00000F00;

	// SET AS INPUT
	GPIOB->MODER &= 0xFFFFFF3F;

	// SET AS PULL UP
	GPIOB->PUPDR |= 0x00000040;
	
	
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);		//set clock
	
	GPIOC->MODER &= 0xFF0FFFFF;	//set PB10 and PB11 as alternate function
	GPIOC->MODER |= 0x00A00000;
	
	GPIOC->AFR[1] |= 0x77 << 8; //alternate function 7
	
	GPIOC->OSPEEDR |= 0x00F00000;		//high speed
	
	GPIOC->PUPDR &= 0xFF0FFFFF;			//pull-up
	GPIOC->PUPDR &= 0x00A00000;
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;	//enable clock for USART3
	
	RCC->CCIPR &= ~(RCC_CCIPR_USART3SEL);	//select sysclock as usart clock source for uart3
	RCC->CCIPR |= ~(RCC_CCIPR_USART3SEL_0);
	
	EXTI_INIT();

	USART_Init(USART3);
	
	//SET UP GPIOC AND PC3
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	GPIOC->MODER &= 0xFFFFFF3F;
	GPIOC->PUPDR |= 0x00000040;
	
	//gpioEnable();
	//EXTI_INIT();
	//keyPad();
	///below this will never execute, so put into sysTick as state machine
	while(1){
		/*if(current == 0){
			if(state == 1){
				stopGo[0] = 's';
			  stopGo[1] = 't';
				stopGo[2] = 'o';
				stopGo[3] = 'p';
				USART_Write(USART3, stopGo, 4);
			}
		}
		else if(current == 1){
			if (state == 0){
				stopGo[0] = 'g';
			  stopGo[1] = 'o';
				stopGo[2] = '\n';
				stopGo[3] = '\n';
				USART_Write(USART3, stopGo, 4);
			}
		}*/
	}
	
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
		//USARTx->BRR = 0x01A1;		//set baud rate to 38400
		USARTx->BRR = 0x0683;		//set baud rate to 9600
	}
	
	USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);		//enable transmission and reception
	
	USARTx->CR1 |= USART_CR1_UE;	//enalbe UART
	
	while((USARTx->ISR & USART_ISR_TEACK) == 0);
	
	while((USARTx->ISR & USART_ISR_REACK) == 0);
	
}






