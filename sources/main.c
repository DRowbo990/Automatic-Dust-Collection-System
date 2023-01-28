#include "stm32l476xx.h"

short int state = 0;
short int time = 0;
static unsigned char stopGo[4] = {'a','a','a','a'};

void USART_Read (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes)
{
	while(1)
	{
	int i; 				//define variables
	for (i = 0; i < nBytes; i++) { 	//Loop
		while (!(USARTx->ISR & USART_ISR_RXNE)); // Wait until hardware sets RXNE 
		buffer[i] = USARTx->RDR; // Reading RDR clears RXNE 
	}
	if(buffer[0] == 's'){
		GPIOB->ODR &= 0xFFFFFFFE;
	}
	else if(buffer[0]== 'g') {
		GPIOB->ODR |= 0x1;
	}
  state = 0;	
}
} 

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

void stopWrite(void) // TURNS OFF DUST COLLECOR.
{
	stopGo[0] = 's';
	stopGo[1] = 't';
	stopGo[2] = 'o';
	stopGo[3] = 'p';
	USART_Write(USART3, stopGo, 4);
	time = 0;
}

void goWrite(void) // TURNS ON DUST COLLECTOR.
{
	stopGo[0] = 'g';
	stopGo[1] = 'o';
	stopGo[2] = 'o';
	stopGo[3] = 'o';
	USART_Write(USART3, stopGo, 4);
}

void countDown(long int downCount)
{
	short int butts;
	GPIOB->ODR &= 0x00000000; // RESET ODR
	GPIOB->ODR |= 0x00000E00; // SETS ROW 4 TO 0
	
	downCount *= 160000;//for minutes, times by 60
	GPIOB->ODR |= 0x1;
	while(downCount)
	{
		downCount--;
		butts = (GPIOB->IDR & 0xF0);
		if(butts == 0x70)
		{
			break;
		}
	}
	GPIOB->ODR &= 0x0;
	time = 0;
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

void USART_enable(void)
{
	GPIOC->MODER &= 0xFF0FFFFF;	//set PC10 and PC11 as alternate function
	GPIOC->MODER |= 0x00A00000;
	
	GPIOC->AFR[1] |= 0x77 << 8; //alternate function 7
	
	GPIOC->OSPEEDR |= 0x00F00000;		//high speed
	
	GPIOC->PUPDR &= 0xFF0FFFFF;			//pull-up
	GPIOC->PUPDR &= 0x00A00000;
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;	//enable clock for USART3
	
	RCC->CCIPR &= ~(RCC_CCIPR_USART3SEL);	//select sysclock as usart clock source for uart3
	RCC->CCIPR |= ~(RCC_CCIPR_USART3SEL_0);
	
	USART_Init(USART3);
}



void timer(short int temp) // SETS TIMER FOR DUST COLLECTOR. UP TO 99 MIN.
{
	// CHECKS IF NO TIME HAS BEEN ENTERED YET
	if (time == 0)
	{
		time += temp;
	}
	// IF TIME ENTERED, CHECKS IF MORE TIME CAN BE ADDED.
	else
	{
		// IF MORE TIME CAN BE ENTERED, ADDS TIME.
		if (time < 10)
		{
			time *= 10;
			time += temp;
		}
	}
}

void keyPad()
{
	USART3->CR1 &= ~USART_CR1_UE;	//disable UART
	volatile unsigned int butts;
	GPIOB->MODER &= 0xFFFF00FF; // SETS GPIOB 4-7 input
	GPIOB->MODER &= 0xFF55FFFF; // SET GPIOB 8-11 AS OUTPUT
	while (1)
	{
		GPIOB->ODR &= 0x00000000;	 // RESET ODR
		GPIOB->ODR |= 0x0000700;	 // SETS ROW 1 TO 0
		butts = (GPIOB->IDR & 0xF0); // HOLDS IDR VALUE TO BE TESTED
		switch (butts)				 // SWITCH TO CASCADE THROUGH EACH ROW TO CHECK FOR PUSH
		{
		case 0x70:
			timer(1);
			while ((GPIOB->IDR & 0xF0) == 0x70)
				;
			break;

		case 0xB0:
			timer(2);
			while ((GPIOB->IDR & 0xF0) == 0xB0)
				;
			break;

		case 0xD0:
			timer(3);
			while ((GPIOB->IDR & 0xF0) == 0xD0)
				;
			break;

			/*case 0xE0:
				timer(a);
				while((GPIOB->IDR & 0xF0) == 0xE0);
				break;*/
		}

		GPIOB->ODR &= 0x00000000; // RESET ODR
		GPIOB->ODR |= 0x00000B00; // SETS ROW 2 TO 0
		butts = (GPIOB->IDR & 0xF0);
		switch (butts)
		{
		case 0x70:
			timer(4);

			while ((GPIOB->IDR & 0xF0) == 0x70)
				;
			break;

		case 0xB0:
			timer(5);

			while ((GPIOB->IDR & 0xF0) == 0xB0)
				;
			break;

		case 0xD0:
			timer(6);

			while ((GPIOB->IDR & 0xF0) == 0xD0)
				;
			break;

			/*case 0xE0:
				//LCD_WriteData('B');

				while((GPIOB->IDR & 0xF0) == 0xE0);
				break;*/
		}

		GPIOB->ODR &= 0x00000000; // RESET ODR
		GPIOB->ODR |= 0x00000D00; // SETS ROW 3 TO 0
		butts = (GPIOB->IDR & 0xF0);
		switch (butts)
		{
		case 0x70:
			timer(7);

			while ((GPIOB->IDR & 0xF0) == 0x70)
				;
			break;

		case 0xB0:
			timer(8);

			while ((GPIOB->IDR & 0xF0) == 0xB0)
				;
			break;

		case 0xD0:
			timer(9);

			while ((GPIOB->IDR & 0xF0) == 0xD0)
				;
			break;

			/*case 0xE0:
				//LCD_WriteData('C');

				while((GPIOB->IDR & 0xF0) == 0xE0);
				break;*/
		}

		GPIOB->ODR &= 0x00000000; // RESET ODR
		GPIOB->ODR |= 0x00000E00; // SETS ROW 4 TO 0
		butts = (GPIOB->IDR & 0xF0);
		switch (butts)
		{
		// FOR BYRANT. MAKE * KILL
		case 0x70:
			return;

			while ((GPIOB->IDR & 0xF0) == 0x70)
				;
			break;

		case 0xB0:
			timer(0);

			while ((GPIOB->IDR & 0xF0) == 0xB0)
				;
			break;

		// FOR BYRANT. MAKE # START TIMER
		case 0xD0:
			countDown(time);

			while ((GPIOB->IDR & 0xF0) == 0xD0)
				;
			USART_Init(USART3); 
			return;

			/*case 0xE0:
				//LCD_WriteData('D');

				while((GPIOB->IDR & 0xF0) == 0xE0);
				break;*/
		}
	}
}

void gpioEnable(void)
{
	// SET UP GPIOC AND B
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOBEN;

	// SET GPIOB 8-11 AS OPEN DRAIN
	GPIOB->OTYPER |= 0x00000F00;

	// SET AS INPUT
	GPIOB->MODER &= 0xFFFFFF3D;
	GPIOB->MODER |= 0x00000001;

	// SET AS PULL UP
	GPIOB->PUPDR |= 0x00000040;
	
	
}

void EXTI3_IRQHandler(void)
{
	if ((EXTI->PR1 & EXTI_PR1_PIF3) == EXTI_PR1_PIF3)
	{
		// TURN ON DUST COLLECTOR
		keyPad();

		// CHECK FOR DEVICE TO BE OFF
		//while ((GPIOB->IDR & 0xF) == 0x0)
			;
		
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

int main(void)
{
	// SET CLOCK TO 1600MHZ
	RCC->CR |= RCC_CR_HSION;
	while ((RCC->CR & RCC_CR_HSIRDY) == 0);
	
	gpioEnable();

	USART_enable();
	EXTI_INIT();
	USART_Read(USART3, stopGo, 4);
}