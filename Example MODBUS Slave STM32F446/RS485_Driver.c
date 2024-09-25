#include "RS485_Driver.h"
#include "stm32f4xx.h"                  // Device header

/***************************************************************************************
Author: Sadat Rafi
File: UART1_Driver.c
Description: This file contains the source code for UART communication over an RS485 
network using the MAX485 transceiver and STM32F446RE Nucleo board. The txBufferUart1 peripheral 
on the STM32F446RE is configured to handle the half-duplex communication required for 
MODBUS RTU protocol. 

The MAX485 IC is used to interface with the RS485 bus, allowing both transmission and 
reception of data. A dedicated GPIO pin is used to control the MAX485's transmit/receive 
direction by switching between driver mode (transmitting) and receiver mode.

This driver captures the MODBUS RTU frame transmitted from the master, passes it to the 
MODBUS slave library, and sends the resulting reply frame back to the RS485 bus via UART.

User instructions:
- Configure txBufferUart1 for half-duplex mode.
- Control the MAX485 IC's DE/RE pin using a GPIO to switch between transmission and 
  reception modes.
- Call this driver to capture MODBUS RTU frames and send replies to the RS485 bus.
****************************************************************************************/

/***************************************************************************************
Author: Sadat Rafi
Function: initialize_UART1
Description: 
	This function initializes UART1 on the STM32F446RE microcontroller with the specified
	baud rate. It enables the necessary clocks for USART1 and GPIOA, configures the TX (PA9)
	and RX (PA10) pins for alternate function, sets the baud rate based on the system clock,
	enables the transmitter, receiver, and RX interrupt, and finally configures the NVIC to
	handle USART1 interrupts.
****************************************************************************************/
void initialize_RS485(int baudRate)
{
	// 1. Enable the clock for USART1 and GPIOA (USART1 TX is PA9, RX is PA10)
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;    // Enable USART1 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;     // Enable GPIOA clock

	// 2. Configure PA9 (TX) and PA10 (RX) for alternate function (AF7)
	GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);  // Clear PA9 and PA10 mode bits
	GPIOA->MODER |= (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);  // Set alternate function mode for PA9 and PA10

	GPIOA->AFR[1] |= 0x00000770;  // Set AF7 for PA9 (TX) and PA10 (RX)

	// 3. Configure USART1
	USART1->CR1 = 0;   // Clear all settings
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;   // Enable transmitter and receiver

	// 4. Configure baud rate (APB2 Clock is 90MHz for STM32F446RE)
	SystemCoreClockUpdate();
	USART1->BRR = (SystemCoreClock / 2) / baudRate;   // APB2 runs at half the system clock, calculate baud rate

	// 5. Enable RX interrupt (Receive Data Register Not Empty)
	USART1->CR1 |= USART_CR1_RXNEIE;   // Enable RXNE interrupt

	// 6. Enable USART1
	USART1->CR1 |= USART_CR1_UE;   // Enable USART

	// 7. Configure NVIC for USART1 IRQ
	NVIC_EnableIRQ(USART1_IRQn);   // Enable USART1 interrupt in NVIC
	
	// 8. Configure PA8 pin as output for MAX485 DE/RE control
	GPIOA->MODER &= ~(GPIO_MODER_MODE8);  // Clear PA8 mode bits
	GPIOA->MODER |= (GPIO_MODER_MODE8_0);  // Set PA8 as output
	GPIOA->OSPEEDR |= (3U << (2 * 8));  // Set high speed for PA8

	// 9. Initialize PA8 to receiver mode (pull DE/RE pin low to receive)
	GPIOA->ODR &=~ GPIO_ODR_OD8;  // Reset PA8 (receiver mode)
}

typedef struct
{
	uint8_t data[MXIMUM_TX_BUFFER_SIZE];
	uint8_t enqueueIndex;
	uint8_t dequeueIndex;
	uint16_t length;
}TxBuffer;

static TxBuffer txBufferUart1;

/***************************************************************************************
Author: Sadat Rafi
Date: 25 September 2024
Function: Transfer_Message
Description: 
	This function first enqueues the payload in a citcular buffer if the enqueue and dequeue
index are same (i.e. the circular buffer is currently empty). If the enqueue and dequeue
index are not same, it indicates that a UART transmission is in progress. After enqueueing,
PA8 is set which turns MAX485 into bus controller mode. Then it enables UART1 transmit 
register empty interrupt.
****************************************************************************************/
int Transfer_Message(uint8_t *payload, uint16_t length)
{
if(length > (uint16_t)MXIMUM_TX_BUFFER_SIZE)
{
	return ERROR_INCREASE_CIRCULAR_BUFFER_SIZE;
}
	
/***************************************************************************************
NOTE: In modbus zero may come in srting as a part of data. Modbus RTU string is not ASCII
character.So specifying a end indicating character or zro as end of frame is not practical.
****************************************************************************************/
	for(int tempIndex = 0; tempIndex <= length-1;tempIndex++)
	{
		if(txBufferUart1.enqueueIndex == txBufferUart1.dequeueIndex)
		{
			txBufferUart1.data[txBufferUart1.enqueueIndex] = payload[tempIndex];
			txBufferUart1.enqueueIndex++;
			if(txBufferUart1.enqueueIndex >= (uint8_t)MXIMUM_TX_BUFFER_SIZE) 
			{
				txBufferUart1.enqueueIndex = 0;
			}
		}
		else 
			return ERROR_UART1_TRANSMISSION_ONGOING;
		
		GPIOA->ODR |= GPIO_ODR_OD8;  // Set PA8 (Transmitter mode)
		//Enable USART Transmit Register Empty Interrupt
		USART1->CR1 |= USART_CR1_TXEIE;
	}
	return SUCCESS;
}

/***************************************************************************************
Author: Sadat Rafi
Date: 25 September 2024
Function: USART1_IRQHandler
Description: 
	USART1 Interrupt Handler manages both transmission and receiption. If Transmit Register
Empty Interrupt occurs, it dequeues 'txBufferUart1' circular buffer until the enqueue and
dequeue index become same. As soon as tranmit circular buffer is empty, the UART1 transmit 
register empty interrupt is disbled.
****************************************************************************************/
void USART1_IRQHandler(void)
{
	if (USART1->SR & USART_SR_TXE)  // Check if TXE flag is set
	{
		if(txBufferUart1.enqueueIndex != txBufferUart1.dequeueIndex)
		{
			USART1->DR = txBufferUart1.data[txBufferUart1.dequeueIndex];
			txBufferUart1.dequeueIndex++;
			if(txBufferUart1.dequeueIndex >= (uint8_t)MXIMUM_TX_BUFFER_SIZE)
			{
				txBufferUart1.dequeueIndex = 0;
			}
			USART1->CR1 |= USART_CR1_TXEIE;
		}
		else 
		{
			//Disable UART Transmit Register Empty Interrupt if Enqueue and dequeue 
			USART1->CR1 &=~ USART_CR1_TXEIE;
		}
	}
	else if (USART1->SR & USART_SR_RXNE)  // Check if RXE flag is set
	{
		
	}
}

