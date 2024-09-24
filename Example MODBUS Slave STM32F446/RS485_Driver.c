#include "UART1_Driver.h"
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
****************************************************************************************/
void initialize_UART1(int baudRate)
{
	//Enable Clock
	//Enable TX
	//Enable RX
	//Enable
}

typedef struct
{
	uint8_t data[MXIMUM_TX_BUFFER_SIZE];
	uint8_t enqueueIndex;
	uint8_t dequeueIndex;
	uint16_t length;
}TxBuffer;

static TxBuffer txBufferUart1;

int Transfer_Message(uint8_t *payload, uint16_t length)
{
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
		else return ERROR_UART1_TRANSMISSION_ONGOING;
		
		//Enable USART Transmit Register Empty Interrupt
		USART1->CR1 |= USART_CR1_TXEIE;
	}
	return SUCCESS;
}

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
}

