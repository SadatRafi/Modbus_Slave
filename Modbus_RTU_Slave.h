#ifndef MODBUS_RTU_H   // Check if MODBUS_RTU_H is not defined
#define MODBUS_RTU_H   // Define MODBUS_RTU_H to prevent further inclusion

#include <stdint.h> 

//all the data handled via MODBUS (bits, registers) must be located in device application memory
#define DEVICE_APPLICATION_MEMORY 256  

typedef struct
{
	uint16_t numberOfCoils;         
	uint16_t numberOfDiscreteInputs;
  uint16_t numberOfInputRegisters;
	uint16_t numberOfHoldingRegisters;
}ModbusApplicationMemory;

int Initialize_MODBUS(uint8_t deviceID,ModbusApplicationMemory modbusSlaveMemoryMap);


#define ERROR_CODE_CRC_MISMATCH -1
#define ERROR_CODE_DEVICE_ID_MISMATCH -2
#define INSUFFICIENT_DEVICE_APPLICATION_MEMORY -3
#define SUCCESSFUL 0

/*************************************************************************************************************
Author: Sadat Rafi
Date Of Last Modification: 18 September 2024
Function: Modbus_Slave_ProcessMessage
User Instruction: 
	This function processes a Modbus message received from a master device. The user should pass the received 
message along with its length, and an array to store the response message. The function will validate the 
message, check the CRC, and handle the requested Modbus function (e.g., reading holding registers). If any 
Modbus-related errors occur, it will reply standard Modbus exception codes (defined by Modicon). Additionally,
this function returns developer-defined error codes, such as device ID mismatches or CRC Mismatch .
**************************************************************************************************************/
int Modbus_Slave_ProcessMessage(uint8_t *receivedMessage, uint16_t length, uint8_t *responseMessage);

int Load_float_to_Holding_Register(unsigned int firstHoldingRegisterAddress, float userData);

#endif // MODBUS_RTU_H
