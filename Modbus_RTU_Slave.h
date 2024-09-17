#ifndef MODBUS_RTU_H   // Check if MODBUS_RTU_H is not defined
#define MODBUS_RTU_H   // Define MODBUS_RTU_H to prevent further inclusion

#include <stdint.h> 

#define DEVICE_ID 101            // Slave Device ID Number
#define MAX_COILS 128            // Maximum number of coils (discrete outputs)
#define MAX_DISCRETE_INPUTS 128  // Maximum number of discrete inputs
#define MAX_HOLDING_REGISTERS 64 // Maximum number of holding registers
#define MAX_INPUT_REGISTERS 64   // Maximum number of input registers

#define ERROR_CODE_CRC_MISMATCH -1
#define ERROR_CODE_DEVICE_ID_MISMATCH -2
#define ERROR_CODE_ILLEGAL_FUNCTION -3
#define ERROR_CODE_UNDEFINED_REGISTER_ADDRESS -4

#define SUCCESSFUL 0

int Modbus_Slave_ProcessMessage(uint8_t *receivedMessage, uint16_t length, uint8_t *responseMessage);
int Load_float_to_Holding_Register(unsigned int firstHoldingRegisterAddress, float userData);

#endif // MODBUS_RTU_H
