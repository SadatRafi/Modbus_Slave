#ifndef MODBUS_RTU_H   // Check if MODBUS_RTU_H is not defined
#define MODBUS_RTU_H   // Define MODBUS_RTU_H to prevent further inclusion

#include <stdint.h>    // Include standard integer types (for uint8_t, uint16_t)

#define MAX_COILS 128            // Maximum number of coils (discrete outputs)
#define MAX_DISCRETE_INPUTS 128  // Maximum number of discrete inputs
#define MAX_HOLDING_REGISTERS 64 // Maximum number of holding registers
#define MAX_INPUT_REGISTERS 64   // Maximum number of input registers

uint8_t Read_Coil(uint16_t address);                   				 // Function to read coils
void Write_Coil(uint16_t address, uint8_t value);      				 // Function to write coils
uint16_t Read_Holding_Register(uint16_t address);           	 // Function to read holding registers
void Write_Holding_Register(uint16_t address, uint16_t value); // Function to write holding registers

void Modbus_Slave_ProcessMessage(uint8_t *receivedMessage, uint16_t length);

#endif // MODBUS_RTU_H
