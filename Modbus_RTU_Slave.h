#ifndef MODBUS_RTU_H   // Check if MODBUS_RTU_H is not defined
#define MODBUS_RTU_H   // Define MODBUS_RTU_H to prevent further inclusion

#include <stdint.h>    // Include standard integer types (for uint8_t, uint16_t)

#define DEVICE_ID 101            // Slave Device ID Number
#define MAX_COILS 128            // Maximum number of coils (discrete outputs)
#define MAX_DISCRETE_INPUTS 128  // Maximum number of discrete inputs
#define MAX_HOLDING_REGISTERS 64 // Maximum number of holding registers
#define MAX_INPUT_REGISTERS 64   // Maximum number of input registers

#define ERROR_CODE_CRC_MISMATCH -1
#define ERROR_CODE_DEVICE_ID_MISMATCH -2
#define ERROR_CODE_ILLEGAL_FUNCTION -3



int Modbus_Slave_ProcessMessage(uint8_t *receivedMessage, uint16_t length);

#endif // MODBUS_RTU_H
