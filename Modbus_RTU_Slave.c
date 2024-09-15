#include "MODBUS_RTU.h"


uint16_t Modbus_CRC16(uint8_t *modbusDataFrame, uint16_t modbusDataFrameLength);  // Function to calculate CRC
void Append_CRC(uint8_t *modbusMessage, uint16_t modbusMessageLength);

// Arrays for Modbus data
static uint8_t coils[MAX_COILS / 8];                     // Coils stored as bits
static uint8_t discreteInputs[MAX_DISCRETE_INPUTS / 8];  // Discrete Inputs stored as bits
static uint16_t holdingRegisters[MAX_HOLDING_REGISTERS]; // Holding registers (16-bit values)
static uint16_t inputRegisters[MAX_INPUT_REGISTERS];     // Input registers (16-bit values)

/****************************************************************************************
Author: Sadat Rafi
Date of Last Modification: 15 September 2024
Function: void Modbus_Slave_ProcessMessage(uint8_t *receivedMessage, uint16_t length)
Description: 
*****************************************************************************************/
void Modbus_Slave_ProcessMessage(uint8_t *receivedMessage, uint16_t length) 
{
    // Calculate CRC for received message (excluding the last two CRC bytes)
    uint16_t crcCalculated = Modbus_CRC16(receivedMessage, length - 2);
    uint16_t crcReceived = (receivedMessage[length - 1] << 8) | receivedMessage[length - 2];

    // Check if CRC matches
    if (crcCalculated == crcReceived) {
        uint8_t functionCode = receivedMessage[1];  // Extract function code
        uint16_t startAddress = (receivedMessage[2] << 8) | receivedMessage[3];  // Extract starting address
        uint16_t quantity = (receivedMessage[4] << 8) | receivedMessage[5];  // Extract quantity of registers to read

        switch (functionCode) {
            case 0x03:  // Read Holding Registers
                Modbus_Read_Holding_Registers(receivedMessage[0], startAddress, quantity);  // Call function to process
                break;

            // Add more cases for other function codes...

            default:
                // Handle unsupported function code (send exception response)
                Send_Modbus_Exception(receivedMessage[0], functionCode, 0x01);  // Illegal Function exception
                break;
        }
    } else {
        // Handle CRC error (send exception response)
        Send_Modbus_Exception(receivedMessage[0], receivedMessage[1], 0x03);  // CRC Error exception
    }
}

// Function to handle Modbus function code 0x03 (Read Holding Registers)
void Modbus_Read_Holding_Registers(uint8_t slaveId, uint16_t startAddress, uint16_t quantity) {
    uint8_t responseMessage[256];  // Create response message buffer
    uint16_t byteCount = quantity * 2;  // Each register is 2 bytes

    // Check if the requested registers are within valid range
    if ((startAddress + quantity) > MAX_HOLDING_REGISTERS) {
        // Send illegal data address exception
        Send_Modbus_Exception(slaveId, 0x03, 0x02);  // Illegal Data Address exception
        return;
    }

    // Prepare the response
    responseMessage[0] = slaveId;           // Slave ID
    responseMessage[1] = 0x03;              // Function code 0x03
    responseMessage[2] = byteCount;         // Number of bytes to follow (2 × quantity)

    // Add the requested register values to the response message
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t registerValue = holdingRegisters[startAddress + i];
        responseMessage[3 + i * 2] = (registerValue >> 8) & 0xFF;  // High byte of register value
        responseMessage[4 + i * 2] = registerValue & 0xFF;         // Low byte of register value
    }

    // Append CRC to the response message
    Append_CRC(responseMessage, 3 + byteCount);

    // Send the response message (function to be implemented for your specific communication interface)
    Modbus_Send(responseMessage, 5 + byteCount);  // 3 bytes header + data + 2 bytes CRC
}

/**************************************************************************************************************************
Function: uint16_t Modbus_CRC16(uint8_t *data, uint16_t modbusDataFrameLength)
This function calculates the 16-bit CRC for a Modbus RTU frame.
**************************************************************************************************************************/
uint16_t Modbus_CRC16(uint8_t *modbusDataFrame, uint16_t modbusDataFrameLength) 
{
    uint16_t modbusDataFrameCrc = 0xFFFF; // Initialize CRC with 0xFFFF
    for (int pos = 0; pos < modbusDataFrameLength; pos++) 
    {
        modbusDataFrameCrc ^= (uint16_t)modbusDataFrame[pos];  // XOR the data with the CRC
        for (int bitNumber = 8; bitNumber != 0; bitNumber--) 
        {
            if ((modbusDataFrameCrc & 0x0001) != 0) 
            {
                modbusDataFrameCrc >>= 1;
                modbusDataFrameCrc ^= 0xA001;  // XOR with Modbus polynomial
            }
            else
            {
                modbusDataFrameCrc >>= 1;
            }
        }
    }
    return modbusDataFrameCrc;
}

// Function to append CRC to the message
void Append_CRC(uint8_t *modbusMessage, uint16_t modbusMessageLength)
{
    uint16_t crc = Modbus_CRC16(modbusMessage, modbusMessageLength); 
    modbusMessage[modbusMessageLength] = crc & 0xFF;         // CRCLo
    modbusMessage[modbusMessageLength + 1] = (crc >> 8) & 0xFF;  // CRCHi
}
