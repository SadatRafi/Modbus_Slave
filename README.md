# MODBUS RTU Slave

## Description

This repository implements the **MODBUS RTU protocol** for a slave device. Modbus RTU is a widely used serial communication protocol in industrial automation systems, following a master-slave architecture. The master initiates the communication by sending a request, and the slave responds accordingly. The protocol ensures data integrity using a **Cyclic Redundancy Check (CRC)** for error detection.

### MODBUS RTU Protocol Overview

Modbus RTU transmits data in binary format (8 data bits) over serial interfaces such as RS-485 or RS-232. The protocol is used for communication between a master (e.g., a PLC) and slave devices (e.g., temperature sensors, controllers). A typical Modbus RTU message frame includes:

- **Slave ID**: The unique address of the slave device (1 byte).
- **Function Code**: Specifies the operation to be performed (e.g., read/write) (1 byte).
- **Data**: Contains the address and the values to be read or written (variable length).
- **CRC**: A 16-bit field used for error-checking during communication.

---

### Master Frame

The **master** sends a request to the **slave** using the following frame format:

| Field         | Size   | Description                            |
|---------------|--------|----------------------------------------|
| **Slave ID**  | 1 byte | Unique address of the slave device     |
| **Function Code** | 1 byte | Operation to be performed (e.g., Read/Write) |
| **Start Address** | 2 bytes | Starting address for the action      |
| **Quantity/Count** | 2 bytes | Number of registers or values to read/write |
| **CRC (Lo-Hi)** | 2 bytes | 16-bit error-checking field            |

#### Example Master Request Frame:
A request from the master to read 2 holding registers starting from address `0x0001` from slave `0x11`:


---

### Slave Response

The **slave** responds to the master with the requested data or an error code if the request fails. The response frame includes:

| Field         | Size   | Description                              |
|---------------|--------|------------------------------------------|
| **Slave ID**  | 1 byte | Address of the responding slave          |
| **Function Code** | 1 byte | Same function code as in the request   |
| **Byte Count** | 1 byte | Number of bytes being returned           |
| **Data**      | Variable | The requested data from the registers  |
| **CRC (Lo-Hi)** | 2 bytes | 16-bit error-checking field            |

#### Example Slave Response Frame:
For two registers containing the values `0x1234` and `0x5678`, the slave would send:


---

### CRC Calculation

The **CRC** is a 16-bit field used for detecting transmission errors. It is calculated using the polynomial `0xA001` and added at the end of every Modbus message. The slave verifies the CRC of the received message and calculates the CRC for the response before sending it back to the master.

---

## Features

- **Function Code 0x03 (Read Holding Registers)**: Fully implemented.
- **Error Handling**: Includes CRC validation and exception handling for invalid function codes or data addresses.
