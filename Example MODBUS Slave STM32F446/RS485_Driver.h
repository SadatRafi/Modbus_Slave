#include <stdint.h>

#define MXIMUM_TX_BUFFER_SIZE 256

#define ERROR_UART1_TRANSMISSION_ONGOING -1
#define ERROR_INCREASE_CIRCULAR_BUFFER_SIZE -2

void initialize_RS485(int baudRate);