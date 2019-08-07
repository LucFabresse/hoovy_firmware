/** Basic UART communication with whatever is hooked 
* up to the long UART wires (closer to the speaker).
*/

#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f1xx_hal.h"

#define BUFFER_LENGTH_RX 512
#define BUFFER_LENGTH_TX 512
#define BAUD_RATE 115200
	
struct UART {
	volatile uint8_t RX_buffer[BUFFER_LENGTH_RX];
	volatile uint8_t TX_buffer[BUFFER_LENGTH_TX];
	volatile uint8_t RX_pointer;
	volatile uint8_t RX_available;
	volatile uint8_t TX_free;
};

void MX_USART2_UART_Init(void);

uint8_t Uart_RX_process();
void Uart_TX(char *message);

uint8_t Uart_is_RX_available(void);
uint8_t Uart_is_TX_free(void);

extern void error_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
