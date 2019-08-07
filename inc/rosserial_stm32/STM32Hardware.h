/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

// Black magic cast to remove volatile. DO NOT TOUCH!!!
#define tbuf  ((uint8_t *)&uart.TX_buffer[0])

// #define DELAY_BETWEEN_TX 8 // ms if BAUD_RATE = 115200 (cf. usart.h)
#define DELAY_BETWEEN_TX 3 // ms if BAUD_RATE = 250K (cf. usart.h)
// #define DELAY_BETWEEN_TX 1 // ms if BAUD_RATE = 500K (cf. usart.h)

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_tim.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern volatile struct UART uart;

extern "C" {
	#include "uart.h"
	#include "delay.h"
	#include <string.h>
}

class STM32Hardware {
  protected:
    UART_HandleTypeDef *huart;
 
    uint32_t rind;
 
    uint32_t twind, tfind;

  public:
    STM32Hardware():
      huart(&huart2), rind(0), twind(0), tfind(0){
    }

    STM32Hardware( UART_HandleTypeDef *huart_):
     huart(huart_), rind(0), twind(0), tfind(0){
    }
  
  	 // Ros::NodeHandle callback
    void init(){
      
    }

	 
	 int read(){
	 	int rx_value = -1;
	 	int dma_count = -1;
	
	 	if (uart.RX_available) {
	 		// __HAL_DMA_GET_COUNTER(__HANDLE__) Returns the number of remaining data units in the current DMAy Streamx transfer
	 		dma_count =  BUFFER_LENGTH_RX - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
					
	 		if( ((int) rind) != dma_count){
	 		    rx_value = uart.RX_buffer[rind++];
	 		    rind &= BUFFER_LENGTH_RX - 1;
	 		}
	 		
	 	}
	 	return rx_value;
	 }


	 void flush(void){
	   if (Uart_is_TX_free()) {			
	      uart.TX_free = 0;    //busy
	      if(twind != tfind){
	        uint16_t len = tfind < twind ? twind - tfind : BUFFER_LENGTH_TX - tfind;
	        HAL_UART_Transmit_DMA(huart,&(tbuf[tfind]), len);
	        tfind = (tfind + len) & (BUFFER_LENGTH_TX - 1);
			  delay_ms(DELAY_BETWEEN_TX); // important for rosserial
	      }
	   }
	 }


	 void write(uint8_t* data, int length){
	   uint32_t n = length;
	   n = n <= BUFFER_LENGTH_TX ? n : BUFFER_LENGTH_TX;

	   uint32_t n_tail = n <= BUFFER_LENGTH_TX - twind ? n : BUFFER_LENGTH_TX - twind;
	   memcpy(&tbuf[twind], data, n_tail);
	   twind = (twind + n) & (BUFFER_LENGTH_TX - 1);

	   if(n != n_tail){
	     memcpy(tbuf, &(data[n_tail]), n - n_tail);
	   }

	   flush();
	 }

    unsigned long time(){ return HAL_GetTick(); }

  protected:
};

#endif

