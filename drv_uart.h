/*!
	@file   drv_uart.h
	@brief  <brief description here>
	@t.odo	-
	-----------------------------------------------------------------------

	MIT License
	Copyright (c) 2021 Io. D

	Permission is hereby granted, free of charge, to any person obtaining a
	copy of this software and associated documentation files (the "Software"),
	to deal	in the Software without restriction, including without limitation
	the rights to use, copy, modify, merge, publish, distribute, sublicense,
	and/or sell copies of the Software, and to permit persons to whom the
	Software is furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included
	in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
	OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
	THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
	ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
	OTHER DEALINGS IN THE SOFTWARE.
*/
/******************************************************************************
* Preprocessor Definitions & Macros
******************************************************************************/

#ifndef DRIVERS_INC_DRV_UART_H_
#define DRIVERS_INC_DRV_UART_H_

#define UART_TIMER TIM15
#define UART_TIMER_HANDLER htim15

/******************************************************************************
* Includes
******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "usart.h"
#if __has_include("tim.h")
	#include "tim.h"
#endif
#if __has_include("FreeRTOS.h")
	#include "FreeRTOS.h"
#endif
#if __has_include("lib_crc_ccitt.h")
	#include "lib_crc_ccitt.h"
#endif

/******************************************************************************
* Enumerations, structures & Variables
******************************************************************************/

#ifndef ENUM_I_STATUS
#define ENUM_I_STATUS
typedef enum
{
	I_OK 			= 0x00,
	I_INVALID 		= 0x01,
	I_EXISTS 		= 0x02,
	I_NOTEXISTS 		= 0x03,
	I_FAILED 		= 0x04,
	I_EXPIRED 		= 0x05,
	I_UNKNOWN 		= 0x06,
	I_INPROGRESS 		= 0x07,
	I_IDLE			= 0x08,
	I_FULL			= 0x09,
	I_EMPTY			= 0x0A,
	I_YES			= 0x0B,
	I_NO			= 0x0C,
	I_SKIP			= 0x0D,
	I_DEBUG_01 		= 0xE0,
	I_DEBUG_02 		= 0xE1,
	I_DEBUG_03 		= 0xE2,
	I_DEBUG_04 		= 0xE3,
	I_DEBUG_05 		= 0xE4,
	I_DEBUG_06 		= 0xE5,
	I_DEBUG_07 		= 0xE6,
	I_DEBUG_08 		= 0xE7,
	I_DEBUG_09 		= 0xE8,
	I_DEBUG_10 		= 0xE9,
	I_DEBUG_11 		= 0xEA,
	I_DEBUG_12 		= 0xEB,
	I_DEBUG_13 		= 0xEC,
	I_DEBUG_14 		= 0xED,
	I_DEBUG_15 		= 0xEE,
	I_DEBUG_16 		= 0xEF,
	I_MEMUNALIGNED 		= 0xFD,
	I_NOTIMPLEMENTED 	= 0xFE,
	I_ERROR 		= 0xFF
}i_status;
#endif

struct uart_callback
{
	void (*callback)(uint8_t *buffer, uint32_t size);
	struct uart_callback *next;
};

typedef struct uart_callback uart_callback_t;

typedef struct
{
	void (*mx_init)();
	UART_HandleTypeDef *huart;
	uart_callback_t * callbacks;
	GPIO_TypeDef * tx_port;
	uint16_t tx_pin;
	uint16_t send_custom_low;
	uint8_t parse_as_protocol;
	uint8_t* in_buffer;
	uint32_t in_buffer_sz;
	uint32_t in_buffer_tsz;
	uint8_t raw_timout;
	uint8_t max_raw_timout;
}uart_t;

/******************************************************************************
* Declaration | Public Functions
******************************************************************************/

i_status uart_initialize(uart_t* uart);
i_status uart_send(uart_t* uart, uint8_t *buffer, uint32_t size);
i_status uart_send_message(uart_t* uart, uint8_t *buffer, uint32_t size);
i_status uart_send_lowpulse(uart_t* uart, uint32_t microseconds);
i_status uart_callback_add(uart_t* canbus, void(*cb)(uint8_t *buffer, uint32_t size));

#ifdef HAL_TIM_MODULE_ENABLED
void uart_tim_complete_cb(TIM_HandleTypeDef *htim);
#endif
/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif
