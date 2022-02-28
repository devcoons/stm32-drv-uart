/*!
	@file   drv_uart.c
	@brief  <brief description here>
	@t.odo	-
	---------------------------------------------------------------------------

	MIT License
	Copyright (c) 2021 Io. D

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/
/******************************************************************************
* Preprocessor Definitions & Macros
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/

#include "drv_uart.h"

#ifdef DRV_UART_ENABLED
/******************************************************************************
* Preprocessor Post-Definitions & Macros
******************************************************************************/


#ifdef LIB_CRYPTO_ENABLE_CRC
#define UART_MSG_MIN_CHAR 4
#else
#define UART_MSG_MIN_CHAR 2
#endif

/******************************************************************************
* Enumerations, structures & Variables
******************************************************************************/

#ifdef UART_TIMER
	static uint8_t is_timer_initialized = 0;
#endif

static uart_t* uart_interfaces[8] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
static uint32_t uart_interfaces_cnt = 0;

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

/******************************************************************************
* Definition  | Static Functions
******************************************************************************/

#ifdef DRV_UART_TIMER

static void MX_UART_TIM_Init(void)
{
	if(HAL_TIM_Base_GetState(&UART_TIMER_HANDLER) != HAL_TIM_STATE_RESET)
		return;

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	UART_TIMER_HANDLER.Instance = UART_TIMER;
	UART_TIMER_HANDLER.Init.Prescaler = (HAL_RCC_GetPCLK2Freq()/10000000) - 1;
	UART_TIMER_HANDLER.Init.CounterMode = TIM_COUNTERMODE_UP;
	UART_TIMER_HANDLER.Init.Period = 199;
	UART_TIMER_HANDLER.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	UART_TIMER_HANDLER.Init.RepetitionCounter = 0;
	UART_TIMER_HANDLER.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if (HAL_TIM_Base_Init(&UART_TIMER_HANDLER) != HAL_OK)
	{
		Error_Handler();
  	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	if (HAL_TIM_ConfigClockSource(&UART_TIMER_HANDLER, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&UART_TIMER_HANDLER, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_Base_Start_IT(&UART_TIMER_HANDLER);
}
#endif

/******************************************************************************
* Definition  | Public Functions
******************************************************************************/

i_status uart_initialize(uart_t* uart)
{
	#ifdef DRV_UART_TIMER
	if(is_timer_initialized == 0)
	{
		MX_UART_TIM_Init();
		is_timer_initialized = 1;
	}
	#endif

	uart->in_buffer_sz = 0;
	memset(uart->in_buffer,0,uart->in_buffer_tsz);

	#ifndef DRV_UART_TIMER
		uart->parse_as_protocol = 1;
	#endif
	uart->mx_init();

	for(register uint32_t i=0;i<uart_interfaces_cnt;i++)
	{
		if(uart_interfaces[i] == uart)
		{
			HAL_UART_Receive_IT(uart->huart,&uart->in_buffer[uart->in_buffer_sz],1);
			return I_OK;
		}
	}
	uart_interfaces[uart_interfaces_cnt] = uart;
	uart_interfaces_cnt++;

	HAL_UART_Transmit(uart->huart, (uint8_t*)"\r\n", 2,100);
	HAL_UART_Receive_IT(uart->huart,&uart->in_buffer[uart->in_buffer_sz],1);

	return I_OK;
}

i_status uart_send(uart_t* uart, uint8_t *buffer, uint32_t size)
{
#ifdef DRV_UART_TIMER
	if(uart->send_custom_low !=0)
		return I_INPROGRESS;
#endif
	return HAL_UART_Transmit(uart->huart, buffer, size,100) == HAL_OK ? I_OK : I_ERROR;
}

i_status uart_send_message(uart_t* uart, uint8_t *buffer, uint32_t size)
{
#ifdef DRV_UART_TIMER
	if(uart->send_custom_low !=0)
		return I_INPROGRESS;
#endif

#ifdef LIB_CRYPTO_ENABLE_CRC
	uint16_t calc_crc = crc16_ccitt(0xFFFF, buffer, size);
	uint8_t crc[2];
	crc[0] = (calc_crc & 0xFF00) >> 8;
	crc[1] = (calc_crc & 0x00FF) >> 0;

	HAL_UART_Transmit(uart->huart, crc, 2, 100);
#endif
	HAL_UART_Transmit(uart->huart, buffer, size,100);
	HAL_UART_Transmit(uart->huart, (uint8_t*)"\r\n", 2,100);
	return I_OK;
}

i_status uart_callback_add(uart_t* uart, void(*cb)(uint8_t *buffer, uint32_t size))
{
	__disable_irq();

	uart_callback_t* node = (uart_callback_t*)malloc(sizeof(uart_callback_t));

#ifdef INC_FREERTOS_H
	if(node == NULL)
		node = (uart_callback_t*)pvPortMalloc(sizeof(uart_callback_t));
#endif
	if(node == NULL)
		goto uart_callback_add_error;

	node->callback = cb;
	node->next = uart->callbacks;

	uart->callbacks = node;

	__enable_irq();
	return I_OK;
	uart_callback_add_error:
	__enable_irq();
	return I_ERROR;

	return I_NOTIMPLEMENTED;
}

#ifdef DRV_UART_TIMER
i_status uart_send_lowpulse(uart_t* uart, uint32_t microseconds)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = uart->tx_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_WritePin(uart->tx_port, uart->tx_pin, GPIO_PIN_SET);
	HAL_UART_DeInit(uart->huart);
	HAL_GPIO_Init(uart->tx_port, &GPIO_InitStruct);
	uart->send_custom_low =1 + (microseconds / 10);
	__ISB();
	__DSB();
	return I_OK;
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uart_t* current_uart = NULL;
	static uart_callback_t* callback_item = NULL;
	static uint32_t is_complete = 0;

	for(register uint32_t i=0;i<uart_interfaces_cnt;i++)
		if(uart_interfaces[i]->huart == huart)
			current_uart = uart_interfaces[i];

	if(current_uart == NULL)
		return;

	if(current_uart->parse_as_protocol == 1)
	{
		if(current_uart->in_buffer_sz >= 1)
		{
			if(current_uart->in_buffer[current_uart->in_buffer_sz-1] == '\r'
				&& current_uart->in_buffer[current_uart->in_buffer_sz] == '\n')
			{
				is_complete = current_uart->in_buffer_sz+1;
				current_uart->in_buffer_sz = 0;
			}
			else
			{
				is_complete = 0;
				current_uart->in_buffer_sz = current_uart->in_buffer_sz < current_uart->in_buffer_tsz ? current_uart->in_buffer_sz + 1 : 0;
			}
		}
		else
		{
			is_complete = 0;
			current_uart->in_buffer_sz = current_uart->in_buffer_sz < current_uart->in_buffer_tsz ? current_uart->in_buffer_sz + 1 : 0;
		}

		HAL_UART_Receive_IT(current_uart->huart,&current_uart->in_buffer[current_uart->in_buffer_sz],1);

		if(current_uart->callbacks != NULL && is_complete >= UART_MSG_MIN_CHAR)
		{
#ifdef LIB_CRYPTO_ENABLE_CRC
			uint16_t crc = 0 + (current_uart->in_buffer[0] << 8 | current_uart->in_buffer[1]);
			uint16_t calc_crc = crc16_ccitt(0xFFFF, &current_uart->in_buffer[2], is_complete-4);

			if(crc!=calc_crc)
				return;
#endif
			callback_item = current_uart->callbacks;
			while(callback_item!=NULL)
			{
#ifdef LIB_CRYPTO_ENABLE_CRC
				callback_item->callback(&current_uart->in_buffer[2], is_complete-UART_MSG_MIN_CHAR);
#else
				callback_item->callback(&current_uart->in_buffer[0], is_complete-UART_MSG_MIN_CHAR);
#endif

				callback_item = callback_item->next;
			}
		}
	}
	else
	{
#ifdef DRV_UART_TIMER
		current_uart->raw_timout = current_uart->max_raw_timout;
#endif
		current_uart->in_buffer_sz = current_uart->in_buffer_sz < current_uart->in_buffer_tsz ? current_uart->in_buffer_sz + 1 : 0;
		HAL_UART_Receive_IT(current_uart->huart,&current_uart->in_buffer[current_uart->in_buffer_sz],1);
#ifdef DRV_UART_TIMER
		current_uart->raw_timout = current_uart->max_raw_timout;
#endif
	}
}
#ifdef DRV_UART_TIMER
void uart_tim_complete_cb(TIM_HandleTypeDef *htim)
{
	if (htim->Instance != UART_TIMER)
		return;

	static uart_t* current_uart = NULL;
	static uart_callback_t* callback_item = NULL;
	for(register uint32_t i=0;i<uart_interfaces_cnt;i++)
	{
		if(uart_interfaces[i] != NULL)
		{
			if(uart_interfaces[i]->parse_as_protocol == 0 && uart_interfaces[i]->in_buffer_sz != 0)
			{
				current_uart = uart_interfaces[i];
				if(current_uart->raw_timout != 0)
				{
					current_uart->raw_timout = current_uart->raw_timout - 1;
				}
				else
				{
					callback_item = current_uart->callbacks;
					while(callback_item!=NULL)
					{
						callback_item->callback(current_uart->in_buffer, current_uart->in_buffer_sz);
						callback_item = callback_item->next;
					}
					current_uart->in_buffer_sz = 0;
					HAL_UART_AbortReceive_IT(current_uart->huart);
					__ISB();
					__DSB();
					HAL_UART_Receive_IT(current_uart->huart,&current_uart->in_buffer[current_uart->in_buffer_sz],1);
				}
			}
		}
		if(uart_interfaces[i] != NULL)
		{
			if(uart_interfaces[i]->send_custom_low != 0)
			{
				HAL_GPIO_WritePin(uart_interfaces[i]->tx_port, uart_interfaces[i]->tx_pin, GPIO_PIN_RESET);
				uart_interfaces[i]->send_custom_low--;
				if(uart_interfaces[i]->send_custom_low == 0)
				{
					HAL_GPIO_WritePin(uart_interfaces[i]->tx_port, uart_interfaces[i]->tx_pin, GPIO_PIN_SET);
					uart_initialize(uart_interfaces[i]);
				}
			}
		}
	}
}
#endif
#endif
/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
