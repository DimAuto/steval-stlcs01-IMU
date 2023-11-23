/*
 * uart.c
 *
 *  Created on: Jun 16, 2023
 *      Author: dkalaitzakis
 */

#include "uart.h"
#include "ring_buffer.h"
#include "main.h"
#include "message_handler.h"


UART_HandleTypeDef huart1;
UART_HandleTypeDef huart5;

RB_t uart5RXrb;
RB_t uart1RXrb;

uint8_t rxChar = 0x00;
uint8_t prvRxChar = 0x00;

uint8_t ack_rcv_flag = 0;

extern osMessageQueueId_t messageQueueHandle;
extern osEventFlagsId_t ack_rcvd;

static void error_Handler(void);


/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 921600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	RB_init(&uart1RXrb, RB_SIZE);

}

void MX_UART5_Init(void)
{

	huart5.Instance = UART5;
	huart5.Init.BaudRate = 115200;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart5) != HAL_OK)
	{
		Error_Handler();
	}
	RB_init(&uart5RXrb, RB_SIZE);
	uart_write_debug("UART5 initialized\r\n", 10);

}

void error_Handler(void){
	uart_write_debug("Failed to Init UART5\r\n", 10);
}


HAL_StatusTypeDef uart_receive_it(UART_select device){
	UART_HandleTypeDef *huart;

	switch (device){
	case UART_DEBUG:
		huart = &huart1;
		break;
	case UART_NYX:
		huart = &huart5;
		break;
	case UART_IRIS:
		huart = &huart1;
		break;
	}
	return HAL_UART_Receive_IT(huart, &rxChar, 1);
}


HAL_StatusTypeDef uart_write_debug(uint8_t *pData, uint32_t Timeout){
	return HAL_UART_Transmit(&huart5,pData,strlen(pData),Timeout);// Sending in normal mode
}

HAL_StatusTypeDef uart_write(uint8_t *pData, uint8_t len, UART_select device, uint32_t Timeout){
	UART_HandleTypeDef *huart;
	switch (device){
	case UART_DEBUG:
		huart = &huart1;
		break;
	case UART_NYX:
		huart = &huart5;
		break;
	case UART_IRIS:
		huart = &huart1;
		break;
	}
	if (len == 0){
		return HAL_UART_Transmit(huart,pData,strlen(pData),Timeout);// Sending in normal mode
	}
	return HAL_UART_Transmit(huart,pData,len,Timeout);// Sending in normal mode
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if (UartHandle->Instance == UART5)
	{
		if (ack_rcv_flag == 1){
			if ((rxChar != ESC) && (prvRxChar == ACK)){
				osEventFlagsSet(ack_rcvd, ACK_FLAG);
				prvRxChar = 0x00;
				ack_rcv_flag = 0;
			}
			else if ((rxChar != ESC) && (prvRxChar == NACK)){
				osEventFlagsSet(ack_rcvd, NACK_FLAG);
				prvRxChar = 0x00;
				ack_rcv_flag = 0;
			}

		}
		else{
			if (((rxChar == ACK) || (rxChar == NACK)) && (prvRxChar == 0xFF)){
				ack_rcv_flag = 1;
				prvRxChar = rxChar;
			}
			else if ((rxChar == ETX) && (prvRxChar != ESC)){
				uint8_t start_ch = 0;
				start_ch = RB_pop(&uart5RXrb);
				if (start_ch == STX){
					uint8_t rb_len = RB_size(&uart5RXrb);
					RB_pushFront(&uart5RXrb, rb_len);
					osMessageQueuePut(messageQueueHandle, uart5RXrb.buffer, 0U, 0U);
					prvRxChar = 0xFF;
				}
				RB_clear(&uart5RXrb);
			}
			else if ((rxChar == ESC) && (prvRxChar != ESC)){
				 prvRxChar = rxChar;
			}
			else if ((rxChar == ESC) && (prvRxChar == ESC)){
				RB_push(&uart5RXrb, rxChar);
				prvRxChar = 0x00;
			}
			else {
			  RB_push(&uart5RXrb, rxChar);
			  prvRxChar = rxChar;
			  if (rxChar == 0xFF) prvRxChar = 0x00;
			}

		}
		HAL_UART_Receive_IT(&huart5, &rxChar, 1);
	}

}
