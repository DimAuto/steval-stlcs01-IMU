/*
 * spi.c
 *
 *  Created on: Nov 6, 2023
 *      Author: dkalaitzakis
 */

#include "spi.h"
#include <stdint.h>
#include "main.h"

HAL_StatusTypeDef SPI_Init(SPI_HandleTypeDef *handler){
	handler->Instance = SPI2;
	handler->Init.Mode = SPI_MODE_MASTER;
	handler->Init.Direction = SPI_DIRECTION_1LINE;
	handler->Init.DataSize = SPI_DATASIZE_8BIT;
	handler->Init.CLKPolarity = SPI_POLARITY_LOW;
	handler->Init.CLKPhase = SPI_PHASE_1EDGE;
	handler->Init.NSS = SPI_NSS_SOFT;
	handler->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	handler->Init.FirstBit = SPI_FIRSTBIT_MSB;
	handler->Init.TIMode = SPI_TIMODE_DISABLE;
	handler->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	handler->Init.CRCPolynomial = 7;
	handler->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	handler->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	return HAL_SPI_Init(handler);
}


HAL_StatusTypeDef SPI_write(SPI_HandleTypeDef *handler, SPI_CS_t cs_pin, uint8_t *data, uint8_t w_len, uint32_t timeout){
	HAL_GPIO_WritePin(cs_pin.GPIOx, cs_pin.pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit (handler, data, w_len, timeout);
	HAL_GPIO_WritePin(cs_pin.GPIOx, cs_pin.pin, GPIO_PIN_SET);
}


HAL_StatusTypeDef SPI_read(SPI_HandleTypeDef *handler, SPI_CS_t cs_pin, uint8_t *w_data, uint8_t w_len, uint8_t *r_data, uint8_t r_len, uint32_t timeout){
	HAL_GPIO_WritePin(cs_pin.GPIOx, cs_pin.pin, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(handler, w_data, r_data, r_len, timeout);
	HAL_SPI_Transmit(handler, w_data, w_len, timeout);
	HAL_SPI_Receive(handler, r_data, r_len, timeout);
	HAL_GPIO_WritePin(cs_pin.GPIOx, cs_pin.pin, GPIO_PIN_SET);
}
