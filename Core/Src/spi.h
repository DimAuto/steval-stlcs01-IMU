/*
 * spi.h
 *
 *  Created on: Nov 6, 2023
 *      Author: dkalaitzakis
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_

#include "stm32l4xx_hal.h"

typedef struct{
	GPIO_TypeDef  *GPIOx;
	uint16_t pin;
}SPI_CS_t;

HAL_StatusTypeDef SPI_Init(SPI_HandleTypeDef *handler);

HAL_StatusTypeDef SPI_write(SPI_HandleTypeDef *handler, SPI_CS_t cs_pin, uint8_t *data, uint8_t w_len, uint32_t timeout);

HAL_StatusTypeDef SPI_read(SPI_HandleTypeDef *handler, SPI_CS_t cs_pin, uint8_t *w_data, uint8_t w_len, uint8_t *r_data, uint8_t r_len, uint32_t timeout);



#endif /* SRC_SPI_H_ */

