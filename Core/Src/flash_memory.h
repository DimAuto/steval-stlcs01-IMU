/*
 * flash_memory.h
 *
 *  Created on: Aug 1, 2023
 *      Author: dkalaitzakis
 */

#include "main.h"
#include "lsm6_gyro.h"
#include "Fusion/Fusion.h"

#ifndef SRC_FLASH_MEMORY_H_
#define SRC_FLASH_MEMORY_H_


void FlashReadData (uint32_t StartPageAddress, uint64_t *RxBuf, uint16_t numberofwords);

void Flash_Write_NUM (uint32_t StartSectorAddress, float Num);

float Flash_Read_NUM (uint32_t StartSectorAddress);

uint32_t FlashWriteData (uint32_t StartPageAddress, uint64_t *Data, uint16_t numberofwords);

uint32_t Flash_Write_Vector (uint32_t StartSectorAddress, FusionVector *gyro_data);

uint32_t Flash_isWritten (uint32_t StartSectorAddress);

uint32_t FlashErase(uint32_t StartPageAddress, uint32_t numberofpages);

uint32_t Flash_Read_Vector (uint32_t StartSectorAddress, FusionVector *data);

uint32_t Flash_Write_Matrix (uint32_t StartSectorAddress, FusionMatrix *data);

uint32_t Flash_Read_Matrix (uint32_t StartSectorAddress, FusionMatrix *data);

void Flash_Write_Double (uint32_t StartSectorAddress, double Num);

double Flash_Read_Double (uint32_t StartSectorAddress);

#endif /* SRC_FLASH_MEMORY_H_ */
