/*
 * lsm6_gyro.c
 *
 *  Created on: May 25, 2023
 *      Author: dkalaitzakis
 */


#include <string.h>
#include "stm32l4xx_hal.h"
#include "lsm6_gyro.h"
#include "helpers.h"
#include "uart.h"
#include "flash_memory.h"
#include "Fusion/Fusion.h"
#include "ellipsoid_fit.h"
#include "spi.h"
#include "main.h"


// I2C object
SPI_HandleTypeDef hspi2;
SPI_CS_t lsm6_cs;
SPI_CS_t lsm303_m_cs;
SPI_CS_t lsm303_a_cs;

//static void debugPrintMEMS(mems_data_t *mems_data);

float magn_samples[3*MAGN_CALIB_SAMPLES];
float acc_samples[3*ACC_CALIB_SAMPLES];

FusionVector hardiron;
FusionMatrix softiron;

FusionVector acc_offset;
FusionMatrix acc_misalign;

uint16_t gyro_offset_counter = 0;
uint16_t magn_calib_counter = 0;
uint16_t acc_calib_counter = 0;
gyro_data_t gyro_sum;
FusionVector gyro_mean;


extern FusionAhrs ahrs;

void tick_gyro(mems_data_t * mems_data){
	lsm303_acc_read(mems_data);
    lis3_magn_read(mems_data);
    gyro_read(mems_data);
//    debugPrintMEMS(mems_data);
}


HAL_StatusTypeDef lsm6_bus_init(void)
{
	return SPI_Init(&hspi2);

}

uint8_t whoIam_lsm6(void){
	uint8_t w_data[2] = {0};
	uint8_t addr[3] = {0};
	HAL_StatusTypeDef res = 0;
	w_data[0] = WHO_I_AM | 0x80;
	res = SPI_read(&hspi2, lsm6_cs, w_data, 1, addr, 3, 100);
	if (res != HAL_OK){
		return res;
	}
	return addr[0];
}

uint8_t whoIam_lsm303(void){
	uint8_t w_data[2] = {0};
	uint8_t addr[3] = {0};
	HAL_StatusTypeDef res = 0;
	w_data[0] = WHO_AM_I_A | 0x80;
	res = SPI_read(&hspi2, lsm303_a_cs, w_data, 1, addr, 3, 100);
	if (res != HAL_OK){
		return res;
	}
 	return addr[0];
}


HAL_StatusTypeDef gyro_init(void){
	uint8_t w_data[3] = {0};
	HAL_StatusTypeDef res = HAL_OK;
	lsm6_cs.GPIOx = GPIOB;
	lsm6_cs.pin = GPIO_PIN_12;
    w_data[0] = CTRL2_G;
    w_data[1] = 0x5C;   //gyro 208Hz-2000dps
    res += SPI_write(&hspi2, lsm6_cs, w_data, 2, 20);
    w_data[0] = CTRL7_G;
	w_data[1] = 0x00;	//HPF and HighPerf on
    res += SPI_write(&hspi2, lsm6_cs, w_data, 2, 20);
    w_data[0] = CTRL3_C;
	w_data[1] = 0x4C;   // block data update - reg addr auto incr
    res += SPI_write(&hspi2, lsm6_cs, w_data, 2, 20);
    return res;
}

HAL_StatusTypeDef lsm6_acc_init(void){
	uint8_t w_data[2] = {0};
	HAL_StatusTypeDef res = HAL_OK;
	lsm6_cs.GPIOx = GPIOB;
	lsm6_cs.pin = GPIO_PIN_12;
    w_data[0] = CTRL1_XL;
//	w_data[1] = 0x50;
	w_data[1] = 0x00; //Disable acc.
    res += SPI_write(&hspi2, lsm6_cs, w_data, 2, 20);
    w_data[0] = CTRL10_C;
	w_data[1] = 0x20; //Enable timestamp
	res += SPI_write(&hspi2, lsm6_cs, w_data, 2, 20);
    return res;
}

HAL_StatusTypeDef magn_init(void){
	uint8_t w_data[2] = {0};
	lsm303_m_cs.GPIOx = GPIOB;
	lsm303_m_cs.pin = GPIO_PIN_1;
	lsm303_a_cs.GPIOx = GPIOC;
	lsm303_a_cs.pin = GPIO_PIN_4;
	HAL_StatusTypeDef res = HAL_OK;
	w_data[0] = CFG_REG_A_M;
	w_data[1] = 0x8C;
	res += SPI_write(&hspi2, lsm303_m_cs, w_data, 2, 20);
//	w_data[0] = CFG_REG_B_M;
//	w_data[1] = 0x50;
//	res += SPI_Write(&hspi2, lsm303_m_cs, w_data, 2, 20);
	w_data[0] = CTRL_REG1_A;
	w_data[1] = 0x67;  //200Hz Normal mode
	res += SPI_write(&hspi2, lsm303_a_cs, w_data, 2, 20);
	w_data[0] = CTRL_REG4_A;
	w_data[1] = 0x01;
	res += SPI_write(&hspi2, lsm303_a_cs, w_data, 2, 20);
    return res;
}

HAL_StatusTypeDef gyro_read(mems_data_t *mems_data){
	uint8_t w_data[2]={0};
	uint8_t data[6]={0};
	int16_t gyro_x, gyro_y, gyro_z;
	HAL_StatusTypeDef res = HAL_OK;
	w_data[0] = OUTX_L_G | 0x80;
	res += SPI_read(&hspi2, lsm6_cs, w_data, 1, data, 6, 50);
//    if (data[0] == 255){
//    	res += SPI_read(&hspi2, lsm6_cs, 0x00, 1, data, 6, 50);
//	}
    gyro_x = ((int16_t)((data[1] << 8) | data[0]));
    gyro_y = ((int16_t)((data[3] << 8) | data[2]));
    gyro_z = ((int16_t)((data[5] << 8) | data[4]));
#ifndef GYRO_TS
    mems_data->timestamp = osKernelGetTickCount();
#else
    uint8_t ts_data[4]={0};
    w_data[0] = TIMESTAMP0 | 0x80;
	res += SPI_read(&hspi2, lsm6_cs, w_data, 1, ts_data, 6, 50);
    mems_data->timestamp = (uint32_t) ((ts_data[2]<<16)|(ts_data[1]<<8)|(ts_data[0]));
#endif
    mems_data->gyro.gyro_x = - (float)(gyro_x * 0.072f);// * -1.0f;			Scaling for 2000dps.
    mems_data->gyro.gyro_y = - (float)(gyro_y * 0.072);// * -1.0f;
    mems_data->gyro.gyro_z =   (float)(gyro_z * 0.072f);// * -1.0f;
    return res;
}

//HAL_StatusTypeDef gyroReadTS(mems_data_t){
//
//}

HAL_StatusTypeDef lsm6_acc_read(mems_data_t *mems_data){
	uint8_t w_data[1]={0};
	uint8_t data[6] = {0};
	int16_t acc_x, acc_y, acc_z;
	HAL_StatusTypeDef res = HAL_OK;
	w_data[0] = OUTX_L_A | 0xC0;
	res = SPI_read(&hspi2, lsm6_cs, w_data, 1, data, 6, 50);
	if (res != HAL_OK){
		return res;
	}
    acc_x = ((int16_t)((data[1] << 8) | data[0]));
    acc_y = ((int16_t)((data[3] << 8) | data[2]));
    acc_z = ((int16_t)((data[5] << 8) | data[4]));
    mems_data->acc.acc_x = 	(float)(acc_x / 16384.0f);//  * -1.0f;
    mems_data->acc.acc_y = 	(float)(acc_y / 16384.0f);// * -1.0f;
    mems_data->acc.acc_z =	(float)(acc_z / 16384.0f);// * -1.0f;
    return res;
}

HAL_StatusTypeDef lsm303_acc_read(mems_data_t *mems_data){
	uint8_t w_data[2]={0};
	uint8_t data[6] = {0};
	int16_t acc_x, acc_y, acc_z;
	HAL_StatusTypeDef res = HAL_OK;
	w_data[0] = OUT_X_L_A | 0xC0;
	res = SPI_read(&hspi2, lsm303_a_cs, w_data, 1, data, 6, 50);
	if (res != HAL_OK){
		return res;
	}
    acc_x = ((int16_t)((data[1] << 8) | data[0]));
    acc_y = ((int16_t)((data[3] << 8) | data[2]));
    acc_z = ((int16_t)((data[5] << 8) | data[4]));
    mems_data->acc.acc_y = 	(float)(acc_x / 16384.0f);//  * -1.0f;
    mems_data->acc.acc_x = 	(float)(acc_y / 16384.0f);// * -1.0f;
    mems_data->acc.acc_z =	(float)(acc_z / 16384.0f);// * -1.0f;
    return res;
}

HAL_StatusTypeDef lis3_magn_read(mems_data_t *mems_data){
	uint8_t w_data[1]={0};
	uint8_t data[6] = {0};
    int16_t magn_x, magn_y, magn_z;
    HAL_StatusTypeDef res = HAL_OK;

    uint8_t OUT_AUTO_I_M = (OUTX_L_REG_M | OUT_AUTO_I);

    w_data[0] = OUTX_L_REG_M | 0xC0;
	res = SPI_read(&hspi2, lsm303_m_cs, w_data, 1, data, 6, 50);
    if (res != HAL_OK){
    	return res;
	}
    magn_x = ((int16_t)((data[1] << 8) | data[0]));
    magn_y = ((int16_t)((data[3] << 8) | data[2]));
    magn_z = ((int16_t)((data[5] << 8) | data[4]));
    mems_data->magn.magn_y = (float)(magn_x / 10.0f);
    mems_data->magn.magn_x = (float)(magn_y / 10.0f);
    mems_data->magn.magn_z = (float)(magn_z / 10.0f);
    return res;
}


uint8_t gyro_offset_calculation(mems_data_t *mems_data){
	gyro_read(mems_data);
	gyro_sum.gyro_x += mems_data->gyro.gyro_x;
	gyro_sum.gyro_y += mems_data->gyro.gyro_y;
	gyro_sum.gyro_z += mems_data->gyro.gyro_z;
	gyro_offset_counter++;
	if (gyro_offset_counter >= GYRO_CALIB_SAMPLES){
		gyro_mean.array[0] = gyro_sum.gyro_x / gyro_offset_counter;
		gyro_mean.array[1] = gyro_sum.gyro_y / gyro_offset_counter;
		gyro_mean.array[2] = gyro_sum.gyro_z / gyro_offset_counter;
		setGyroOffset(gyro_mean);
		gyro_offset_counter = 0;
		Flash_Write_Vector(GYRO_OFFSET_ADDR, &gyro_mean);
		return 0;
	}
	return 1;
}

uint8_t magneto_sample(mems_data_t *mems_data){
	lis3_magn_read(mems_data);
	magn_samples[magn_calib_counter * 3 + 0] = mems_data->magn.magn_x;
	magn_samples[magn_calib_counter * 3 + 1] = mems_data->magn.magn_y;
	magn_samples[magn_calib_counter * 3 + 2] = mems_data->magn.magn_z;
	magn_calib_counter++;
	if (magn_calib_counter >= MAGN_CALIB_SAMPLES){
		magn_calib_counter = 0;
		magneto_calculate(magn_samples, MAGN_CALIB_SAMPLES, &hardiron, &softiron);
		setMagnCoeff(hardiron, softiron);
		Flash_Write_Vector(MAGN_HIRON_ADDR, &hardiron);
		Flash_Write_Matrix(MAGN_SIRON_ADDR, &softiron);
		setMagnCalibratedFlag(true);
		return 0;
	}
	return 1;
}

uint8_t magnetoSetErrorCoeff(mems_data_t *mems_data){
	FusionEuler angles;
	lsm303_acc_read(mems_data);
	lis3_magn_read(mems_data);
	gyro_read(mems_data);
	magn_calib_counter++;
	if (magn_calib_counter == MAGN_ERROR_COEFF_SAMPLES -1){
		setMagnCalibratedFlag(true);
	}
	FusionCalcHeading(mems_data, &angles);
	if (magn_calib_counter >= MAGN_ERROR_COEFF_SAMPLES){
		magn_calib_counter = 0;
		return 0;
	}
	return 1;
}

uint8_t acc_sample(mems_data_t *mems_data){
	lsm303_acc_read(mems_data);
	acc_samples[acc_calib_counter * 3 + 0] = mems_data->acc.acc_x;
	acc_samples[acc_calib_counter * 3 + 1] = mems_data->acc.acc_y;
	acc_samples[acc_calib_counter * 3 + 2] = mems_data->acc.acc_z;
	acc_calib_counter++;
	if (acc_calib_counter >= ACC_CALIB_SAMPLES){
		acc_calib_counter = 0;
		magneto_calculate(acc_samples, ACC_CALIB_SAMPLES, &acc_offset, &acc_misalign);
		setAccCoeff(acc_offset, acc_misalign);
		Flash_Write_Vector(ACC_VECTOR_ADDR, &acc_offset);
		Flash_Write_Matrix(ACC_MATRIX_ADDR, &acc_misalign);
		return 0;
	}
	return 1;
}

//void debugPrintMEMS(mems_data_t *mems_data){
//	uint8_t text[20] = {0};
//	uart_write("Raw:", 0, UART_NYX, 50);
//	memcpy(text,0,20);
//	sprintf(text, "%f,", mems_data->acc.acc_x);
//	uart_write(text, 0, UART_NYX, 50);
//	memcpy(text,0,20);
//	sprintf(text, "%f,", mems_data->acc.acc_y);
//	uart_write(text, 0, UART_NYX, 50);
//	memcpy(text,0,20);
//	sprintf(text, "%f\r\n,", mems_data->acc.acc_z);
//	uart_write(text, 0, UART_NYX, 50);
//	memcpy(text,0,20);
//	sprintf(text, "%f,", mems_data->gyro.gyro_x);
//	uart_write_debug(text, 20);
//	memcpy(text,0,20);
//	sprintf(text, "%f,", mems_data->gyro.gyro_y);
//	uart_write_debug(text, 20);
//	memcpy(text,0,20);
//	sprintf(text, "%f\r\n", mems_data->gyro.gyro_z);
//	uart_write_debug(text, 20);
//	memcpy(text,0,20);
//	sprintf(text, "%f,", mems_data->magn_x);
//	uart_write_debug(text, 20);
//	memcpy(text,0,20);
//	sprintf(text, "%d,", mems_data->magn_y);
//	uart_write(text, 0, UART_NYX, 50);
//	memcpy(text,0,20);
//	sprintf(text, "%d\r\n", mems_data->magn_z);
//	uart_write(text, 0, UART_NYX, 50);
//	memcpy(text,0,20);
//}


