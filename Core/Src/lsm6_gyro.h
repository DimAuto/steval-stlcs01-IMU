/*
 * lsm6_gyro.h
 *
 *  Created on: May 25, 2023
 *      Author: dkalaitzakis
 */

#ifndef SRC_LSM6_GYRO_H_
#define SRC_LSM6_GYRO_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define LSM6		(0x6A << 1)
#define LIS3_MAGN	(0x1E << 1)

//#define GYRO_TS
#define GYRO_TIMESTAMP_LSB_USEC 25

//LSM6DRX
#define INT1_CTRL   0x0D
#define INT2_CTRL   0x0E
//#define CHIP_ADDR   0x0F
#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL3_C     0x12
#define CTRL4_C     0x13
#define CTRL5_C     0x14
#define CTRL7_G     0x16
#define	CTRL10_C    0x19
#define STATUS_REG  0x1E
#define OUTX_L_G    0x22
#define OUTX_H_G    0x23
#define OUTY_L_G    0x24
#define OUTY_H_G    0x25
#define OUTZ_L_G    0x26
#define OUTZ_H_G    0x27
#define OUTX_L_A    0x28
#define OUTX_H_A    0x29
#define OUTY_L_A    0x2A
#define OUTY_H_A    0x2B
#define OUTZ_L_A    0x2C
#define OUTZ_H_A    0x2D
#define EMB_FUNC_STATUS_MAINPAGE    0x35
#define INTERNAL_FREQ_FINE          0x63
#define TIMESTAMP0	0x40
#define TIMESTAMP1	0x41
#define TIMESTAMP2	0x42
#define TIMESTAMP3	0x43
#define WHO_I_AM	0x0F

//LSM303
#define STATUS_REG_AUX_A        0x07
#define OUT_TEMP_L_A            0x0C
#define OUT_TEMP_H_A            0x0D
#define INT_COUNTER_REG_A       0x0E
#define WHO_AM_I_A              0x0F
#define TEMP_CFG_REG_A          0x1F
//ACCELEROMETER CONTROL REGISTERS
#define CTRL_REG1_A             0x20
#define CTRL_REG2_A             0x21
#define CTRL_REG3_A             0x22
#define CTRL_REG4_A             0x23
#define CTRL_REG5_A             0x24
#define CTRL_REG6_A             0x25

#define REFERENCE               0x26
//ACCELEROMETER STATUS REGISTER
#define STATUS_REG_A            0x27
//ACCELEROMETER OUTPUT REGISTERS
#define OUT_X_L_A               0x28
#define OUT_X_H_A               0x29
#define OUT_Y_L_A               0x2A
#define OUT_Y_H_A               0x2B
#define OUT_Z_L_A               0x2C
#define OUT_Z_H_A               0x2D
//FIFO REGISTERS
#define FIFO_CTRL_REG_A         0x2E
#define FIFO_SRC_REG_A          0x2F

//MAGNETOMETER HARD-IRON REGISTERS
#define OFFSET_X_REG_L_M        0x45
#define OFFSET_X_REG_H_M        0x46
#define OFFSET_Y_REG_L_M        0x47
#define OFFSET_Y_REG_H_M        0x48
#define OFFSET_Z_REG_L_M        0x49
#define OFFSET_Z_REG_H_M        0x4A
//MAGNETOMETER CONFIGURATION REGISTERS
#define CFG_REG_A_M             0x60
#define CFG_REG_B_M             0x61
#define CFG_REG_C_M             0x62
//MAGNETOMETER INTERRUPT CONFIGURATION REGISTERS
#define INT_CRTL_REG_M          0x63
#define INT_SOURCE_REG_M        0x64
#define INT_THS_L_REG_M         0x65
#define INT_THS_H_REG_M         0x66

#define STATUS_REG_M            0x67
#define WHO_AM_I_M              0x4F
//MAGNETOMETER OYTPUT REGISTERS
#define OUTX_L_REG_M            0x68
#define OUTX_H_REG_M            0x69
#define OUTY_L_REG_M            0x6A
#define OUTY_H_REG_M            0x6B
#define OUTZ_L_REG_M            0x6C
#define OUTZ_H_REG_M            0x6D

#define OUT_AUTO_I              0x80

#define GYRO_OFFSET_ADDR	0x0807B800
#define MAGN_HIRON_ADDR		0x0807C800
#define MAGN_SIRON_ADDR		0x0807D800
#define ACC_MATRIX_ADDR		0x0807A800
#define ACC_VECTOR_ADDR		0x08079800
#define MAGN_CALIB_ADDR		0x08078800
//#define DUMMY_WRITE_ADDR	0x0807B040


#define GYRO_CALIB_SAMPLES	700
#define MAGN_CALIB_SAMPLES	500
#define ACC_CALIB_SAMPLES	300
#define MAGN_ERROR_COEFF_SAMPLES 300


typedef struct{
	float gyro_x;
	float gyro_y;
	float gyro_z;
}gyro_data_t;


typedef struct{
	float acc_x;
	float acc_y;
	float acc_z;
}acc_data_t;

typedef struct{
    float magn_x;
    float magn_y;
    float magn_z;
}magn_data_t;

typedef struct{
	gyro_data_t gyro;
	acc_data_t acc;
	magn_data_t magn;
    uint32_t	timestamp;
}mems_data_t;

uint8_t whoIam_lsm6(void);

uint8_t whoIam_lsm303(void);

HAL_StatusTypeDef lsm6_bus_init(void);

HAL_StatusTypeDef gyro_init(void);

HAL_StatusTypeDef magn_init(void);

HAL_StatusTypeDef gyro_read(mems_data_t *mems_data);

HAL_StatusTypeDef lsm6_acc_init(void);

HAL_StatusTypeDef lsm6_acc_read(mems_data_t *mems_data);

HAL_StatusTypeDef lsm303_acc_read(mems_data_t *mems_data);

HAL_StatusTypeDef lis3_magn_read(mems_data_t *mems_data);

void tick_gyro(mems_data_t *mems_data);

uint8_t gyro_offset_calculation(mems_data_t *mems_data);

uint8_t magneto_sample(mems_data_t *mems_data);

uint8_t magnetoSetErrorCoeff(mems_data_t *mems_data);

#endif /* SRC_LSM6_GYRO_H_ */
