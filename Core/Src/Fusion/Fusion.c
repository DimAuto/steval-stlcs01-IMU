/*
 * Fusion.c
 *
 *  Created on: Jun 19, 2023
 *      Author: dkalaitzakis
 */

#include "Fusion.h"
#include "../uart.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include "../helpers.h"
#include <math.h>
#include "cmsis_os2.h"
#include "../lsm6_gyro.h"
#include "../../Inc/main.h"
#include "../flash_memory.h"

#define SAMPLE_PERIOD (0.034f)
#define SAMPLE_RATE (200)

const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
static FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
static FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
static FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
static FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
static FusionMatrix softIronMatrix = {0.0f, 0.0f, 0.0f, 0.0f, 0.0, 0.0f, 0.0f, 0.0f, 0.0f};
static FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

static uint32_t prv_tick = 0;

static clock_t timestamp = 0;
static clock_t previousTimestamp = 0;
static uint32_t update_duration = 0;

static bool magn_cal_finished = false;

FusionAhrs ahrs;
FusionOffset offset;

void setGyroOffset(FusionVector values){
	gyroscopeOffset.array[0] = values.array[0];
	gyroscopeOffset.array[1] = values.array[1];
	gyroscopeOffset.array[2] = values.array[2];
}

void setMagnCoeff(FusionVector hardiron, FusionMatrix softiron){
	for (uint8_t i=0; i<3; i++){
		hardIronOffset.array[i] = hardiron.array[i];
	}
	for (uint8_t i=0; i<3; i++){
		for (uint8_t j=0; j<3; j++){
			softIronMatrix.array[i][j] = softiron.array[i][j];
		}
	}
	uint8_t text[40] = {0};
	sprintf(text, "Hardiron: %f  %f  %f\r\n,", hardIronOffset.array[0], hardIronOffset.array[1], hardIronOffset.array[2]);
	uart_write_debug(text, 20);

}

void setAccCoeff(FusionVector acc_offset, FusionMatrix acc_misalign){
	for (uint8_t i=0; i<3; i++){
		accelerometerOffset.array[i] = acc_offset.array[i];
	}
	for (uint8_t i=0; i<3; i++){
		for (uint8_t j=0; j<3; j++){
			accelerometerMisalignment.array[i][j] = acc_misalign.array[i][j];
		}
	}
	uint8_t text[40] = {0};
	sprintf(text, "Acc-Offset: %f  %f  %f\r\n,", accelerometerOffset.array[0], accelerometerOffset.array[1], accelerometerOffset.array[2]);
	uart_write_debug(text, 20);
}


/* Initialize Fusion algorithm. */
void FusionInit(void){
	FusionVector GyroVector;
	FusionVector HIron_vector;
	FusionMatrix SIron_matrix;
	FusionVector acc_vector;
	FusionMatrix acc_matrix;
	float magn_calib;
	FusionOffsetInitialise(&offset, SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);
	const FusionAhrsSettings settings = {
	            .convention = FusionConventionNed,
	            .gain = 0.4f,
	            .gyroscopeRange = 2000.0f,
	            .accelerationRejection = 10.0f,
	            .magneticRejection = 3.0f,
	            .recoveryTriggerPeriod = 7 * SAMPLE_RATE,
	};
	FusionAhrsSetSettings(&ahrs, &settings);
	Flash_Read_Vector(GYRO_OFFSET_ADDR, &GyroVector);
	setGyroOffset(GyroVector);
	Flash_Read_Matrix(MAGN_SIRON_ADDR, &SIron_matrix);
	Flash_Read_Vector(MAGN_HIRON_ADDR, &HIron_vector);
	setMagnCoeff(HIron_vector, SIron_matrix);
	Flash_Read_Matrix(ACC_MATRIX_ADDR, &acc_matrix);
	Flash_Read_Vector(ACC_VECTOR_ADDR, &acc_vector);
	setAccCoeff(acc_vector, acc_matrix);
	ahrs.magnVectorLengthInit = Flash_Read_Double(MAGN_CALIB_ADDR);
}

/* Calculate angle based only on Accelerometer and gyroscope.*/
void FusionCalcAngle(mems_data_t *memsData, FusionEuler *output_angles){
	FusionVector gyroscope = {memsData->gyro.gyro_x, memsData->gyro.gyro_y, memsData->gyro.gyro_z};
	const FusionVector accelerometer = {memsData->acc.acc_x, memsData->acc.acc_y, memsData->acc.acc_z};
	gyroscope = FusionVectorSubtract(gyroscope, gyroscopeOffset);

	gyroscope = FusionOffsetUpdate(&offset, gyroscope);
#ifndef GYRO_TS
	float delta = (float)(memsData->timestamp - prv_tick) / 1000.0f;
	prv_tick = memsData->timestamp;
#else
	float delta = (float) ( memsData->timestamp - previousTimestamp) * (float) GYRO_TIMESTAMP_LSB_USEC / (float) 1000000;
	previousTimestamp = memsData->timestamp;
#endif
//	delta += 0.006; //Add a const offset.
	if ((delta >= MEMS_SR_SEC - 7) && (delta <= MEMS_SR_SEC + 7)){
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta);
	}
//
//	uint8_t text[20] = {0};
//	sprintf(text, "%f\r\n,", delta);
//	uart_write_debug(text, 20);

	*output_angles = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	if (output_angles->angle.yaw < 0){
		output_angles->angle.yaw += 360;
	}
	if (output_angles->angle.roll < 0){
		output_angles->angle.roll += 360;
	}
	if (output_angles->angle.pitch < 0){
		output_angles->angle.pitch += 360;
	}
	//	const FusionVect = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
}

/* Calculate heading based on all three sensors.*/
void FusionCalcHeading(mems_data_t *memsData, FusionEuler *output_angles){
	FusionVector gyroscope = {memsData->gyro.gyro_x, memsData->gyro.gyro_y, memsData->gyro.gyro_z};
	FusionVector accelerometer = {memsData->acc.acc_x, memsData->acc.acc_y, memsData->acc.acc_z};
	FusionVector magnetometer = {memsData->magn.magn_x, memsData->magn.magn_y, memsData->magn.magn_z}; // replace this with actual magnetometer data in arbitrary units

	// Apply calibration
	gyroscope = FusionVectorSubtract(gyroscope, gyroscopeOffset);
	accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
	magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

	// Update gyroscope offset correction algorithm
	gyroscope = FusionOffsetUpdate(&offset, gyroscope);

#ifndef GYRO_TS
	float delta = (float)(memsData->timestamp - prv_tick) / 1000.0f;
	prv_tick = memsData->timestamp;
#else
	float delta = (float) ( memsData->timestamp - previousTimestamp) * (float) GYRO_TIMESTAMP_LSB_USEC / (float) 1000000;
	previousTimestamp = memsData->timestamp;
#endif
	// Update gyroscope AHRS algorithm
	if (((delta >= MEMS_SR_SEC - 7) && (delta <= MEMS_SR_SEC + 7)) || (ahrs.calibrating == true)){
		FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta);
	}

	// Print algorithm outputs
	*output_angles = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	if (output_angles->angle.yaw < 0){
		output_angles->angle.yaw += 360;
	}
	if (output_angles->angle.roll < 0){
		output_angles->angle.roll += 360;
	}
	if (output_angles->angle.pitch < 0){
		output_angles->angle.pitch += 360;
	}
//	const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
}


double get_magn_vector_magnitude(void)
{
	return ahrs.magnVectorLength;
}

int get_magn_transient_field(void){
	return ahrs.magnTransientField;
}

void SetMagnCalibratingFlag(bool value){
	ahrs.calibrating = value;
}

void FusionReset(void){
	FusionAhrsReset(&ahrs);
}

void setMagnCalibratedFlag(bool value){
	ahrs.calibrated = value;
}
