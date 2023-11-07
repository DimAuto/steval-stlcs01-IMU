/**
 * @file Fusion.h
 * @author Seb Madgwick
 * @brief Main header file for the Fusion library.  This is the only file that
 * needs to be included when using the library.
 */

#ifndef FUSION_H
#define FUSION_H

//------------------------------------------------------------------------------
// Includes

#ifdef __cplusplus
extern "C" {
#endif

#include "FusionAhrs.h"
#include "FusionAxes.h"
#include "FusionCalibration.h"
#include "FusionCompass.h"
#include "FusionConvention.h"
#include "FusionMath.h"
#include "FusionOffset.h"
#include "../lsm6_gyro.h"

#ifdef __cplusplus
}
#endif

void FusionCalcHeading(mems_data_t *memsData, FusionEuler *output_angles);

void setGyroOffset(FusionVector values);

void setMagnCoeff(FusionVector hardiron, FusionMatrix softiron);

void setAccCoeff(FusionVector acc_offset, FusionMatrix acc_misalign);

void SetMagnCalibratingFlag(bool value);

void setMagnCalibratedFlag(bool value);

void FusionReset(void);

double get_magn_vector_magnitude(void);

int get_magn_transient_field(void);

#endif
//------------------------------------------------------------------------------
// End of file
