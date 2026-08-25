#ifndef CALIBRATION_STORE_H
#define CALIBRATION_STORE_H

#include "main.h"

#define BNO055_CALIBRATION_PROFILE_SIZE 22U

typedef struct
{
  uint8_t data[BNO055_CALIBRATION_PROFILE_SIZE];
  uint8_t calibration_status;
} BNO055CalibrationProfile;

void CalibrationStore_Init(void);
uint8_t CalibrationStore_IsPresent(void);
uint32_t CalibrationStore_GetAddress(void);
uint8_t CalibrationStore_Load(BNO055CalibrationProfile *profile);
uint8_t CalibrationStore_Save(const BNO055CalibrationProfile *profile);
uint8_t CalibrationStore_Erase(void);

#endif
