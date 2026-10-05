#ifndef INC_SENSOR_IMU_H_
#define INC_SENSOR_IMU_H_

#include "main.h"

extern float pitch;
extern float roll;
extern float gx_dps, gy_dps, gz_dps;
extern uint8_t bmi323_chip_id;

void IMU_Init_And_Calibrate(void);
void IMU_Read_And_Calculate_Angles(void);
#endif /* INC_SENSOR_IMU_H_ */
