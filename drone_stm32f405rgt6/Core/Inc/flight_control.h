/*
 * flight_control.h
 *
 *  Created on: Aug 13, 2026
 *      Author: ADMIN
 */

#ifndef INC_FLIGHT_CONTROL_H_
#define INC_FLIGHT_CONTROL_H_

#include "main.h"

// Cho phep view qua Live Expressions
extern float Kp_pitch, Ki_pitch, Kd_pitch;
extern float Kp_roll, Ki_roll, Kd_roll;
extern float Kp_yaw, Ki_yaw, Kd_yaw;
extern int16_t out_m1, out_m2, out_m3, out_m4;

void Motor_Init_And_Calibrate_By_RC(void);
void FlightControl_Update_PID(void);

#endif /* INC_FLIGHT_CONTROL_H_ */
