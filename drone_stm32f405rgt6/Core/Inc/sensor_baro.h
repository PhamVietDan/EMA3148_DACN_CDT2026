/*
 * sensor_baro.h
 *
 *  Created on: Aug 13, 2026
 *      Author: ADMIN
 */

#ifndef INC_SENSOR_BARO_H_
#define INC_SENSOR_BARO_H_

#include "main.h"

extern float current_altitude;

void Baro_Init(void);
void Baro_Read_Altitude(void);

#endif /* INC_SENSOR_BARO_H_ */
