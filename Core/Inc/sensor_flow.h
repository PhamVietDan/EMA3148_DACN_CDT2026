/*
 * sensor_flow.h
 *
 *  Created on: Aug 13, 2026
 *      Author: ADMIN
 */

#ifndef INC_SENSOR_FLOW_H_
#define INC_SENSOR_FLOW_H_

#include "main.h"

extern float tof_distance;
extern float flow_vel_x, flow_vel_y;

void Flow_DMA_Init(void);
void Flow_Process_Data(void);

#endif /* INC_SENSOR_FLOW_H_ */
