/*
 * sensor_gps_mag.h
 *
 *  Created on: Aug 13, 2026
 *      Author: ADMIN
 */

#ifndef INC_SENSOR_GPS_MAG_H_
#define INC_SENSOR_GPS_MAG_H_

#include "main.h"

extern float gps_lat, gps_lon;
extern int gps_sats;
extern float heading;
extern volatile uint8_t gps_byte;
extern int16_t mag_x;
extern int16_t mag_y;
extern int16_t mag_z;

void GPS_Init(void);
void GPS_Parse_Interrupt(void);
void GPS_Process_Sentence(void);

void Mag_Init(void);
void Mag_Read_Heading(void);

#endif /* INC_SENSOR_GPS_MAG_H_ */
