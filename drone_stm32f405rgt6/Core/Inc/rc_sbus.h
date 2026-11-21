#ifndef INC_RC_SBUS_H_
#define INC_RC_SBUS_H_

#include "main.h"

extern uint16_t rc_channel[16];
extern uint16_t throttle_pwm;
extern uint8_t sbus_byte;

long map(long x, long in_min, long in_max, long out_min, long out_max);
void SBUS_Init(void);
void SBUS_Parse(void);

#endif /* INC_RC_SBUS_H_ */
