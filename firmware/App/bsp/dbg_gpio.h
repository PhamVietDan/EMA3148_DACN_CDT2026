// Debug pins for logic-analyzer timing. Label pins DBG1..DBG3 in CubeMX; unlabeled pins compile to nothing.
#pragma once

#include "main.h"

// BSRR writes only: an ODR read-modify-write could undo a pin change made by an ISR on the same port.
#define DBG_PIN_TOGGLE(port, pin) \
    ((port)->BSRR = ((port)->ODR & (pin)) ? ((uint32_t)(pin) << 16) : (uint32_t)(pin))

#ifdef DBG1_Pin
#define DBG1_HIGH()   (DBG1_GPIO_Port->BSRR = DBG1_Pin)
#define DBG1_LOW()    (DBG1_GPIO_Port->BSRR = (uint32_t)DBG1_Pin << 16)
#else
#define DBG1_HIGH()   ((void)0)
#define DBG1_LOW()    ((void)0)
#endif

#ifdef DBG2_Pin
#define DBG2_TOGGLE() DBG_PIN_TOGGLE(DBG2_GPIO_Port, DBG2_Pin)
#else
#define DBG2_TOGGLE() ((void)0)
#endif

#ifdef DBG3_Pin
#define DBG3_TOGGLE() DBG_PIN_TOGGLE(DBG3_GPIO_Port, DBG3_Pin)
#else
#define DBG3_TOGGLE() ((void)0)
#endif
