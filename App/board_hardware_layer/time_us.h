// Cycle-accurate time from DWT->CYCCNT (168 MHz). Wraps every ~25.5 s: only use uint32 differences.
#pragma once

#include <stdint.h>
#include "main.h"

#define TIME_CYCLES_PER_US 168u

void time_init(void);

static inline uint32_t time_cycles(void)
{
    return DWT->CYCCNT;
}

static inline uint32_t time_cycles_to_us(uint32_t cycles)
{
    return cycles / TIME_CYCLES_PER_US;
}
