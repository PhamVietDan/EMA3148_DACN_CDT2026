// Cycle-accurate time from DWT->CYCCNT (168 MHz).
#include "board_hardware_layer/time_us.h"

void time_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
