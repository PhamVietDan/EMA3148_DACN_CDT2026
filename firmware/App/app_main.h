// Application entry points, called from USER CODE blocks in Core/Src/main.c.
#pragma once

void app_init(void);                       // after MX_*_Init, before osKernelStart
void app_telem_task(void const *argument); // body of the CubeMX-generated task (USB already initialised)
