/*
 * sensor_flow.c
 *
 *  Created on: Aug 13, 2026
 *      Author: ADMIN
 */
#include "sensor_flow.h"
#include "usart.h"
#include "dma.h"

static uint8_t flow_dma_buf[128];
static uint16_t flow_read_ptr = 0;

static uint8_t msp_state = 0, msp_cmd = 0, msp_len = 0, msp_count = 0;
static uint8_t msp_payload[32];

float tof_distance = 0.0f;
float flow_vel_x = 0.0f;
float flow_vel_y = 0.0f;

void Flow_DMA_Init(void) {
    HAL_UART_Receive_DMA(&huart2, flow_dma_buf, 128);
}

void Flow_Process_Data(void) {
    uint16_t flow_write_ptr = 128 - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    while (flow_read_ptr != flow_write_ptr) {
        uint8_t flow_byte = flow_dma_buf[flow_read_ptr++];
        if (flow_read_ptr >= 128) flow_read_ptr = 0;

        switch (msp_state) {
            case 0: if (flow_byte == '$') msp_state = 1; break;
            case 1: if (flow_byte == 'X') msp_state = 2; else msp_state = 0; break;
            case 2: if (flow_byte == '<') msp_state = 3; else msp_state = 0; break;
            case 3: msp_state = 4; break;
            case 4: msp_cmd = flow_byte; msp_state = 5; break;
            case 5: msp_state = 6; break;
            case 6: msp_len = flow_byte; msp_state = 7; break;
            case 7: msp_state = 8; msp_count = 0; break;
            case 8:
                msp_payload[msp_count++] = flow_byte;
                if (msp_count >= msp_len) msp_state = 9;
                break;
            case 9:
                if (msp_cmd == 1 && msp_len == 5) {
                    int32_t dist_mm = (msp_payload[4] << 24) | (msp_payload[3] << 16) | (msp_payload[2] << 8) | msp_payload[1];
                    if (dist_mm > 0 && dist_mm < 12000) tof_distance = (float)dist_mm / 1000.0f;
                }
                else if (msp_cmd == 2 && msp_len == 9) {
                    int32_t mot_x = (msp_payload[4] << 24) | (msp_payload[3] << 16) | (msp_payload[2] << 8) | msp_payload[1];
                    int32_t mot_y = (msp_payload[8] << 24) | (msp_payload[7] << 16) | (msp_payload[6] << 8) | msp_payload[5];
                    flow_vel_x = (float)mot_x;
                    flow_vel_y = (float)mot_y;
                }
                msp_state = 0;
                break;
        }
    }
}

