// MicoAir MTF-01P frames over MSPv2 (PLAN §6.3).
#include "drivers/mtf01p.h"

static int32_t i32_le(const uint8_t *b)
{
    return (int32_t)((uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24));
}

bool mtf01p_parse_range(const mspv2_frame_t *f, mtf01p_range_t *out)
{
    if (f->cmd != MTF01P_CMD_RANGE || f->size != 5u) {
        return false;
    }
    out->quality = f->payload[0];
    out->distance_mm = i32_le(&f->payload[1]);
    return true;
}

bool mtf01p_parse_flow(const mspv2_frame_t *f, mtf01p_flow_t *out)
{
    if (f->cmd != MTF01P_CMD_FLOW || f->size != 9u) {
        return false;
    }
    out->quality = f->payload[0];
    out->motion_x = i32_le(&f->payload[1]);
    out->motion_y = i32_le(&f->payload[5]);
    return true;
}
