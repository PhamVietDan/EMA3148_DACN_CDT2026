// RAM black box (PLAN M6): fixed-size records at 50 Hz in CCM RAM, newest ~30 s kept, dumped over USB.
#include "control_logic/blackbox.h"

#include <stdio.h>

// CCM is not reachable by DMA, which is fine: only task_telem touches this buffer. The NOLOAD section
// (linker script) keeps 54 KB of zeros out of the flash image.
#if defined(__arm__)
#define CCM_BSS __attribute__((section(".ccm_bss")))
#else
#define CCM_BSS
#endif

static bb_rec_t buf[BB_RECORDS] CCM_BSS;
static size_t head;   // next slot to write
static size_t count;

void blackbox_reset(void)
{
    head = 0;
    count = 0;
}

void blackbox_add(const bb_rec_t *r)
{
    buf[head] = *r;
    head = (head + 1u) % BB_RECORDS;
    if (count < BB_RECORDS) {
        count++;
    }
}

size_t blackbox_count(void)
{
    return count;
}

const bb_rec_t *blackbox_get(size_t i)
{
    if (i >= count) {
        return NULL;
    }
    size_t oldest = (head + BB_RECORDS - count) % BB_RECORDS;
    return &buf[(oldest + i) % BB_RECORDS];
}

const char *blackbox_header(void)
{
    return "L,t_ms,arm,fs,mode,rc_valid,roll_cdeg,pitch_cdeg,yaw_cdeg,p_ddps,q_ddps,r_ddps,"
           "roll_sp_cdeg,pitch_sp_cdeg,yawr_sp_ddps,thr_milli,m1_us,m2_us,m3_us,m4_us\r\n";
}

// Raw integer fields: the GCS rescales them, and integer printf needs no float support.
size_t blackbox_format(const bb_rec_t *r, char *out, size_t size)
{
    int n = snprintf(out, size, "L,%lu,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u\r\n",
                     (unsigned long)r->t_ms, r->arm, r->fs, r->mode, r->rc_valid, r->roll, r->pitch, r->yaw,
                     r->p, r->q, r->r, r->roll_sp, r->pitch_sp, r->yawr_sp, r->thr, r->m_us[0], r->m_us[1],
                     r->m_us[2], r->m_us[3]);
    return (n > 0 && (size_t)n < size) ? (size_t)n : 0u;
}
