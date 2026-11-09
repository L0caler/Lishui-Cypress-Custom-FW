#include <m8c.h>
#include "bldc.h"
#include "sense.h"

// mV = wVraw * VBAT_MUL / 256 + VBAT_OFS_MV
#define VBAT_MUL     5333
#define VBAT_OFS_MV  667

tLive Live;

static tRaw stRaw;

uint8_t Sense_bUpdate(void)
{
    static int16_t d;              // static: saves stack
    static uint32_t x;

    if (!BLDC_bGetRaw(&stRaw)) return 0;

    Live.wIraw = stRaw.wI;
    Live.wVraw = stRaw.wV;
    Live.bDuty = BLDC_bGetDuty();

    d = (int16_t)(stRaw.wI - Live.wIzero);
    Live.iIbat_mA = (int16_t)((int32_t)d * CS_MA_PER_COUNT_Q8 / 256);

    x = (((uint32_t)stRaw.wV * VBAT_MUL) >> 8) + VBAT_OFS_MV;
    Live.wVbat_mV = (x > 0xFFFF) ? 0xFFFF : (uint16_t)x;        // > 65.5 V

    if (Live.iIbat_mA > 0) {
        x = (uint32_t)Live.wVbat_mV * (uint16_t)Live.iIbat_mA;
        Live.wPbat_W = (uint16_t)((((x >> 10) * 1074) >> 20));           // / 1e6
        if (Live.bDuty) {
            x = ((uint32_t)Live.iIbat_mA << 8) / Live.bDuty;
            Live.wIph_mA = (x > 0xFFFF) ? 0xFFFF : (uint16_t)x;
        } else {
            Live.wIph_mA = 0;
        }
    } else {
        Live.wPbat_W = 0;
        Live.wIph_mA = 0;
    }

        // 6 edges per e-rev, 0.5 s window: erpm = edges * 20
    Live.wErpm = (stRaw.wEdges > 3276) ? 0xFFFF
               : (stRaw.wEdges << 4) + (stRaw.wEdges << 2);
    return 1;
}

void Sense_Zero(void)
{
    Live.wIzero = Live.wIraw;
}
