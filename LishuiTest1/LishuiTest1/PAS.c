// PAS.c - pedal assist sensor
//
// Copyright (c) 2026 Leonard Kazemi.
#include <m8c.h>
#include "PAS.h"

#define TICKS_PER_MIN   30720U  // 512 Hz
#define PERIOD_MIN      (TICKS_PER_MIN / (PAS_RPM_MAX * PAS_MAGNETS))
#define PERIOD_MAX      (TICKS_PER_MIN / (PAS_RPM_MIN * PAS_MAGNETS))

#if PERIOD_MAX > 254
#error PAS_RPM_MIN too low for 8 bit tick counters
#endif

uint8_t PAS_bLevel, PAS_bFilt, PAS_bSinceRise, PAS_bSinceFall = 255;
uint8_t PAS_bPeriod, PAS_bHigh, PAS_bNew;

static uint8_t bCount;
static uint8_t bLast = PERIOD_MAX;
static uint16_t wRpm;

void PAS_Update(void)
{
    uint8_t p, h, fwd;

    if (!PAS_bNew) {
        h = PAS_bSinceFall;
        if (bCount && (h >= PERIOD_MAX || h > (uint16_t)bLast * PAS_DROP)) {
            bCount = 0;
            wRpm = 0;
        }
        return;
    }

    M8C_DisableGInt;
    p = PAS_bPeriod;
    h = PAS_bHigh;
    PAS_bNew = 0;
    M8C_EnableGInt;

    bLast = p;
    if (p <= PERIOD_MIN || p >= PERIOD_MAX) return;

#if PAS_DIR_DETECT
    fwd = ((uint16_t)h * 100 < (uint16_t)PAS_DUTY_PCT * p) != PAS_DIR_REVERSE;
#else
    fwd = 1;
#endif
    if (!fwd) {
        if (bCount) bCount--;
        if (!bCount) wRpm = 0;
        return;
    }
    if (bCount < PAS_TRIGGER) bCount++;
    wRpm = TICKS_PER_MIN / ((uint16_t)PAS_MAGNETS * p);
}

uint8_t PAS_bIsActive(void) { return bCount >= PAS_TRIGGER; }
uint16_t PAS_wGetRpm(void)   { return wRpm; }
