// sense.h - live values in real units
//
// Who writes what:
//   ISRs (bldc.c)      raw sums, private; handed over as a tRaw snapshot
//   Sense_bUpdate()    Live, from the snapshot, main loop only
// Live is written and read in the main loop only: no locking needed.
#ifndef SENSE_H
#define SENSE_H

#include <m8c.h>
#include "types.h"
#include "config.h"

// mA per count x16, Q8
#define CS_MA_PER_COUNT_Q8  ((uint16_t)(62500.0 * VDD_MV * (CS_R_SERIES_OHM + CS_R_PULLUP_OHM) \
    / ((double)CS_PGA_GAIN * CS_R_PULLUP_OHM * SHUNT_UOHM) + 0.5))

typedef struct {
    int16_t iIbat_mA;// battery current, signed
    uint16_t wVbat_mV;
    uint16_t wPbat_W;
    uint8_t bDuty;// actual, 0..255
    uint16_t wErpm;
    uint16_t wIph_mA;// Ibat * 256 / duty
    uint16_t wIraw;// raw, for calibration: counts x16
    uint16_t wIzero;// wIraw at the last g, gates off
    uint16_t wVraw;
} tLive;

extern tLive Live;

uint8_t Sense_bUpdate(void);   	// 1 = new values (every 0.5 s)
void Sense_Zero(void);      	// motor stopped: wIzero = wIraw

#endif
