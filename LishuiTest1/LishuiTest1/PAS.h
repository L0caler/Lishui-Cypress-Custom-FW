// PAS.h - pedal assist sensor, sampled by the 512 Hz tick
//
// Copyright (c) 2026 Leonard Kazemi.
#ifndef PAS_H
#define PAS_H

#include <m8c.h>
#include "types.h"
#include "config.h"

// TODO: pin.  Not port 2: reading PRT2DR re-arms the hall ChangeFromRead.
#define PAS_DR          PRT0DR
#define PAS_MASK        0x80

extern uint8_t PAS_bLevel, PAS_bFilt, PAS_bSinceRise, PAS_bSinceFall;
extern uint8_t PAS_bPeriod, PAS_bHigh, PAS_bNew;

// Tick ISR only (no call).  Counters saturate at 255 ticks.
#define PAS_TICK() do {                                                 \
    if (PAS_bSinceRise != 255) PAS_bSinceRise++;                        \
    if (PAS_bSinceFall != 255) PAS_bSinceFall++;                        \
    if (!(PAS_DR & PAS_MASK) == !PAS_bLevel) {                          \
        PAS_bFilt = 0;                                                  \
    } else if (++PAS_bFilt >= PAS_FILTER) {                             \
        PAS_bFilt = 0;                                                  \
        PAS_bLevel ^= 1;                                                \
        if (PAS_bLevel) {                                               \
            PAS_bSinceRise = 0;                                         \
        } else {                                                        \
            PAS_bPeriod = PAS_bSinceFall;                               \
            PAS_bHigh = PAS_bSinceRise;                                 \
            PAS_bSinceFall = 0;                                         \
            PAS_bNew = 1;                                               \
        }                                                               \
    }                                                                   \
} while (0)

void PAS_Update(void);         // main loop
uint8_t PAS_bIsActive(void);
uint16_t PAS_wGetRpm(void);

#endif
