#include <m8c.h>
#include "PSoCAPI.h"
#include "bldc.h"
#include "PAS.h"
#include "sense.h"
#include "limit.h"
#include "inputs.h"
#include "comm.h"
#include "config.h"

// Battery level, only ever moves down the list: each step plays its cue once.
// Full > VBAT_WARN_MV >= mid > Cfg.wVbatMin_mV >= empty
enum { BAT_BOOT, BAT_FULL, BAT_MID, BAT_EMPTY };

static void cueFull(void)
{
	BLDC_Beep(BEEP_HZ(784),  BEEP_TICKS * 3 / 2); // 3/8 note G5
    BLDC_Beep(BEEP_HZ(880),  BEEP_TICKS / 2);     // 1/8 note A5
    BLDC_Beep(BEEP_HZ(784),  BEEP_TICKS);         // 1/4 note G5
    BLDC_Beep(BEEP_HZ(659),  BEEP_TICKS);         // 1/4 note E5
}

static void cueMid(void)
{
    BLDC_Beep(BEEP_HZ(1500), BEEP_TICKS);
    BLDC_Beep(BEEP_HZ(1000), BEEP_TICKS);
}

static void cueEmpty(void)
{
    cueMid();
    cueMid();
}

void main(void)
{
    uint8_t i, half;
    static uint8_t bBat;

    BLDC_Init();
    Comm_Init();

    half = BLDC_bGetTick() & 0x80;
    while (1) {
        Comm_Poll();
        if (Sense_bUpdateErpm()) Limit_Update();
        if (Sense_bUpdate()) {
            i = (Live.wVbat_mV > VBAT_WARN_MV) ? BAT_FULL
              : (Live.wVbat_mV > Cfg.wVbatMin_mV) ? BAT_MID : BAT_EMPTY;
            if (i > bBat) {
                if (bBat == BAT_BOOT) Sense_Zero();
                if (i == BAT_FULL) cueFull();
                else if (i == BAT_MID) cueMid();
                else cueEmpty();
#if AUTO_ENABLE
                if (bBat == BAT_BOOT) BLDC_Enable(1);
#endif
                bBat = i;
            }
            Limit_Update();
        }
        PAS_Update();
        Inputs_Update();
        i = BLDC_bGetTick() & 0x80;
        if (i != half) {                // 4 Hz
            half = i;
            Comm_SendStatus();
        }
    }
}
