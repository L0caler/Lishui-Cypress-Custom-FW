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
// Full > VBAT_WARN1_MV >= warn1 > VBAT_WARN2_MV >= warn2 > Cfg.wVbatMin_mV >= empty
enum { BAT_BOOT, BAT_FULL, BAT_WARN1, BAT_WARN2, BAT_EMPTY };

// Brake input: pull-up, the sensor pulls it low while the lever is pulled.
#define BRAKE_PULLED()  (!(PRT1DR & BRAKE_CONNECTOR_MASK))

static void cueFull(void)
{
	BLDC_Beep(BEEP_HZ(784),  BEEP_TICKS * 3 / 2); // 3/8 note G5
    BLDC_Beep(BEEP_HZ(880),  BEEP_TICKS / 2);     // 1/8 note A5
    BLDC_Beep(BEEP_HZ(784),  BEEP_TICKS);         // 1/4 note G5
    BLDC_Beep(BEEP_HZ(659),  BEEP_TICKS);         // 1/4 note E5
}

// n chirp pairs: 1 = WARN1, 2 = WARN2, 3 = empty
static void cueWarn(uint8_t n)
{
    do {
        BLDC_Beep(BEEP_HZ(1500), BEEP_TICKS);
        BLDC_Beep(BEEP_HZ(1000), BEEP_TICKS);
    } while (--n);
}

// Brake pulled at boot: fast rising sweep, twice
static void cueBrake(void)
{
    uint8_t n;
    for (n = 2; n; n--) {
        BLDC_Beep(BEEP_HZ(1000), BEEP_TICKS / 4);
        BLDC_Beep(BEEP_HZ(1500), BEEP_TICKS / 4);
        BLDC_Beep(BEEP_HZ(2000), BEEP_TICKS / 4);
        BLDC_Beep(BEEP_HZ(2500), BEEP_TICKS / 4);
    }
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
            i = (Live.wVbat_mV > VBAT_WARN1_MV) ? BAT_FULL
              : (Live.wVbat_mV > VBAT_WARN2_MV) ? BAT_WARN1
              : (Live.wVbat_mV > Cfg.wVbatMin_mV) ? BAT_WARN2 : BAT_EMPTY;
            if (i > bBat) {
                if (bBat == BAT_BOOT) Sense_Zero();
                if (bBat == BAT_BOOT && BRAKE_PULLED()) cueBrake();  // replaces the boot cue
                else if (i == BAT_FULL) cueFull();
                else cueWarn(i - BAT_FULL);
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
