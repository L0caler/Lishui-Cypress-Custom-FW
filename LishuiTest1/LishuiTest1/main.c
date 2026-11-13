#include <m8c.h>
#include "PSoCAPI.h"
#include "bldc.h"
#include "PAS.h"
#include "sense.h"
#include "limit.h"
#include "inputs.h"
#include "comm.h"
#include "config.h"

void main(void)
{
    uint8_t i, half;
#if AUTO_ENABLE
    static uint8_t bInitialized = 0;
#endif

    BLDC_Init();
    Comm_Init();

    half = BLDC_bGetTick() & 0x80;
    while (1) {
        Comm_Poll();
        if (Sense_bUpdateErpm()) Limit_Update();
#if AUTO_ENABLE
        if (Sense_bUpdate()) {
            if (!bInitialized) {
                Sense_Zero();
                Limit_Update();
                BLDC_Enable(1);
                bInitialized = 1;
            } else {
                Limit_Update();
            }
        }
#else
        if (Sense_bUpdate()) Limit_Update();
#endif
        PAS_Update();
        Inputs_Update();
        i = BLDC_bGetTick() & 0x80;
        if (i != half) {                // 4 Hz
            half = i;
            Comm_SendStatus();
        }
    }
}
