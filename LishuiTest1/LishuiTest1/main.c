#include <m8c.h>
#include "PSoCAPI.h"
#include "bldc.h"
#include "PAS.h"
#include "sense.h"
#include "limit.h"
#include "inputs.h"
#include "comm.h"

void main(void)
{
    uint8_t i, half;

    BLDC_Init();
    Comm_Init();

    half = BLDC_bGetTick() & 0x80;
    while (1) {
        Comm_Poll();
        if (Sense_bUpdate()) Limit_Update();
        PAS_Update();
        Inputs_Update();
        i = BLDC_bGetTick() & 0x80;
        if (i != half) {                // 4 Hz
            half = i;
            Comm_SendStatus();
        }
    }
}
