#include <m8c.h>
#include "PSoCAPI.h"
#include "bldc.h"
#include "PAS.h"
#include "config.h"
#include "inputs.h"

static uint16_t wThrottleAcc;      // << THROTTLE_EMA_SHIFT
static uint8_t bBrake, bBrakeCount;

static uint8_t throttleToRel(uint8_t t)
{
    // static: in one expression with a (uint8_t) cast, ImageCraft narrowed
    // the product to 8 bit before the divide (rel stuck at ~15)
    static uint16_t w;

    if (t <= THROTTLE_MIN) return 0;
    if (t >= THROTTLE_MAX) return 255;
    w = (uint16_t)(t - THROTTLE_MIN) * 255;
    w /= (uint8_t)(THROTTLE_MAX - THROTTLE_MIN);
    return (uint8_t)w;
}

void Inputs_Update(void)
{
    static uint8_t raw, pin;

    if (!BLDC_bGetThrottle(&raw)) return;

    wThrottleAcc = wThrottleAcc - (wThrottleAcc >> THROTTLE_EMA_SHIFT) + raw;

    pin = (PRT1DR & BRAKE_CONNECTOR_MASK) ? 1 : 0;
    if (pin == bBrake) {
        bBrakeCount = 0;
    } else if (++bBrakeCount >= BRAKE_DEBOUNCE) {
        bBrakeCount = 0;
        bBrake = pin;
    }

    if (Cfg.bExternalRel) return;
    if (PAS_bIsActive() && bBrake)
        BLDC_SetCurrentRel(throttleToRel(Inputs_bThrottle()));
    else
        BLDC_SetCurrentRel(0);
}

uint8_t Inputs_bThrottle(void) { return (uint8_t)(wThrottleAcc >> THROTTLE_EMA_SHIFT); }
uint8_t Inputs_bBrake(void)    { return bBrake; }
