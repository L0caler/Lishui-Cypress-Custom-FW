// bldc.c - 6-step hall-commutated BLDC drive
//
// High side: PWMDB8_1 Phase1 on RO0[3], put on the step's HIN pin by its
// Global Select bit, its low side on inverted RO0[2].  Other low side:
// GPIO, on for the whole step.
// Hall GPIO interrupt commutates, 512 Hz sleep timer ramps the duty.
// The ISRs must not call functions (stack: see stackFree in comm_text.c).
#include <m8c.h>
#include "PSoCAPI.h"
#include "bldc.h"
#include "PAS.h"
#include "limit.h"
#include "wavetables/pure_sine_255_gen.c"

#if (HALL1_MASK != 0x01) || (HALL2_MASK != 0x02) || (HALL3_MASK != 0x04)
#error halls are read as P2[2:0]
#endif

#define HALL_MASK  (HALL1_MASK | HALL2_MASK | HALL3_MASK)
#define NO_STEP    0xFF

#define P1_GATES   (Gate_A_HIN_MASK | Gate_A_NLIN_MASK | Gate_B_HIN_MASK | Gate_B_NLIN_MASK)
#define P2_GATES   (Gate_C_HIN_MASK | Gate_C_NLIN_MASK)

// EG2103: HIN active high, NLIN active low
#define P1_GATES_OFF  (Gate_A_NLIN_MASK | Gate_B_NLIN_MASK)
#define P2_GATES_OFF  (Gate_C_NLIN_MASK)
#define P1_ALO  (P1_GATES_OFF & ~Gate_A_NLIN_MASK)
#define P1_BLO  (P1_GATES_OFF & ~Gate_B_NLIN_MASK)
#define P2_CLO  (P2_GATES_OFF & ~Gate_C_NLIN_MASK)

// Hall HALL3:HALL2:HALL1 -> step, forward = step + 1
static const uint8_t abHallToStep[8] = { NO_STEP, 4, 2, 3, 0, 5, 1, NO_STEP };

// Steps: 0 A+B-  1 A+C-  2 B+C-  3 B+A-  4 C+A-  5 C+B-
static const uint8_t abStepP1[6] = {
    P1_BLO, P1_GATES_OFF, P1_GATES_OFF, P1_ALO, P1_ALO, P1_BLO
};
static const uint8_t abStepP2[6] = {
    P2_GATES_OFF, P2_CLO, P2_CLO, P2_GATES_OFF, P2_GATES_OFF, P2_GATES_OFF
};
// PWM phase: HIN = RO0[3], NLIN = inverted RO0[2] (synchronous rectification,
// only while bSync: forward current.  Otherwise NLIN stays off, no braking)
#define A_PWM  (Gate_A_HIN_MASK | Gate_A_NLIN_MASK)
#define B_PWM  (Gate_B_HIN_MASK | Gate_B_NLIN_MASK)
#define C_PWM  (Gate_C_HIN_MASK | Gate_C_NLIN_MASK)
#define P1_HIN (Gate_A_HIN_MASK | Gate_B_HIN_MASK)
#define P2_HIN (Gate_C_HIN_MASK)
static const uint8_t abStepGS1[6] = { A_PWM, A_PWM, B_PWM, B_PWM, 0, 0 };
static const uint8_t abStepGS2[6] = { 0, 0, 0, 0, C_PWM, C_PWM };

// duty 0..255 -> pulse width 0..199 (d * 25 / 32)
#define DUTY_TO_CMP(d)  ((d) - ((d) >> 2) + ((d) >> 5))

// Sine factor over one step: the driven pair's line voltage, 60..120 deg
// = table angle 43..85 (256 = 360 deg), interpolated from the last step's
// length in ticks.  Quarter table: 64..85 mirrors to 64..43.
// Table = 127 + 128 sin, factor = 2 * table - 255 = 255 sin (221..255).
#define STEP_ANGLE   42
#define SINE_START   43

// Interrupts off.  GATES_STEP only after GATES_OFF.
#define GATES_OFF() do {                                                 \
    PRT1GS &= ~P1_GATES;                                                 \
    PRT2GS &= ~P2_GATES;                                                 \
    Port_1_Shadow = (Port_1_Shadow & ~P1_GATES) | P1_GATES_OFF;          \
    PRT1DR = Port_1_Shadow;                                              \
    Port_2_Shadow = (Port_2_Shadow & ~P2_GATES) | P2_GATES_OFF;          \
    PRT2DR = Port_2_Shadow;                                              \
} while (0)

#define GATES_STEP(s) do {                                               \
    Port_1_Shadow = (Port_1_Shadow & ~P1_GATES) | abStepP1[s];           \
    PRT1DR = Port_1_Shadow;                                              \
    Port_2_Shadow = (Port_2_Shadow & ~P2_GATES) | abStepP2[s];           \
    PRT2DR = Port_2_Shadow;                                              \
    PRT1GS |= bSync ? abStepGS1[s] : (abStepGS1[s] & P1_HIN);            \
    PRT2GS |= bSync ? abStepGS2[s] : (abStepGS2[s] & P2_HIN);            \
} while (0)

uint8_t Port_1_Shadow;
uint8_t Port_2_Shadow;

static uint8_t bEnabled;
static uint8_t bDuty;
static uint8_t bSync;
static uint8_t bHall;              // last read, may be 000 / 111
static uint8_t bHallValid;          // last valid
static uint8_t bFaultTicks;
static uint8_t bHallFault;
static uint8_t abHallMap[8];        // abHallToStep with direction

static volatile uint8_t bTick;

static uint8_t bStepTicks, bStepLen = 255, bStepPos;
static uint16_t wStepAcc;

// ISR sums, private.  stRawOut: last finished 0.5 s window
static uint16_t wIsum, wI16, wVsum;
static uint8_t bIlast;
static tRaw stRawOut;
static uint8_t bRawNew;
static uint8_t bEdges, bEdgesOut, bEdgesNew;    // hall edges per 1/8 s
static uint8_t bThrottle, bThrottleNew;

// Current limit, 32 Hz.  Counts x16.  Output: duty ceiling, the duty
// itself is the integrator.
// Setpoint = min(Ibat max, Iph max * duty / 256): Ibat = Iph * duty.
#define RAMP_PER_SAMPLE  (16 / (RAMP_TICKS_MASK + 1))

// res = m * d / 256, inline (no call in the ISR)
#define MUL8(res, m, d) do {                                            \
    uint8_t d_ = (d), i_;                                                  \
    (res) = 0;                                                          \
    for (i_ = 8; i_; i_--) {                                            \
        if (d_ & 1) (res) += (m);                                       \
        (res) >>= 1;                                                    \
        d_ >>= 1;                                                       \
    }                                                                   \
} while (0)

static tIlim stIlim;
static uint8_t bCeil;
static uint8_t bLimiting;          // BLDC_LIM_...

static volatile uint8_t bIrelTarget = 0;
static uint8_t bIrelSmoothed = 0;

static void rebuildMap(void)
{
    uint8_t h, s;

    for (h = 0; h < 8; h++) {
        s = abHallToStep[h];
        abHallMap[h] = s;
    }
}

static void gatesTo(uint8_t step)
{
    GATES_OFF();
    if (step != NO_STEP) GATES_STEP(step);
}

static void commutate(void)
{
    uint8_t step = abHallMap[bHallValid];

    bHallFault = (step == NO_STEP);
    gatesTo(step);
}

static void delayShort(uint8_t n)
{
    volatile uint8_t i;

    for (i = 0; i < n; i++) {
    }
}

#pragma interrupt_handler BLDC_HallISR
void BLDC_HallISR(void)
{
    uint8_t h;

    h = PRT2DR & HALL_MASK;             // re-arms ChangeFromRead
    if (h == bHall) return;
    bHall = h;

    if (abHallToStep[h] == NO_STEP) return;         // 000 / 111

    if (h != bHallValid) {
        Port_2_Shadow ^= LED_EN_MASK;
        if (bEdges != 255) bEdges++;
        bStepLen = bStepTicks ? bStepTicks : 1;
        bStepTicks = 0;
        bStepPos = 0;
        wStepAcc = 0;
    } else if (!bHallFault) {
        return;
    }
    bHallValid = h;
    bFaultTicks = 0;
    bHallFault = 0;
    if (bEnabled) {
        GATES_OFF();
        if (bDuty > 0) {
            GATES_STEP(abHallMap[h]);
        }
    }
    PRT2DR = Port_2_Shadow;
}

// SAR8 input: current, except tick 7 of 16: throttle, tick 15 of 16: battery voltage
#define SAR_CH(ch)  (SAR8_1_CONTROL_0_REG = (SAR8_1_CONTROL_0_REG & ~0x78) | (ch))
#define SAR_I       SAR8_1_ACB00
#define SAR_V       SAR8_1_P0_6
#define SAR_T       SAR8_1_P0_2

#pragma interrupt_handler BLDC_TickISR
void BLDC_TickISR(void)
{
    uint8_t a = SAR8_1_DATA_LO_REG;    //free running, input set last tick
    uint8_t ph = bTick & 15;

    if (ph == 15) {
        wVsum += a;
        a = bIlast;                 //fill missing current with last measurement
        SAR_CH(SAR_I);
    } else if (ph == 7) {
        bThrottle = a;
        bThrottleNew = 1;
        a = bIlast;
        SAR_CH(SAR_I);
    } else {
        bIlast = a;
        if (ph == 14) SAR_CH(SAR_V);
        else if (ph == 6) SAR_CH(SAR_T);
    }
    wIsum += a;                     //0.5 s
    wI16 += a;                      //31 ms, limit
    if (ph == 15 && bEnabled) {
        uint16_t w, t;
        int16_t e;
        uint16_t wMeas;

        // ramp current factor
        if (bIrelSmoothed < bIrelTarget) {
            if (bIrelTarget - bIrelSmoothed >= IREL_RAMP_UP) bIrelSmoothed += IREL_RAMP_UP;
            else bIrelSmoothed = bIrelTarget;
        } else if (bIrelSmoothed > bIrelTarget) {
            if (bIrelSmoothed - bIrelTarget >= IREL_RAMP_DOWN) bIrelSmoothed -= IREL_RAMP_DOWN;
            else bIrelSmoothed = bIrelTarget;
        }

        //target: phase term needs the duty of now, rest from Limit_Update
        MUL8(w, stIlim.wPh, (bDuty < ILIM_DUTY_MIN) ? ILIM_DUTY_MIN : bDuty);
        bLimiting = BLDC_LIM_PH;
        if (w >= stIlim.wBat) {
            w = stIlim.wBat;
            bLimiting = stIlim.bFlags & (BLDC_LIM_BAT | BLDC_LIM_W);
        }
        if (stIlim.bScale != 255) {
            t = w;
            MUL8(w, t, stIlim.bScale);
        }
        bLimiting |= stIlim.bFlags & (BLDC_LIM_VBAT | BLDC_LIM_ERPM);

        if (bIrelSmoothed == 0) {
            bCeil = 0;
            bSync = 0;
            if (bDuty > 0) {
                bDuty = 0;
                GATES_OFF();
            }
        } else {
            //PI using smoothed current factor: scale target
            {
                uint16_t wTarget;
                MUL8(wTarget, w, bIrelSmoothed);
                wTarget += stIlim.wZero;
                wMeas = (uint16_t)wI16;
                e = (int16_t)wTarget - (int16_t)wMeas;
            }
            //below target: ceiling leads the duty, full ramp unless close.
            //above: duty steps down now, proportional
            if (e >= 0) {
                e = 1 + (e >> ILIM_UP_SHIFT);
                if (e >= RAMP_PER_SAMPLE) bLimiting = 0;
                e += bDuty;
                bCeil = (e > 255) ? 255 : (uint8_t)e;
            } else {
                e = (int16_t)bDuty + (e >> ILIM_DOWN_SHIFT);
                bCeil = (e < 0) ? 0 : (uint8_t)e;
                bDuty = bCeil;
                if (bDuty == 0) GATES_OFF();
            }

            {
                uint8_t sync = (wMeas >= stIlim.wZero
                                + (bSync ? SYNC_OFF_COUNTS : SYNC_ON_COUNTS));
                uint8_t step = abHallMap[bHallValid];

                if (sync != bSync) {
                    bSync = sync;
                    if (bDuty > 0 && !bHallFault && step != NO_STEP) {
                        GATES_OFF();
                        GATES_STEP(step);
                    }
                }
            }
        }
    }
    if (ph == 15) wI16 = 0;

    if (abHallToStep[bHall] == NO_STEP) {
        if (bFaultTicks < HALL_FAULT_TICKS && ++bFaultTicks == HALL_FAULT_TICKS
            && bEnabled) {
            bHallFault = 1;
            GATES_OFF();
        }
    } else {
        bFaultTicks = 0;
    }

    if ((bTick & RAMP_TICKS_MASK) == 0 && bDuty != Cfg.bDutyMax) {
        if (bDuty > Cfg.bDutyMax) {
            bDuty--;
            if (bDuty == 0) GATES_OFF();
        } else if (bIrelSmoothed > 0 && bDuty < bCeil && bDuty < Cfg.bDutyMax) {
            if (bDuty == 0) {
                uint8_t step = abHallMap[bHallValid];
                if (step != NO_STEP) {
                    GATES_OFF();
                    GATES_STEP(step);
                }
            }
            bDuty++;
        } else if (bIrelSmoothed == 0 && bDuty > 0) {
            bDuty = 0;
            GATES_OFF();
        }
    }

    if (bStepTicks != 255) bStepTicks++;
    if (bStepPos < STEP_ANGLE) {
        wStepAcc += STEP_ANGLE;
        while (wStepAcc >= bStepLen && bStepPos < STEP_ANGLE) {
            wStepAcc -= bStepLen;
            bStepPos++;
        }
    }
    a = SINE_START + bStepPos;
    if (a & 64) a = 128 - a;
    {
        uint16_t w;

        MUL8(w, bDuty, (uint8_t)(2 * pure_sine_gen[a] - 255));
        PWMDB8_1_PULSE_WIDTH_REG = DUTY_TO_CMP((uint8_t)w);
    }

    PAS_TICK(bTick);

    if ((bTick & 63) == 63) {       //1/8 s
        bEdgesOut = bEdges;
        bEdges = 0;
        bEdgesNew = 1;
    }

    if (++bTick == 0) {             //0.5s
        stRawOut.wI = wIsum >> 4;
        stRawOut.wV = wVsum;        //16 samples
        wIsum = 0;
        wVsum = 0;
        bRawNew = 1;
    }
}

void BLDC_Init(void)
{
    rebuildMap();
    //gates off; hall bits 1 (pull-ups)
    Port_1_Shadow = P1_GATES_OFF | BRAKE_CONNECTOR_MASK;
    Port_2_Shadow = P2_GATES_OFF | HALL_MASK;
    PRT1DR = Port_1_Shadow;
    PRT2DR = Port_2_Shadow;

    PWMDB8_1_Start();
    PGA_1_Start(PGA_1_HIGHPOWER);
    SAR8_1_Start();
    SAR8_1_Trigger();

    bHall = PRT2DR & HALL_MASK;     // hall interrupt
    bHallValid = (abHallToStep[bHall] != NO_STEP) ? bHall : 0;
    M8C_EnableIntMask(INT_MSK0, INT_MSK0_GPIO);
    M8C_EnableIntMask(INT_MSK0, INT_MSK0_SLEEP);
    M8C_EnableGInt;
}

void BLDC_Enable(uint8_t on)
{
    M8C_DisableGInt;
    bDuty = 0;
    bEnabled = 0;
    bSync = 0;
    bLimiting = 0;
    bCeil = 0;
    bIrelTarget = 0;
    bIrelSmoothed = 0;
    PWMDB8_1_PULSE_WIDTH_REG = 0;
    gatesTo(NO_STEP);

    if (on) {
        //charge the bootstraps: all low sides on, ~0.1 ms
        Port_1_Shadow &= ~P1_GATES;
        PRT1DR = Port_1_Shadow;
        Port_2_Shadow &= ~P2_GATES;
        PRT2DR = Port_2_Shadow;
        delayShort(30);
        gatesTo(NO_STEP);
        if (Cfg.bDutyMax > BLDC_DUTY_MAX) Cfg.bDutyMax = BLDC_DUTY_MAX;
        bEnabled = 1;
        bFaultTicks = 0;
        if (bDuty > 0) commutate();
    }
    M8C_EnableGInt;
}

uint8_t BLDC_bGetTick(void)  { return bTick; }
uint8_t BLDC_bLimiting(void) { return bLimiting; }

uint8_t BLDC_bGetRaw(tRaw *p)
{
    if (!bRawNew) return 0;
    M8C_DisableGInt;
    *p = stRawOut;
    bRawNew = 0;
    M8C_EnableGInt;
    return 1;
}

uint8_t BLDC_bGetEdges(uint8_t *p)
{
    if (!bEdgesNew) return 0;
    *p = bEdgesOut;
    bEdgesNew = 0;
    return 1;
}

uint8_t BLDC_bGetThrottle(uint8_t *p)
{
    if (!bThrottleNew) return 0;
    *p = bThrottle;
    bThrottleNew = 0;
    return 1;
}

void BLDC_SetIlim(tIlim *p)
{
    M8C_DisableGInt;
    stIlim = *p;
    M8C_EnableGInt;
}

uint8_t BLDC_bGetCeil(void) { return bCeil; }

void BLDC_SetCurrentRel(uint8_t factor)
{
    bIrelTarget = factor;
}

uint8_t BLDC_bGetCurrentRel(void) { return bIrelTarget; }

uint8_t BLDC_bGetDuty(void)      { return bDuty; }
uint8_t BLDC_bIsEnabled(void)    { return bEnabled; }
uint8_t BLDC_bHallFault(void)    { return bHallFault; }
