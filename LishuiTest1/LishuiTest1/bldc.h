// bldc.h - 6-step hall-commutated BLDC drive, PWMDB8_1 on the high sides
#ifndef BLDC_H
#define BLDC_H

#include <m8c.h>
#include "types.h"
#include "limit.h"

// Shadows of PRT1DR / PRT2DR (PRTxDR reads the pins, not the latch).
// The gate bits belong to bldc.c.
extern uint8_t Port_1_Shadow;
extern uint8_t Port_2_Shadow;

void BLDC_Init(void);
void BLDC_Enable(uint8_t on);
void BLDC_SetReverse(uint8_t reverse);// only while disabled

uint8_t BLDC_bGetDuty(void);
uint8_t BLDC_bIsEnabled(void);
uint8_t BLDC_bIsReverse(void);
uint8_t BLDC_bHallFault(void);         // halls 000 / 111 for > 20 ms

// Raw sums of one 0.5 s window, from the ISRs
typedef struct {
    uint16_t wI;// current, counts x16
    uint16_t wV;// vbat, counts x16
    uint16_t wEdges;// hall edges
} tRaw;

uint8_t BLDC_bGetRaw(tRaw *p);         // 1 = new window copied
uint8_t BLDC_bGetThrottle(uint8_t *p); // 1 = new raw SAR sample, 32 Hz
// Battery current target for the PI, from Limit_Update().
// ISR: target = min(wBat, wPh * duty / 256) * bScale / 256 + wZero
#define BLDC_LIM_BAT   0x01
#define BLDC_LIM_PH    0x02
#define BLDC_LIM_W     0x04
#define BLDC_LIM_VBAT  0x08
#define BLDC_LIM_ERPM  0x10
typedef struct {
    uint16_t wZero;// counts x16
    uint16_t wBat;// min(Ibat max, Pmax / Vbat), counts x16
    uint16_t wPh;// Iph max, counts x16
    uint8_t bScale;// Vbat / ERPM derate, 255 = none
    uint8_t bFlags;// BLDC_LIM_BAT or _W, plus _VBAT / _ERPM
} tIlim;
void BLDC_SetIlim(tIlim *p);
uint8_t BLDC_bLimiting(void);// 0 or BLDC_LIM_... flags
uint8_t BLDC_bGetCeil(void);// PI duty ceiling
uint8_t BLDC_bGetTick(void);// 512 Hz

void BLDC_SetCurrentRel(uint8_t factor); // set current factor 0-255 with ramp up/down
uint8_t BLDC_bGetCurrentRel(void);

#endif
