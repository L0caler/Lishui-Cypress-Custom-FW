// inputs.h - throttle and brake
#ifndef INPUTS_H
#define INPUTS_H

#include <m8c.h>
#include "types.h"
#include "config.h"

void Inputs_Update(void);
uint8_t Inputs_bThrottle(void);     // 8bit, filtered
uint8_t Inputs_bBrake(void);        // debounced

#endif
