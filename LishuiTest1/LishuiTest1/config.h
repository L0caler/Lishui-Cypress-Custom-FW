// config.h - settings
#ifndef CONFIG_H
#define CONFIG_H

#include <m8c.h>
#include "types.h"

// UART
#define USE_BINARY_COMM         0       // 0 = text, 1 = binary
#define COMMAND_TIMEOUT_TICKS   255     // 512 Hz ticks
#define IBAT_MAX_LIMIT_MA       20000
#define IPH_MAX_LIMIT_MA        40000

// Current sense
#define SHUNT_UOHM              4069
#define CS_PGA_GAIN             48      // must match PGA_1
#define CS_R_SERIES_OHM         10000
#define CS_R_PULLUP_OHM         10000
#define VDD_MV                  5000

// Motor drive
#define BLDC_DUTY_MAX           240     // 8bit, bootstrap
#define RAMP_TICKS_MASK         3
#define HALL_FAULT_TICKS        10      // 512 Hz ticks
#define IREL_RAMP_UP            8
#define IREL_RAMP_DOWN          8
#define PI_KP_SHIFT             2
#define PI_KI_SHIFT             0
#define PI_DUTY_MIN             16      // 8bit

// PAS
#define PAS_MAGNETS             12
#define PAS_RPM_MIN             12
#define PAS_RPM_MAX             150
#define PAS_FILTER              2       // 512 Hz ticks
#define PAS_TRIGGER             2       // pulses
#define PAS_DROP                2       // periods
#define PAS_DIR_DETECT          1
#define PAS_DIR_REVERSE         0
#define PAS_DUTY_PCT            50

// Throttle / brake
#define THROTTLE_EMA_SHIFT      2
#define THROTTLE_MIN            50      // 8bit
#define THROTTLE_MAX            200     // 8bit
#define BRAKE_DEBOUNCE          2       // 32 Hz samples

// Cfg defaults
#define CFG_PMAX_W              1000
#define CFG_DUTY_MAX            BLDC_DUTY_MAX
#define CFG_IBAT_MAX_MA         300
#define CFG_IPH_MAX_MA          1000
#define CFG_ERPM_MAX            60000
#define CFG_ERPM_LIM_START      50000
#define CFG_VBAT_MIN_MV         0       // 0 = off
#define CFG_VBAT_LIM_START_MV   0
#define CFG_EXTERNAL_REL        0       // 0 = inputs.c, 1 = UART

typedef struct {
    uint16_t wPmax_W;
    uint8_t bDutyMax;
    uint16_t wIbatMax_mA;
    uint16_t wIphMax_mA;
    uint16_t wErpmMax;
    uint16_t wErpmLimStart;
    uint16_t wVbatMin_mV;
    uint16_t wVbatLimStart_mV;
    uint8_t bExternalRel;
} tCfg;

extern tCfg Cfg;

#endif
