// config.h - settings
#ifndef CONFIG_H
#define CONFIG_H

#include <m8c.h>
#include "types.h"

// UART & System
#define USE_BINARY_COMM         0       // 0 = text, 1 = binary
#define AUTO_ENABLE             1       // 1 = auto-zero current & enable on boot, 0 = wait for UART 'g'
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
#define RAMP_TICKS_MASK         1       // duty +1 per 2 ticks: 256 / s
#define HALL_FAULT_TICKS        10      // 512 Hz ticks
#define IREL_RAMP_UP            8
#define IREL_RAMP_DOWN          8
#define ILIM_UP_SHIFT           4       // duty up per 32 Hz sample: 1 + error >> shift
#define ILIM_DOWN_SHIFT         6       // duty down per sample: error >> shift
#define ILIM_DUTY_MIN           16      // 8bit
#define SYNC_ON_COUNTS          24      // counts x16 above zero, ~0.3 A
#define SYNC_OFF_COUNTS         8       // ~0.1 A

// PAS
#define PAS_MAGNETS             12
#define PAS_RPM_MIN             8
#define PAS_RPM_MAX             150
#define PAS_PRESCALE            2       // period counters: 512 Hz / prescale, power of 2
#define PAS_FILTER              2       // 512 Hz ticks
#define PAS_TRIGGER             1       // pulses
#define PAS_DROP                2       // periods
#define PAS_DIR_DETECT          0
#define PAS_DIR_REVERSE         0
#define PAS_DUTY_PCT            50

// Throttle / brake
#define THROTTLE_EMA_SHIFT      2
#define THROTTLE_MIN            51      // 8bit
#define THROTTLE_MAX            68     // 8bit
#define BRAKE_DEBOUNCE          2       // 32 Hz samples

// Cfg defaults
#define CFG_PMAX_W              500
#define CFG_DUTY_MAX            BLDC_DUTY_MAX
#define CFG_IBAT_MAX_MA         10000
#define CFG_IPH_MAX_MA          20000
#define CFG_ERPM_MAX            4907
#define CFG_ERPM_LIM_START      4300
#define CFG_VBAT_MIN_MV         30500       // 0 = off
#define CFG_VBAT_LIM_START_MV   35000
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
