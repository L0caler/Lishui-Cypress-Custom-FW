// config.h - settings
#ifndef CONFIG_H
#define CONFIG_H

#include <m8c.h>
#include "types.h"

// UART & System
#define USE_BINARY_COMM         1       // 0 = text, 1 = binary
#define AUTO_ENABLE             1       // 1 = auto-zero current & enable on boot, 0 = wait for UART 'g'
// Binary only: while the current command comes from the UART, no current
// frame for this long hands control back to inputs.c (throttle / PAS / brake).
// The ESP32 answers every 32 Hz inputs frame, so 64 ticks (125 ms) is about 4
// missed frames.  Max 255 (8 bit tick).
#define COMMAND_TIMEOUT_TICKS   64      // 512 Hz ticks
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
#define RAMP_TICKS_MASK         0       // duty +1 per tick: 512 / s (was 1: 256 / s)
#define HALL_FAULT_TICKS        10      // 512 Hz ticks

// Tick calibration: the 512 Hz sleep timer runs off the internal 32 kHz
// oscillator (15..64 kHz spec).  At boot, delayShort(255) calls are counted
// over CAL_TICKS ticks, timed by the CPU clock (IMO, +-2.5 %).
#define CPU_HZ                  12000000UL  // SysClk / 2, CPU_CLOCK 2
#define CAL_TICKS               128         // ~0.25 s at boot
#define CAL_CYCLES              6800UL      // CPU cycles per calibration loop pass:
                                            // delayShort(255) = 89 + 26 * 255, + loop ~80
#define CAL_N_NOMINAL           ((uint16_t)(CAL_TICKS * CPU_HZ / 512 / CAL_CYCLES))  // 441
#define CAL_N_MIN               150         // outside: oscillator implausible, use nominal
#define CAL_N_MAX               1500
// erpm = edges per 64 ticks * ERPM_CAL_K / N  (80 per edge at 512 Hz)
#define ERPM_CAL_K              (10UL * CPU_HZ / 64 * CAL_TICKS / CAL_CYCLES)        // 35294
// measured tick rate in Hz = TICK_CAL_K / N
#define TICK_CAL_K              (CAL_TICKS * CPU_HZ / CAL_CYCLES)                    // 225882

#define IREL_RAMP_UP            8
#define IREL_RAMP_DOWN          16      // per 32 Hz sample: 255 -> 0 in ~0.5 s (was 8: ~1 s)
#define ILIM_UP_SHIFT           4       // duty up per 32 Hz sample: 1 + error >> (shift + duty/64)
#define ILIM_DOWN_SHIFT         6       // duty down per sample: error >> (shift + duty/64)
                                        // gain falls with duty (plant gain ~ Vbat*duty/R)
#define ILIM_DUTY_MIN           16      // 8bit
#define ILIM_DEADBAND           8       // counts x16, ~0.1 A: duty holds within +-
#define SYNC_ON_COUNTS          48      // counts x16 above zero, ~0.6 A
#define SYNC_OFF_COUNTS         16      // ~0.2 A
#define SYNC_HOLD               4       // 32 Hz samples before sync switches

// Beep
#define BEEP_PULSE              24      // on-time per period, ~2.6 us each.  Peak A = Vbat * time / L
                                        // (was 16: +50 % peak current, ~+3.5 dB)
// 480..3000 Hz, outside: compile error (array size -1)
#define BEEP_HZ(f)              ((uint8_t)(120000UL / (f) - 5 - BEEP_PULSE * 5 / 16 \
                                    + 0 * sizeof(char[((f) >= 480 && (f) <= 3000) ? 1 : -1])))
#define BEEP_TICKS              140      // 512 Hz ticks per tone

// PAS
#define PAS_MAGNETS             12
#define PAS_RPM_MIN             8
#define PAS_RPM_MAX             150
#define PAS_PRESCALE            2       // period counters: 512 Hz / prescale, power of 2
#define PAS_FILTER              4       // 512 Hz ticks (~7.8 ms), keep well below the
                                        // half period at PAS_RPM_MAX (~16 ms)
#define PAS_TRIGGER             1       // pulses
#define PAS_DROP                1.8     // stop if no pulse for PAS_DROP x the last period
                                        // (cadence drops below rpm / PAS_DROP)
#define PAS_DIR_DETECT          0
#define PAS_DIR_REVERSE         0
#define PAS_DUTY_PCT            50

// Throttle / brake
#define THROTTLE_EMA_SHIFT      2
#define THROTTLE_MIN            52      // 8bit
#define THROTTLE_MAX            67     // 8bit
#define BRAKE_DEBOUNCE          2       // 32 Hz samples

// Cfg defaults
#define CFG_PMAX_W              500
#define CFG_DUTY_MAX            BLDC_DUTY_MAX
#define CFG_IBAT_MAX_MA         8000
#define CFG_IPH_MAX_MA          20000
#define CFG_ERPM_MAX            9900    // edge counter (uint8, 1/8 s) tops out at 20400
#define CFG_ERPM_LIM_START      ((uint16_t)(CFG_ERPM_MAX * 95UL / 100))
#define CFG_VBAT_MIN_MV         30500       // 0 = off
#define CFG_VBAT_LIM_START_MV   35000
#define VBAT_WARN1_MV           35000       // low-battery cues, independent of the derate:
#define VBAT_WARN2_MV           33000       // 1 chirp pair at WARN1, 2 at WARN2, 3 at CFG_VBAT_MIN
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
