// controller_limits.h - the controller limits this ESP32 writes to the Lishui on boot.
//
// The ESP32 sends them as a limits frame (AA 5A) as soon as it hears the
// controller, and again whenever the controller comes back after a reset,
// until the controller acknowledges them (AA 59).  They replace the
// controller's own CFG_* defaults from LishuiTest1/LishuiTest1/config.h while
// the ESP32 is connected; without the ESP32 those defaults apply.
//
// Copied from config.h.  The controller clamps them on receipt, as for its
// own config frame: dutyMax <= BLDC_DUTY_MAX (240), ibatMax <= 20000 mA,
// iphMax <= 40000 mA, erpmLimStart <= erpmMax, vbatMin < vbatLimStart.
#pragma once

#include "lishui_link.h"

namespace limits {

constexpr uint16_t ERPM_MAX = 9900;  // config.h CFG_ERPM_MAX

constexpr lishui::Limits LIMITS = {
    /* pmax_W          */ 500,                                    // CFG_PMAX_W
    /* dutyMax         */ 240,                                    // CFG_DUTY_MAX = BLDC_DUTY_MAX
    /* ibatMax_mA      */ 8000,                                   // CFG_IBAT_MAX_MA
    /* iphMax_mA       */ 15000,                                  // CFG_IPH_MAX_MA
    /* erpmMax         */ ERPM_MAX,                               // CFG_ERPM_MAX
    /* erpmLimStart    */ (uint16_t)(ERPM_MAX * 95UL / 100),      // CFG_ERPM_LIM_START, 9405
    /* vbatMin_mV      */ 30500,                                  // CFG_VBAT_MIN_MV, 0 = off
    /* vbatLimStart_mV */ 35000,                                  // CFG_VBAT_LIM_START_MV
};

// Push / walk assist: while the push button is held without pedalling.
//
// Speed comes from the controller's erpm (hall edges, 8 Hz), scaled by
// ERPM_MAX (9900) = 25 km/h.  The controller ramps the current down from
// WALK_ERPM_LIM_START and reaches 0 at WALK_ERPM_MAX, so 5 km/h is a hard
// ceiling, enforced by the controller itself.
//
// That ramp is a proportional speed controller, and the controller only
// measures speed every 1/8 s.  A steep one (full current until 1 km/h below
// the cap) overshoots and surges, so it is wide: from 2 km/h, ~10 erpm steps.
// The push itself: 80 W, 3 A battery, 14 A phase (torque follows the phase
// current).
constexpr uint16_t ERPM_AT_25_KMH = ERPM_MAX;       // the riding limit is 25 km/h
constexpr uint16_t erpmAtKmh(uint32_t kmh_x10) {
  return (uint16_t)(ERPM_AT_25_KMH * kmh_x10 / 250);
}
constexpr uint16_t WALK_ERPM_MAX = erpmAtKmh(50);   // 5.0 km/h: no current at all
constexpr uint16_t WALK_ERPM_LIM_START = erpmAtKmh(20);  // 2.0 km/h: ramp down from here

constexpr lishui::Limits WALK_LIMITS = {
    /* pmax_W          */ 80,                 // ~2 A at 40 V
    /* dutyMax         */ LIMITS.dutyMax,
    /* ibatMax_mA      */ 3000,              // a push, not a ride
    /* iphMax_mA       */ 14000,             // starting torque from standstill
    /* erpmMax         */ WALK_ERPM_MAX,
    /* erpmLimStart    */ WALK_ERPM_LIM_START,
    /* vbatMin_mV      */ LIMITS.vbatMin_mV,
    /* vbatLimStart_mV */ LIMITS.vbatLimStart_mV,
};

static_assert(WALK_LIMITS.erpmLimStart < WALK_LIMITS.erpmMax, "walk derate range");
static_assert(WALK_LIMITS.erpmMax < LIMITS.erpmLimStart, "walk must be slower than riding");
static_assert(WALK_LIMITS.ibatMax_mA <= LIMITS.ibatMax_mA, "walk weaker than riding");
static_assert(WALK_LIMITS.iphMax_mA <= LIMITS.iphMax_mA, "walk weaker than riding");
static_assert(WALK_LIMITS.pmax_W <= LIMITS.pmax_W, "walk weaker than riding");

static_assert(LIMITS.dutyMax <= 240, "controller clamps dutyMax to BLDC_DUTY_MAX");
static_assert(LIMITS.ibatMax_mA <= 20000, "controller clamps to IBAT_MAX_LIMIT_MA");
static_assert(LIMITS.iphMax_mA <= 40000, "controller clamps to IPH_MAX_LIMIT_MA");
static_assert(LIMITS.erpmLimStart <= LIMITS.erpmMax, "derate starts above the max");
static_assert(LIMITS.vbatMin_mV < LIMITS.vbatLimStart_mV, "Vbat derate range");

}  // namespace limits
