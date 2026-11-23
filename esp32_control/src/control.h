// control.h - the current command, computed exactly as the controller's own
// LishuiTest1/LishuiTest1/inputs.c does it.
//
// The controller sends the inputs after its own filtering (throttle EMA,
// brake debounce, PAS detection), so only the final mapping lives here.
// Keep THROTTLE_MIN/MAX in step with config.h.
#pragma once

#include <stdint.h>

#include "lishui_link.h"

namespace control {

constexpr uint8_t THROTTLE_MIN = 52;  // config.h THROTTLE_MIN, 8 bit
constexpr uint8_t THROTTLE_MAX = 67;  // config.h THROTTLE_MAX, 8 bit
static_assert(THROTTLE_MIN < THROTTLE_MAX, "throttle range");

// inputs.c throttleToRel(): 0 below MIN, 255 above MAX, linear between,
// truncating like the 16 bit integer maths there.
inline uint8_t throttleToRel(uint8_t t) {
  if (t <= THROTTLE_MIN) return 0;
  if (t >= THROTTLE_MAX) return 255;
  uint16_t w = (uint16_t)(t - THROTTLE_MIN) * 255;
  w /= (uint8_t)(THROTTLE_MAX - THROTTLE_MIN);
  return (uint8_t)w;
}

// inputs.c Inputs_Update(): throttle only while pedalling and the brake is
// released (brake pin 1).
inline uint8_t currentRel(const lishui::Inputs &in) {
  if (in.pasActive && in.brake) return throttleToRel(in.throttle);
  return 0;
}

// The push button on top of that.
//
//   pressed while not pedalling   walk: WALK_REL under the walk limits (5 km/h)
//   pressed while pedalling       turbo: full current while pedalling,
//                                 whatever the throttle
//   released                      as above, the controller's own behaviour
//
// The mode is decided when the button is pressed and kept until it is
// released: a stray PAS pulse while pushing the bike cannot turn walk into
// turbo, and turbo without pedalling gives no current.
//
// Walk only runs once the controller has acknowledged the walk limits, and
// throttle / turbo only once it has the normal limits back; in between the
// current is 0.  The brake always wins, here and on the controller.
enum class Mode : uint8_t { NORMAL, TURBO, WALK };
constexpr uint8_t WALK_REL = 255;  // the walk limits set the strength, not this

// The mode for a press that starts now, from the latest inputs.
inline Mode pressMode(const lishui::Inputs &in) {
  return in.pasActive ? Mode::TURBO : Mode::WALK;
}

inline uint8_t currentRel(const lishui::Inputs &in, Mode m, bool walkLimitsActive,
                          bool normalLimitsActive) {
  if (!in.brake) return 0;
  switch (m) {
    case Mode::WALK:
      return walkLimitsActive ? WALK_REL : 0;
    case Mode::TURBO:
      return (normalLimitsActive && in.pasActive) ? 255 : 0;
    default:
      return normalLimitsActive ? currentRel(in) : 0;
  }
}

}  // namespace control
