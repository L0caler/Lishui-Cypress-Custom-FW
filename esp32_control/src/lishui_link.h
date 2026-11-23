// lishui_link.h - binary UART protocol of the Lishui controller firmware.
//
// The other end is LishuiTest1/LishuiTest1/comm_binary.c; keep the two in
// step.  Little endian, checksum = 8 bit sum of every byte before it.
//
//   From the controller:
//     AA 55 enabled duty limiting hallFault Ibat(2) Vbat(2) erpm(2) Iph(2)
//           currentRel pasActive pasRpm(2) throttle brake stackFree checksum
//                                                                     4 Hz
//     AA 58 throttle brake pasActive checksum                         32 Hz,
//           only while the command source is external, right after each
//           throttle sample
//     AA 59 limitsChecksum checksum       a limits frame was applied
//   To the controller:
//     AA 56 currentRel checksum           current command, external only
//     AA 57 source ~source checksum       0 = controller's own inputs, 1 = UART
//     AA 5A Pmax(2) dutyMax IbatMax(2) IphMax(2) erpmMax(2) erpmLimStart(2)
//           VbatMin(2) VbatLimStart(2) checksum ~checksum
//                                         limits, any time, nothing else changes
//
// While external, the controller falls back to its own inputs when no AA 56
// arrives for COMMAND_TIMEOUT_TICKS (125 ms).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace lishui {

constexpr uint8_t SYNC = 0xAA;
constexpr uint8_t STATUS_TYPE = 0x55;
constexpr uint8_t CURRENT_REL_TYPE = 0x56;
constexpr uint8_t SOURCE_TYPE = 0x57;
constexpr uint8_t INPUTS_TYPE = 0x58;
constexpr uint8_t LIMITS_ACK_TYPE = 0x59;
constexpr uint8_t LIMITS_TYPE = 0x5A;

constexpr size_t STATUS_SIZE = 22;
constexpr size_t INPUTS_SIZE = 6;
constexpr size_t LIMITS_ACK_SIZE = 4;
constexpr size_t CURRENT_REL_SIZE = 4;
constexpr size_t SOURCE_SIZE = 5;
constexpr size_t LIMITS_SIZE = 19;

// The controller's tCfg without bExternalRel (config.h, comm_binary.c).
struct Limits {
  uint16_t pmax_W;
  uint8_t dutyMax;          // 8 bit duty
  uint16_t ibatMax_mA;
  uint16_t iphMax_mA;
  uint16_t erpmMax;         // current falls to 0 here ...
  uint16_t erpmLimStart;    // ... from full here
  uint16_t vbatMin_mV;      // current falls to 0 here, 0 = off ...
  uint16_t vbatLimStart_mV; // ... from full here
};

struct Status {
  bool enabled;
  uint8_t duty;        // 0..255
  uint8_t limiting;    // BLDC_LIM_* flags
  bool hallFault;
  int16_t ibat_mA;
  uint16_t vbat_mV;
  uint16_t erpm;
  uint16_t iph_mA;
  uint8_t currentRel;  // current command the controller is using, 0..255
  bool pasActive;
  uint16_t pasRpm;
  uint8_t throttle;    // filtered, 8 bit
  uint8_t brake;       // debounced pin: 1 = released, 0 = pulled
  uint8_t stackFree;   // controller stack bytes never used since boot, 0 = may have wrapped
};

// Exactly what the controller's inputs.c works from.
struct Inputs {
  uint8_t throttle;    // filtered, 8 bit
  uint8_t brake;       // debounced pin: 1 = released, 0 = pulled
  bool pasActive;
};

class Parser {
 public:
  enum Result { NONE, STATUS, INPUTS, LIMITS_ACK };

  // Feed one received byte; returns which frame it completed, if any.
  Result feed(uint8_t c);

  const Status &status() const { return status_; }
  const Inputs &inputs() const { return inputs_; }
  // Checksum byte of the limits frame the last LIMITS_ACK acknowledged.
  uint8_t limitsAck() const { return limitsAck_; }
  uint32_t badFrames() const { return badFrames_; }

 private:
  static size_t frameSize(uint8_t type);
  Result decode();

  uint8_t buf_[STATUS_SIZE];
  size_t pos_ = 0;
  Status status_ = {};
  Inputs inputs_ = {};
  uint8_t limitsAck_ = 0;
  uint32_t badFrames_ = 0;
};

// Build a frame into out (at least the frame size); returns its length.
size_t makeCurrentRel(uint8_t rel, uint8_t *out);
size_t makeSource(bool external, uint8_t *out);
size_t makeLimits(const Limits &l, uint8_t *out);  // out[LIMITS_SIZE - 2]: its checksum

}  // namespace lishui
