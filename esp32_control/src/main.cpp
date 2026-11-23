// ESP32 controller for the Lishui custom firmware.
//
// Talks to the controller over its binary UART protocol (lishui_link.h).
// As soon as the controller is heard, the ESP32 writes its own limits to it
// (controller_limits.h) and waits for the acknowledgement.  Then it switches the
// command source to external and from then on answers every 32 Hz inputs
// frame with the current command, computed exactly as the controller would
// (control.h).  When the controller comes back after a reset, which brings
// back its own defaults, the limits are written again first.
//
// If the ESP32 stops answering, the controller goes back to its own throttle
// / PAS / brake handling after 125 ms; without the ESP32 it never leaves it.
//
// Push button (PUSH_GPIO to GND, internal pull-up): held without pedalling
// it pushes the bike at up to 5 km/h (walk limits, controller_limits.h);
// held while pedalling it gives full current whatever the throttle (turbo).
//
// Held at power-on instead, the button starts service mode, for as long as
// it stays held: a text log of everything the controller sends over
// Bluetooth LE for "Serial Bluetooth Terminal" (ble_log.h), and debug
// commands to change the limits in RAM (commands.h).  The ESP32 writes the
// limits and reports when the controller does not answer, but does not
// control the motor.  Releasing the button turns Bluetooth off and goes on
// in normal mode, with the limits as set.  Bluetooth is off in normal mode.
//
// USB serial (115200): a status line every second, and
//   e  take control (default)     i  hand control back to the controller

#include <Arduino.h>

#include "driver/gpio.h"

#include "control.h"
#include "controller_limits.h"
#include "ble_log.h"
#include "commands.h"
#include "lishui_link.h"

#ifndef CTRL_RX_GPIO
#define CTRL_RX_GPIO 25  // <- controller TX, P1[0], open drain low, pulled up here
#endif
#ifndef CTRL_TX_GPIO
#define CTRL_TX_GPIO 26  // -> controller RX, P1[1]
#endif
#ifndef PUSH_GPIO
#define PUSH_GPIO 27     // push button to GND, low = pressed, internal pull-up
#endif
// Bench only: TARGET_PWR_GPIO (set in platformio.ini) powers the controller's
// 3.3 V Vdd from that pin, as psoc1_prog's power mode gpio does.  Never with
// the controller on its own supply: the pin would drive against its rail.

constexpr uint32_t CTRL_BAUD = 19200;       // controller: VC3 = 24 MHz / 156, 8x
constexpr uint32_t LINK_TIMEOUT_MS = 1000;  // no frame at all: controller gone
constexpr uint32_t INPUTS_TIMEOUT_MS = 100; // no inputs frame: not external
constexpr uint32_t SOURCE_RETRY_MS = 100;   // resend the source frame this often
constexpr uint32_t LIMITS_RETRY_MS = 200;   // resend the limits until acknowledged
constexpr uint32_t REPORT_MS = 1000;
constexpr uint32_t NORMAL_CPU_MHZ = 20;     // crystal / 2, no PLL
constexpr uint32_t LOOP_SLEEP_MS = 1;       // normal mode: idle between passes
constexpr uint32_t SERVICE_CPU_MHZ = 80;    // Bluetooth minimum
constexpr uint32_t BLE_START_MIN_MS = 3000; // after power-on: controller boot and beep done
constexpr uint32_t BLE_START_MAX_MS = 6000; // start even without frames from it
// Button: integrator, sampled at 1 kHz.  Up by one per low sample, down by
// one per high sample, 0..BUTTON_INT_MAX.  Pressed when it reaches the top,
// released when it reaches 0: 0.3 s of a steady level either way, noise
// only slows it down.
constexpr uint16_t BUTTON_INT_MAX = 300;
// Walk soft start: the walk current command rises by this much per inputs
// frame (~28 Hz).  8 = the controller's own ramp (IREL_RAMP_UP), 0 -> 255 in
// ~1 s: the ESP32 adds no delay of its own; faster needs IREL_RAMP_UP in the
// controller firmware.  It drops to 0 at once on release or brake.
constexpr uint8_t WALK_REL_STEP = 8;
// The controller smooths the current command: down by IREL_RAMP_DOWN (16,
// config.h) per 32 Hz step, one step per inputs frame.  After walk the walk
// limits have to stay until that has reached 0, or the tail of the ramp runs
// under the normal limits: a burst at full power.  Modelled here with 15
// per frame, so the model is never below the real value.
constexpr uint8_t CTRL_REL_DOWN_MODEL = 15;
// The controller puts new limits into effect on its next Limit_Update(), 8
// times a second.  Walk current only once the walk limits surely are.
constexpr uint32_t LIMITS_EFFECT_MS = 150;

static HardwareSerial &ctrl = Serial1;
static lishui::Parser parser;

static bool wantExternal = true;
static bool seenFrame = false, seenInputs = false;
// Which of our two limit sets the controller has acknowledged.  NONE after
// boot and whenever the link drops (a reset brings back its own defaults).
enum class LimitSet : uint8_t { NONE, NORMAL, WALK };
static LimitSet activeLimits = LimitSet::NONE;
static LimitSet sentLimits = LimitSet::NONE;   // last frame sent, waiting for its ack
static uint8_t limitsChecksum = 0;             // of that frame, the ack echoes it
static control::Mode mode = control::Mode::NORMAL;  // set on press, kept until release
static lishui::Inputs lastInputs = {0, 1, false};  // latest inputs frame
static bool button = false;                    // debounced, true = pressed
static uint16_t buttonInt = 0;                 // integrator, see BUTTON_INT_MAX
static uint32_t buttonSampleMs = 0;
// Held at power-on (stuck button, shorted wire, a hand on it): starts as
// already pressed, so no walk / turbo until it has been released and
// pressed again.  Walk and turbo only ever start on a fresh press.
static bool buttonLocked = false;
static bool serviceMode = false;               // button held at power-on
static bool bleStarted = false;               // service mode: Bluetooth up yet
static bool brownout = false;                 // last reset was a brown-out
static uint32_t lastFrameMs = 0, lastInputsMs = 0, lastSourceMs = 0, lastReportMs = 0;
static uint32_t lastLimitsMs = 0, limitsSent = 0;
static uint8_t lastRel = 0;
static uint8_t walkRamp = 0;                   // soft start, see WALK_REL_STEP
static uint8_t ctrlRelModel = 0;               // controller's smoothed command, upper bound
static uint32_t limitsAckMs = 0;               // when activeLimits last changed
static uint32_t relFrames = 0;

static const char *limitSetName(LimitSet l) {
  return l == LimitSet::WALK ? "walk" : l == LimitSet::NORMAL ? "normal" : "none";
}

static const char *modeName(control::Mode m) {
  return m == control::Mode::WALK ? "WALK" : m == control::Mode::TURBO ? "TURBO" : "normal";
}

// The button integrator, one step per elapsed millisecond (the loop sleeps
// between passes, so a pass can cover more than one).  +1 pressed,
// -1 released, 0 no change.
static int8_t updateButton(uint32_t now) {
  uint32_t ms = now - buttonSampleMs;
  if (!ms) return 0;
  buttonSampleMs = now;
  if (ms > 10) ms = 10;  // a long stall is one sample, not a press
  if (digitalRead(PUSH_GPIO) == LOW) {
    buttonInt = (buttonInt + ms > BUTTON_INT_MAX) ? BUTTON_INT_MAX : buttonInt + ms;
  } else {
    buttonInt = (buttonInt > ms) ? buttonInt - ms : 0;
  }
  if (!button && buttonInt == BUTTON_INT_MAX) {
    button = true;
    return 1;
  }
  if (button && buttonInt == 0) {
    button = false;
    return -1;
  }
  return 0;
}

// The normal limits as sent: a RAM copy of controller_limits.h, which only
// the BLE debug commands change (service mode).  Never stored anywhere.
static lishui::Limits ramLimits = limits::LIMITS;

static void sendLimits(LimitSet set) {
  uint8_t f[lishui::LIMITS_SIZE];
  size_t n = lishui::makeLimits(set == LimitSet::WALK ? limits::WALK_LIMITS : ramLimits, f);
  limitsChecksum = f[lishui::LIMITS_SIZE - 2];
  sentLimits = set;
  ctrl.write(f, n);
  limitsSent++;
}

static void sendSource(bool external) {
  uint8_t f[lishui::SOURCE_SIZE];
  ctrl.write(f, lishui::makeSource(external, f));
}

static void sendCurrentRel(uint8_t rel) {
  uint8_t f[lishui::CURRENT_REL_SIZE];
  ctrl.write(f, lishui::makeCurrentRel(rel, f));
}

static void onInputs(const lishui::Inputs &in) {
  lastInputs = in;
  if (!wantExternal) return;  // let the controller time out to its own inputs
  bool walkInEffect = activeLimits == LimitSet::WALK && millis() - limitsAckMs >= LIMITS_EFFECT_MS;
  lastRel = control::currentRel(in, mode, walkInEffect, activeLimits == LimitSet::NORMAL);
  if (mode == control::Mode::WALK && lastRel) {
    walkRamp = (walkRamp > 255 - WALK_REL_STEP) ? 255 : walkRamp + WALK_REL_STEP;
    if (lastRel > walkRamp) lastRel = walkRamp;
  } else {
    walkRamp = 0;
  }
  sendCurrentRel(lastRel);
  relFrames++;
  // Up: at once (the controller is slower, so this stays above it).
  // Down: CTRL_REL_DOWN_MODEL per frame (the controller: 16).
  if (lastRel >= ctrlRelModel) ctrlRelModel = lastRel;
  else ctrlRelModel = (ctrlRelModel - lastRel > CTRL_REL_DOWN_MODEL)
                          ? ctrlRelModel - CTRL_REL_DOWN_MODEL : lastRel;
}

static void report(bool linkUp, bool external) {
  const lishui::Status &s = parser.status();
  uint32_t kmh_x10 = (uint32_t)s.erpm * 250 / limits::ERPM_AT_25_KMH;  // ERPM_MAX = 25 km/h
  if (!linkUp) {
    Serial.printf("no controller (bad frames %lu)\n", (unsigned long)parser.badFrames());
    return;
  }
  if (activeLimits == LimitSet::NONE) {
    Serial.printf("controller found, limits not acknowledged yet (sent %lu; "
                  "check GPIO%d -> P1[1] RX)\n",
                  (unsigned long)limitsSent, CTRL_TX_GPIO);
    return;
  }
  Serial.printf(
      "%s %s | %s btn %s lim %s | thr %3u brk %u pas %u rpm %3u -> rel %3u (ctrl %3u) | "
      "%s duty %3u %5umV %6dmA %5umA ph %5uerpm %2u.%ukm/h%s%s | stk %u%s | rel frames %lu bad %lu\n",
      external ? "EXTERNAL" : "internal", wantExternal ? "(want ext)" : "(want int)",
      modeName(mode), buttonLocked ? "LOCKED" : button ? "1" : "0", limitSetName(activeLimits),
      s.throttle, s.brake, s.pasActive, s.pasRpm, lastRel, s.currentRel,
      s.enabled ? "run" : "stop", s.duty, s.vbat_mV, s.ibat_mA, s.iph_mA, s.erpm,
      (unsigned)(kmh_x10 / 10), (unsigned)(kmh_x10 % 10),
      s.limiting ? " LIM" : "", s.hallFault ? " HALLFAULT" : "",
      s.stackFree, s.stackFree < 8 ? " LOW!" : "", (unsigned long)relFrames, (unsigned long)parser.badFrames());
}

void setup() {
#ifdef TARGET_PWR_GPIO
  // Power first, then the UART: its idle-high TX into an unpowered chip
  // would back-feed it through the pin's protection diode.
  gpio_set_level((gpio_num_t)TARGET_PWR_GPIO, 1);
  pinMode(TARGET_PWR_GPIO, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)TARGET_PWR_GPIO, GPIO_DRIVE_CAP_3);  // ~40 mA
#endif
  pinMode(PUSH_GPIO, INPUT_PULLUP);
  Serial.setTxBufferSize(1024);  // so a report line never stalls the loop
  Serial.begin(115200);

  // Button held at power-on: mostly low over 50 ms.
  delay(5);  // let the pull-up charge the wire
  uint8_t low = 0;
  for (uint8_t i = 0; i < 50; i++) {
    if (digitalRead(PUSH_GPIO) == LOW) low++;
    delay(1);
  }
  if (low >= 25) {
    button = true;
    buttonInt = BUTTON_INT_MAX;
    buttonLocked = true;
  }

  // Power: the controller's 5 V rail is weak.  Normal mode needs no radio
  // and little CPU, so it runs off the 40 MHz crystal / 2 (PLL off);
  // Bluetooth needs 80 MHz.  Both UARTs stay exact: up to 250 kbit/s they
  // run from REF_TICK, not from the CPU clock.
  // A brown-out reset with the button held: the radio was too much for the
  // supply.  Do not try again (that would loop); normal mode, button locked.
  brownout = esp_reset_reason() == ESP_RST_BROWNOUT;
  serviceMode = buttonLocked && !brownout;
  // Service mode too starts at the low clock: Bluetooth (and 80 MHz) only
  // comes up in serviceLoop(), after the controller has booted.
  setCpuFrequencyMhz(NORMAL_CPU_MHZ);

  ctrl.setRxBufferSize(256);
  ctrl.begin(CTRL_BAUD, SERIAL_8N1, CTRL_RX_GPIO, CTRL_TX_GPIO);
  // The controller's TX is open drain low: it only pulls to GND, and this
  // ~45 kOhm pull-up to 3.3 V makes the high, so the pin never sees 5 V.
  // Plenty at 19200 baud (52 us per bit, rise time a few us).  No external
  // resistor: psoc1_prog relies on its own pull-down on this wire (SDATA).
  // Set after begin(), which configures the pin.
  gpio_pullup_en((gpio_num_t)CTRL_RX_GPIO);
  gpio_pulldown_dis((gpio_num_t)CTRL_RX_GPIO);
  // Hand received bytes over after one idle symbol instead of waiting for
  // the FIFO to fill: the answer has to beat the controller's next 32 Hz
  // current ramp step, ~15 ms after the inputs frame.
  ctrl.setRxTimeout(1);

  Serial.printf("\nLishui ESP32 control: RX GPIO%d, TX GPIO%d, %lu baud, CPU %lu MHz\n",
                CTRL_RX_GPIO, CTRL_TX_GPIO, (unsigned long)CTRL_BAUD,
                (unsigned long)getCpuFrequencyMhz());
#ifdef TARGET_PWR_GPIO
  Serial.printf("BENCH: powering the controller from GPIO%d\n", TARGET_PWR_GPIO);
#endif
  const lishui::Limits &l = limits::LIMITS;
  Serial.printf("limits: %u W, duty %u, Ibat %u mA, Iph %u mA, erpm %u..%u, Vbat %u..%u mV\n",
                l.pmax_W, l.dutyMax, l.ibatMax_mA, l.iphMax_mA, l.erpmLimStart, l.erpmMax,
                l.vbatMin_mV, l.vbatLimStart_mV);
  const lishui::Limits &w = limits::WALK_LIMITS;
  Serial.printf("walk:   %u W, Ibat %u mA, Iph %u mA, erpm %u..%u (2..5 km/h, %u erpm = 25 km/h)\n",
                w.pmax_W, w.ibatMax_mA, w.iphMax_mA, w.erpmLimStart, w.erpmMax,
                limits::ERPM_AT_25_KMH);
  Serial.printf("push button on GPIO%d (to GND)\n", PUSH_GPIO);
  if (brownout && buttonLocked)
    Serial.println("brown-out reset: no Bluetooth this time, button LOCKED until released");
  if (serviceMode) {
    // Never take control in this mode: the controller stays on its own
    // throttle / PAS / brake, as without the ESP32.
    Serial.println("SERVICE MODE: motor control is off; release the button to leave.");
    Serial.println("Bluetooth LE (BLE_NAME in platformio.ini) at -12 dBm, starting once the "
                   "controller has booted. In the app, send BLE_PASSWORD first, then help.");
    return;
  }
  Serial.println("e = take control, i = hand control back");
}

// --- service mode --------------------------------------------------------
// Only listens and writes the limits: no source frame, no current command,
// so the controller stays on its own throttle / PAS / brake.

constexpr uint32_t SERVICE_ERROR_MS = 2000;   // repeat an error this often
constexpr uint8_t SERVICE_ACK_TRIES = 5;      // limits sent before it is an error

static bool svcAcked = false;
static uint8_t svcTries = 0;
static uint32_t svcRxBytes = 0, svcLastByteMs = 0, svcLastErrorMs = 0;

static void logStatus(Print &out, uint32_t now) {
  const lishui::Status &s = parser.status();
  uint32_t kmh_x10 = (uint32_t)s.erpm * 250 / limits::ERPM_AT_25_KMH;
  out.printf("[%lu.%03lu] %s duty %u | %u mV %d mA | ph %u mA | %u erpm %u.%u km/h | "
             "thr %u brk %s pas %u %urpm | cmd %u%s%s | stk %u%s | btn %s | bad %lu\n",
             (unsigned long)(now / 1000), (unsigned long)(now % 1000),
             s.enabled ? "run" : "stop", s.duty, s.vbat_mV, s.ibat_mA, s.iph_mA, s.erpm,
             (unsigned)(kmh_x10 / 10), (unsigned)(kmh_x10 % 10), s.throttle,
             s.brake ? "off" : "ON", s.pasActive, s.pasRpm, s.currentRel,
             s.limiting ? " LIM" : "", s.hallFault ? " HALLFAULT" : "", s.stackFree,
             s.stackFree < 8 ? " LOW!" : "", button ? "on" : "off",
             (unsigned long)parser.badFrames());
}

// Button released: Bluetooth off, back to the normal clock, and on in
// normal mode.  The limits stay as set over BLE (RAM only), and if the
// controller has acknowledged them, normal mode takes control right away.
static void leaveServiceMode() {
  // No goodbye over BLE: the link just ends.
  blelog::end();
  setCpuFrequencyMhz(NORMAL_CPU_MHZ);
  serviceMode = false;
  buttonLocked = false;
  mode = control::Mode::NORMAL;
  sentLimits = LimitSet::NORMAL;  // limitsChecksum is that of ramLimits
  activeLimits = svcAcked ? LimitSet::NORMAL : LimitSet::NONE;
  Serial.printf("NORMAL MODE: Bluetooth off, CPU %lu MHz\n", (unsigned long)getCpuFrequencyMhz());
  commands::printLimits(ramLimits, Serial);
}

static void serviceLoop() {
  // Bluetooth only once the controller is through its boot (and its start-up
  // beep): both together drag its 5 V rail down.  Frames from it mean it
  // runs; without any, start anyway after BLE_START_MAX_MS.
  if (!bleStarted) {
    uint32_t t = millis();
    if (t >= BLE_START_MIN_MS && (seenFrame || t >= BLE_START_MAX_MS)) {
      setCpuFrequencyMhz(SERVICE_CPU_MHZ);
      blelog::begin();
      bleStarted = true;
      Serial.printf("Bluetooth on, CPU %lu MHz\n", (unsigned long)getCpuFrequencyMhz());
    }
  }
  blelog::loop();
  Print &out = blelog::log();

  // Debug commands, BLE only.  A changed limit: send them all again now.
  std::string cmd;
  while (blelog::readLine(cmd)) {
    if (commands::run(cmd, ramLimits, out)) {
      svcAcked = false;
      svcTries = 0;
      lastLimitsMs = millis() - LIMITS_RETRY_MS;
    }
  }

  while (ctrl.available()) {
    uint32_t now = millis();
    svcRxBytes++;
    svcLastByteMs = now;
    lishui::Parser::Result r = parser.feed((uint8_t)ctrl.read());
    if (r == lishui::Parser::NONE) continue;
    if (!seenFrame || now - lastFrameMs >= LINK_TIMEOUT_MS) out.println("controller found");
    seenFrame = true;
    lastFrameMs = now;
    if (r == lishui::Parser::STATUS) {
      logStatus(out, now);
    } else if (r == lishui::Parser::INPUTS) {
      const lishui::Inputs &in = parser.inputs();
      out.printf("inputs | thr %u brk %s pas %u\n", in.throttle, in.brake ? "off" : "ON", in.pasActive);
    } else if (r == lishui::Parser::LIMITS_ACK) {
      if (parser.limitsAck() != limitsChecksum) {
        out.printf("limits ack with checksum 0x%02X, expected 0x%02X\n", parser.limitsAck(),
                   limitsChecksum);
      } else if (!svcAcked) {
        svcAcked = true;
        out.printf("OK: controller acknowledged the limits (%u tries)\n", svcTries);
        commands::printLimits(ramLimits, out);
      }
    }
  }

  uint32_t now = millis();
  if (updateButton(now) < 0) {
    leaveServiceMode();
    return;
  }

  bool linkUp = seenFrame && now - lastFrameMs < LINK_TIMEOUT_MS;
  if (!linkUp && (svcAcked || svcTries)) {
    // Lost, or reset to its own defaults: write the limits again when back.
    if (svcAcked) out.println("controller lost: no valid frame for 1 s");
    svcAcked = false;
    svcTries = 0;
  }
  if (linkUp && !svcAcked && now - lastLimitsMs >= LIMITS_RETRY_MS) {
    sendLimits(LimitSet::NORMAL);
    lastLimitsMs = now;
    if (svcTries < 255) svcTries++;
  }

  if (now - svcLastErrorMs < SERVICE_ERROR_MS) return;
  svcLastErrorMs = now;
  if (!linkUp) {
    if (!svcRxBytes || now - svcLastByteMs >= LINK_TIMEOUT_MS)
      out.printf("ERROR: nothing received from the controller (GPIO%d <- P1[0] TX). "
                 "Check that wire, GND and that the controller is on.\n", CTRL_RX_GPIO);
    else
      out.printf("ERROR: bytes from the controller, but no valid frame (%lu bad checksums). "
                 "Check the GPIO%d <- P1[0] TX wire and GND.\n",
                 (unsigned long)parser.badFrames(), CTRL_RX_GPIO);
  } else if (!svcAcked && svcTries >= SERVICE_ACK_TRIES) {
    out.printf("ERROR: the controller does not acknowledge the limits (sent %u times). "
               "Check the GPIO%d -> P1[1] RX wire.\n", svcTries, CTRL_TX_GPIO);
  }
}

void loop() {
  if (serviceMode) {
    serviceLoop();
    delay(LOOP_SLEEP_MS);  // idle between passes here too
    return;
  }

  while (ctrl.available()) {
    lishui::Parser::Result r = parser.feed((uint8_t)ctrl.read());
    if (r == lishui::Parser::NONE) continue;
    uint32_t now = millis();
    seenFrame = true;
    lastFrameMs = now;
    if (r == lishui::Parser::INPUTS) {
      seenInputs = true;
      lastInputsMs = now;
      onInputs(parser.inputs());
    } else if (r == lishui::Parser::LIMITS_ACK && sentLimits != LimitSet::NONE &&
               parser.limitsAck() == limitsChecksum) {
      if (activeLimits != sentLimits) {
        Serial.printf("%s limits written to the controller\n", limitSetName(sentLimits));
        limitsAckMs = now;
      }
      activeLimits = sentLimits;
    }
  }

  uint32_t now = millis();

  int8_t edge = updateButton(now);
  if (edge > 0) {
    mode = control::pressMode(lastInputs);
  } else if (edge < 0) {
    mode = control::Mode::NORMAL;
    if (buttonLocked) {
      buttonLocked = false;
      Serial.println("push button released: enabled");
    }
  }

  bool linkUp = seenFrame && now - lastFrameMs < LINK_TIMEOUT_MS;
  // The controller only sends inputs frames while its source is external.
  bool external = seenInputs && now - lastInputsMs < INPUTS_TIMEOUT_MS;

  // A silent controller may have reset to its own defaults: write the
  // limits again once it is back.
  if (!linkUp) activeLimits = LimitSet::NONE;
  if (!external) ctrlRelModel = 0;  // no frames to count; the controller has it
  // Walk limits while walking under our control, and after it until the
  // controller's smoothed current is back at 0 (CTRL_REL_DOWN_MODEL): the
  // command is already 0 then, so this delay cannot be felt.  Anything else,
  // including handing control back, gets the normal limits.
  bool walking = wantExternal && external && mode == control::Mode::WALK;
  bool walkTail = activeLimits == LimitSet::WALK && ctrlRelModel > 0;
  LimitSet want = (walking || walkTail) ? LimitSet::WALK : LimitSet::NORMAL;

  if (linkUp && activeLimits != want) {
    // Right away on a change of set, then every LIMITS_RETRY_MS until acked.
    if (sentLimits != want || now - lastLimitsMs >= LIMITS_RETRY_MS) {
      sendLimits(want);
      lastLimitsMs = now;
    }
  } else if (linkUp && external != wantExternal && now - lastSourceMs >= SOURCE_RETRY_MS) {
    // Only once our limits are in place.
    sendSource(wantExternal);
    lastSourceMs = now;
  }

  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'e' || c == 'E') wantExternal = true;
    if (c == 'i' || c == 'I') wantExternal = false;
  }

  if (now - lastReportMs >= REPORT_MS) {
    lastReportMs = now;
    report(linkUp, external);
  }

  // Power: sleep until the next tick instead of spinning; the idle task
  // halts the CPU until an interrupt.  The UART driver buffers what comes
  // in, so this adds at most ~1 ms to a reply, against ~15 ms of margin
  // before the controller's next current step.
  delay(LOOP_SLEEP_MS);
}
