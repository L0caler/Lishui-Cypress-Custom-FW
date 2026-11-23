#include "commands.h"

#include <Arduino.h>

#include <ctype.h>
#include <stdlib.h>

namespace commands {

// The controller's clamps (comm_binary.c applyLimits), checked here so a
// value is refused with a reason instead of being changed silently.
constexpr uint32_t DUTY_MAX = 240;        // BLDC_DUTY_MAX
constexpr uint32_t IBAT_MAX_MA = 20000;   // IBAT_MAX_LIMIT_MA
constexpr uint32_t IPH_MAX_MA = 40000;    // IPH_MAX_LIMIT_MA

struct Field {
  const char *name;
  const char *unit;
  uint32_t max;
  bool wide;  // uint16_t field, else uint8_t
  size_t offset;
};

static const Field FIELDS[] = {
    {"pmax", "W", 65535, true, offsetof(lishui::Limits, pmax_W)},
    {"duty", "", DUTY_MAX, false, offsetof(lishui::Limits, dutyMax)},
    {"ibat", "mA", IBAT_MAX_MA, true, offsetof(lishui::Limits, ibatMax_mA)},
    {"iph", "mA", IPH_MAX_MA, true, offsetof(lishui::Limits, iphMax_mA)},
    {"erpm", "erpm", 65535, true, offsetof(lishui::Limits, erpmMax)},
    {"erpmstart", "erpm", 65535, true, offsetof(lishui::Limits, erpmLimStart)},
    {"vbatmin", "mV", 65535, true, offsetof(lishui::Limits, vbatMin_mV)},
    {"vbatstart", "mV", 65535, true, offsetof(lishui::Limits, vbatLimStart_mV)},
};

static uint32_t get(const lishui::Limits &lim, const Field &f) {
  const uint8_t *p = (const uint8_t *)&lim + f.offset;
  return f.wide ? *(const uint16_t *)p : *p;
}

static void set(lishui::Limits &lim, const Field &f, uint32_t v) {
  uint8_t *p = (uint8_t *)&lim + f.offset;
  if (f.wide) *(uint16_t *)p = (uint16_t)v;
  else *p = (uint8_t)v;
}

void printLimits(const lishui::Limits &lim, Print &out) {
  out.print("limits (RAM):");
  for (const Field &f : FIELDS) out.printf(" %s=%lu%s", f.name, (unsigned long)get(lim, f), f.unit);
  out.println();
}

static void help(Print &out) {
  out.println("commands: limits | help | <name>=<value>");
  out.print("names:");
  for (const Field &f : FIELDS) out.printf(" %s", f.name);
  out.println();
  out.println("example: erpm=9999 (RAM only, kept in normal mode until power-off)");
}

// The pairs the controller would otherwise clamp.
static const char *checkPairs(const lishui::Limits &l) {
  if (l.erpmLimStart > l.erpmMax) return "erpmstart must not be above erpm";
  if (l.vbatMin_mV >= l.vbatLimStart_mV) return "vbatmin must be below vbatstart";
  return nullptr;
}

static std::string clean(const std::string &line) {
  std::string s;
  for (char c : line)
    if (!isspace((unsigned char)c)) s += (char)tolower((unsigned char)c);
  return s;
}

bool run(const std::string &line, lishui::Limits &lim, Print &out) {
  std::string s = clean(line);
  if (s.empty()) return false;
  out.printf("> %s\n", line.c_str());

  if (s == "limits") {
    printLimits(lim, out);
    return false;
  }
  if (s == "help" || s == "?") {
    help(out);
    return false;
  }

  size_t eq = s.find('=');
  if (eq == std::string::npos) {
    out.println("unknown command, try help");
    return false;
  }
  std::string name = s.substr(0, eq), value = s.substr(eq + 1);
  const Field *field = nullptr;
  for (const Field &f : FIELDS)
    if (name == f.name) field = &f;
  if (!field) {
    out.printf("unknown limit '%s', try help\n", name.c_str());
    return false;
  }
  char *end = nullptr;
  unsigned long v = strtoul(value.c_str(), &end, 10);
  if (value.empty() || *end || v > field->max) {
    out.printf("%s: need a number 0..%lu\n", field->name, (unsigned long)field->max);
    return false;
  }

  lishui::Limits next = lim;
  set(next, *field, v);
  if (const char *why = checkPairs(next)) {
    out.printf("refused: %s\n", why);
    printLimits(lim, out);
    return false;
  }
  lim = next;
  out.printf("%s=%lu%s, sending the limits\n", field->name, v, field->unit);
  return true;
}

}  // namespace commands
