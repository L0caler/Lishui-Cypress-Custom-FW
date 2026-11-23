// commands.h - debug commands over BLE, service mode only.
//
//   limits           show the limits in RAM
//   help             list the commands
//   <name>=<value>   change one limit in RAM and send all limits to the
//                    controller, e.g. erpm=9999
//
// Names: pmax duty ibat iph erpm erpmstart vbatmin vbatstart (units as in
// controller_limits.h: W, 0..240, mA, mA, erpm, erpm, mV, mV).
//
// Nothing is stored: the changed limits stay in RAM, also in normal mode
// after the button is released, until the next power-on, which starts
// again from controller_limits.h.
#pragma once

#include <Print.h>

#include <string>

#include "lishui_link.h"

namespace commands {

// Run one command line.  Returns true when `lim` changed and has to be sent.
bool run(const std::string &line, lishui::Limits &lim, Print &out);

void printLimits(const lishui::Limits &lim, Print &out);

}  // namespace commands
