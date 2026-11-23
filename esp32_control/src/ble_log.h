// ble_log.h - text log over Bluetooth LE, for "Serial Bluetooth Terminal"
// (Kai Morich) or any other Nordic UART Service (NUS) client.
//
// Only on in service mode (push button held from power-on until released).
// The ESP32 advertises as BLE_NAME (platformio.ini) at the lowest transmit
// power (-12 dBm), so it only works next to the bike.  In the app:
// Devices -> Bluetooth LE -> BLE_NAME.
//
// Locked on every connection: nothing is sent and nothing is answered until
// the first line typed is BLE_PASSWORD (answered with "ok").  Three wrong
// lines drop the connection.
#pragma once

#include <Print.h>

#include <string>

namespace blelog {

void begin();  // start BLE and advertise
void loop();   // send buffered text; call every loop
void end();    // disconnect, radio off; log() then only writes to USB

// The next line the phone sent (without the line end), if there is one.
// The app must send a line end: Serial Bluetooth Terminal does (CR+LF).
bool readLine(std::string &line);

// Text log: USB serial and, while a phone is connected, BLE.
Print &log();

}  // namespace blelog
