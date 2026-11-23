/* Pin level access for the ESP32.
 *
 * Default pin map, for a classic ESP32 DevKit (WROOM or WROVER).  Every pin
 * can be moved with a -D in platformio.ini.
 *
 *   SDATA = target P1[0] = target TX  <-> GPIO25   ISSP_SDA_GPIO
 *   SCLK  = target P1[1] = target RX  <-  GPIO26   ISSP_SCL_GPIO
 *   XRES                              <-  GPIO27   ISSP_RES_GPIO
 *   Vdd from a GPIO (power mode gpio) <-  GPIO15   ISSP_PWR_GPIO
 *   second Vdd GPIO, optional         <-  -1       ISSP_PWR2_GPIO
 *   supply sense (power mode external)->  GPIO34   ISSP_SENSE_GPIO
 *   LED                                   GPIO2    ISSP_LED_GPIO
 *
 * The pins avoid the flash pins (6-11), GPIO16/17, which WROVER modules use
 * for PSRAM, and the strapping pins that decide how the ESP32 boots (0, 2,
 * 5, 12).  GPIO2 is only the LED, which is fine.  GPIO15 is a strapping pin
 * as well, but it only decides whether the ROM prints its boot log: a target
 * that loads it low at reset just silences that log.  Its weak boot-time
 * pull-up is the only thing the target sees until setup() drives it.
 *
 * The ESP32 is a 3.3 V part and its pins are NOT 5 V tolerant, so the
 * target has to run at 3.3 V, or SDATA and the sense input need level
 * shifting - see README.md.
 */

#ifndef HAL_ESP32_H
#define HAL_ESP32_H

#include <Arduino.h>
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#define HAL_PLATFORM "esp32"

#ifndef ISSP_SDA_GPIO
#define ISSP_SDA_GPIO 25
#endif
#ifndef ISSP_SCL_GPIO
#define ISSP_SCL_GPIO 26
#endif
#ifndef ISSP_RES_GPIO
#define ISSP_RES_GPIO 27
#endif
#ifndef ISSP_PWR_GPIO
#define ISSP_PWR_GPIO 15
#endif
#ifndef ISSP_PWR2_GPIO
#define ISSP_PWR2_GPIO -1  /* a second pin in parallel for more current */
#endif
#ifndef ISSP_SENSE_GPIO
#define ISSP_SENSE_GPIO 34
#endif
#ifndef ISSP_LED_GPIO
#define ISSP_LED_GPIO 2
#endif

/* Half an SCLK period in ns.  1000 gives a 500 kHz clock, about what the
 * Mega manages; the ISSP limit is 8 MHz, so this can go down a long way. */
#ifndef ISSP_HALF_NS
#define ISSP_HALF_NS 1000
#endif

/* SDATA, SCLK and XRES are switched through the GPIO_OUT/GPIO_ENABLE
 * set/clear registers, which only cover GPIO0..31. */
static_assert(ISSP_SDA_GPIO < 32 && ISSP_SCL_GPIO < 32 && ISSP_RES_GPIO < 32,
              "SDATA, SCLK and XRES must be on GPIO0..31");

/* The ESP32 can only give the target 3.3 V, so default to those vectors. */
#define DEFAULT_VDD_5V 0

#define HAVE_PWR_SENSE (ISSP_SENSE_GPIO >= 0)

#define SDA_MASK (1UL << ISSP_SDA_GPIO)
#define SCL_MASK (1UL << ISSP_SCL_GPIO)
#define RES_MASK (1UL << ISSP_RES_GPIO)

#define SDA_HIGH() REG_WRITE(GPIO_OUT_W1TS_REG, SDA_MASK)
#define SDA_LOW()  REG_WRITE(GPIO_OUT_W1TC_REG, SDA_MASK)
#define SDA_OUT()  REG_WRITE(GPIO_ENABLE_W1TS_REG, SDA_MASK)
#define SDA_IN()   REG_WRITE(GPIO_ENABLE_W1TC_REG, SDA_MASK)
#define SDA_READ() ((REG_READ(GPIO_IN_REG) & SDA_MASK) != 0)
#define SCL_HIGH() REG_WRITE(GPIO_OUT_W1TS_REG, SCL_MASK)
#define SCL_LOW()  REG_WRITE(GPIO_OUT_W1TC_REG, SCL_MASK)
#define SCL_OUT()  REG_WRITE(GPIO_ENABLE_W1TS_REG, SCL_MASK)
#define RES_HIGH() REG_WRITE(GPIO_OUT_W1TS_REG, RES_MASK)
#define RES_LOW()  REG_WRITE(GPIO_OUT_W1TC_REG, RES_MASK)

static uint32_t hal_half_cycles = 240;

static inline void hal_issp_delay() {
  uint32_t start = ESP.getCycleCount();
  while (ESP.getCycleCount() - start < hal_half_cycles) {}
}
#define ISSP_DELAY() hal_issp_delay()

#if ISSP_LED_GPIO >= 0
#define LED_ON()  digitalWrite(ISSP_LED_GPIO, HIGH)
#define LED_OFF() digitalWrite(ISSP_LED_GPIO, LOW)
#else
#define LED_ON()  do { } while (0)
#define LED_OFF() do { } while (0)
#endif

/* On a single core part the loop task shares the CPU with the idle task,
 * whose watchdog fires if long waits never yield. */
#if CONFIG_FREERTOS_UNICORE
#define HAL_IDLE() delay(1)
#else
#define HAL_IDLE() do { } while (0)
#endif

/* Route SDATA/SCLK/XRES to plain GPIO, outputs disabled, no pulls.  Also
 * takes them back from the UART after monitor mode. */
static inline void hal_issp_pins_init() {
  const uint8_t pins[] = {ISSP_SDA_GPIO, ISSP_SCL_GPIO, ISSP_RES_GPIO};
  for (uint8_t p : pins) {
    gpio_set_level((gpio_num_t)p, 0);
    pinMode(p, OUTPUT);  /* connects the pin to the GPIO output register */
    pinMode(p, INPUT);   /* ... and disables the driver again */
  }
}

static inline void hal_init() {
  hal_half_cycles = (uint32_t)getCpuFrequencyMhz() * ISSP_HALF_NS / 1000;
  if (!hal_half_cycles) hal_half_cycles = 1;
#if ISSP_LED_GPIO >= 0
  pinMode(ISSP_LED_GPIO, OUTPUT);
#endif
#if ISSP_SENSE_GPIO >= 0
  pinMode(ISSP_SENSE_GPIO, INPUT);
#endif
  hal_issp_pins_init();
}

/* Float SCLK, SDATA and XRES. */
static inline void hal_issp_idle() {
  REG_WRITE(GPIO_ENABLE_W1TC_REG, SDA_MASK | SCL_MASK | RES_MASK);
  REG_WRITE(GPIO_OUT_W1TC_REG, SDA_MASK | SCL_MASK | RES_MASK);
}

/* Drive the ISSP lines, clock and XRES low. */
static inline void hal_issp_drive() {
  REG_WRITE(GPIO_OUT_W1TC_REG, SCL_MASK | RES_MASK);
  REG_WRITE(GPIO_ENABLE_W1TS_REG, SDA_MASK | SCL_MASK | RES_MASK);
}

static inline void hal_xres_delay() { delayMicroseconds(300); }

static inline void hal_power_pin(int pin, uint8_t on) {
  if (pin < 0) return;
#ifdef PWR_ACTIVE_LOW
  on = !on;
#endif
  /* gpio_set_level, not digitalWrite: Arduino-ESP32 3.x ignores
   * digitalWrite on a pin pinMode has not claimed yet, and the level has to
   * be in place before the driver turns on. */
  gpio_set_level((gpio_num_t)pin, on);
  pinMode(pin, OUTPUT);
  /* ~40 mA, the most an ESP32 pin can source */
  gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_CAP_3);
}

/* Power the target from the GPIO(s), or pull its Vdd to ground. */
static inline void hal_power(uint8_t on) {
  hal_power_pin(ISSP_PWR_GPIO, on);
  hal_power_pin(ISSP_PWR2_GPIO, on);
}

/* Float the power GPIOs, so they never fight an external supply. */
static inline void hal_power_release() {
  if (ISSP_PWR_GPIO >= 0) pinMode(ISSP_PWR_GPIO, INPUT);
  if (ISSP_PWR2_GPIO >= 0) pinMode(ISSP_PWR2_GPIO, INPUT);
}

/* Hold SDATA low while it floats, so an unpowered target cannot read as
 * the high it raises at power on. */
static inline void hal_sda_pulldown(uint8_t on) {
  if (on)
    gpio_pulldown_en((gpio_num_t)ISSP_SDA_GPIO);
  else
    gpio_pulldown_dis((gpio_num_t)ISSP_SDA_GPIO);
}

static inline void hal_sda_pullup(uint8_t on) {
  if (on)
    gpio_pullup_en((gpio_num_t)ISSP_SDA_GPIO);
  else
    gpio_pullup_dis((gpio_num_t)ISSP_SDA_GPIO);
}

static inline uint8_t hal_supply_sensed() {
#if ISSP_SENSE_GPIO >= 0
  return digitalRead(ISSP_SENSE_GPIO) == HIGH;
#else
  return 1;
#endif
}

/* UART1 routed to the ISSP pins: RX on SDATA, TX on SCLK. */
#define MON_SERIAL Serial1

static inline void hal_mon_begin(uint32_t baud) {
  Serial1.setRxBufferSize(1024);
  Serial1.begin(baud, SERIAL_8N1, ISSP_SDA_GPIO, ISSP_SCL_GPIO);
  gpio_pullup_en((gpio_num_t)ISSP_SDA_GPIO);  /* absent target reads idle */
}

static inline void hal_mon_end() {
  Serial1.flush();
  Serial1.end();
  gpio_pullup_dis((gpio_num_t)ISSP_SDA_GPIO);
  hal_issp_pins_init();
}

#endif
