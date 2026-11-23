/* Pin level access for the Arduino Mega 2560.
 *
 * Everything ISSP lives on PORTH.  PH0/PH1 are also RXD2/TXD2, so monitor
 * mode can bridge the same two wires through the hardware USART2.
 *
 *   SDATA = target P1[0] = target TX  ->  D17  PH0  RXD2
 *   SCLK  = target P1[1] = target RX  <-  D16  PH1  TXD2
 *   XRES                              <-  D6   PH3
 *   Vdd switch                        <-  D7   PH4
 */

#ifndef HAL_AVR_H
#define HAL_AVR_H

#include <Arduino.h>
#include <avr/io.h>

#if !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__)
#error "This firmware needs an Arduino Mega: monitor mode uses USART2."
#endif

#define HAL_PLATFORM "mega2560"

#define ISSP_PORT PORTH
#define ISSP_DDR  DDRH
#define ISSP_IN   PINH
#define SDA_MASK  _BV(0)
#define SCL_MASK  _BV(1)
#define RES_MASK  _BV(3)
#define PWR_MASK  _BV(4)

#define LED_PORT PORTB
#define LED_DDR  DDRB
#define LED_MASK _BV(7) /* D13, on board LED */

/* The Mega runs the target at 5 V, so default to the 5 V INITIALIZE-3. */
#define DEFAULT_VDD_5V 1

/* No supply sense input: only the switched (GPIO) supply is available. */
#define HAVE_PWR_SENSE 0

static volatile uint16_t hal_i;

#define SDA_HIGH() (ISSP_PORT |= SDA_MASK)
#define SDA_LOW()  (ISSP_PORT &= ~SDA_MASK)
#define SDA_OUT()  (ISSP_DDR |= SDA_MASK)
#define SDA_IN()   (ISSP_DDR &= ~SDA_MASK)
#define SDA_READ() ((ISSP_IN & SDA_MASK) != 0)
#define SCL_HIGH() (ISSP_PORT |= SCL_MASK)
#define SCL_LOW()  (ISSP_PORT &= ~SCL_MASK)
#define SCL_OUT()  (ISSP_DDR |= SCL_MASK)
#define RES_HIGH() (ISSP_PORT |= RES_MASK)
#define RES_LOW()  (ISSP_PORT &= ~RES_MASK)

/* Half an SCLK period, roughly a microsecond at 16 MHz. */
#define ISSP_DELAY() do { for (hal_i = 0; hal_i < 1; hal_i++); } while (0)

#define LED_ON()  (LED_PORT |= LED_MASK)
#define LED_OFF() (LED_PORT &= ~LED_MASK)

/* Nothing else needs the CPU while we busy wait. */
#define HAL_IDLE() do { } while (0)

static inline void hal_init() {
  LED_DDR |= LED_MASK;
  ISSP_DDR |= PWR_MASK;
}

/* Float SCLK, SDATA and XRES. */
static inline void hal_issp_idle() {
  ISSP_DDR &= ~(SCL_MASK | SDA_MASK | RES_MASK);
  ISSP_PORT &= ~(SCL_MASK | SDA_MASK | RES_MASK);
}

/* Drive the ISSP lines, clock and XRES low. */
static inline void hal_issp_drive() {
  ISSP_PORT &= ~(SCL_MASK | RES_MASK);
  ISSP_DDR |= SCL_MASK | SDA_MASK | RES_MASK;
}

/* Txres, 10 us minimum. */
static inline void hal_xres_delay() {
  for (hal_i = 0; hal_i < 500; hal_i++);
}

/* Define PWR_ACTIVE_LOW when the Vdd switch is a high side p-channel MOSFET,
 * so that a low pin powers the target. */
static inline void hal_power(uint8_t on) {
#ifdef PWR_ACTIVE_LOW
  on = !on;
#endif
  if (on)
    ISSP_PORT |= PWR_MASK;
  else
    ISSP_PORT &= ~PWR_MASK;
}

static inline void hal_power_release() {}
static inline void hal_sda_pulldown(uint8_t) {}
/* SDATA is an input while this is used, so the PORT bit is its pull-up. */
static inline void hal_sda_pullup(uint8_t on) {
  if (on)
    ISSP_PORT |= SDA_MASK;
  else
    ISSP_PORT &= ~SDA_MASK;
}
static inline uint8_t hal_supply_sensed() { return 1; }

#define MON_SERIAL Serial2

static inline void hal_mon_begin(uint32_t baud) {
  ISSP_PORT |= SDA_MASK;  /* pull-up on RXD2, so an absent target reads idle */
  Serial2.begin(baud);
}

static inline void hal_mon_end() {
  Serial2.flush();
  Serial2.end();
}

#endif
