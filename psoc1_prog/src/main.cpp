/* Cypress PSoC 1 ISSP programmer firmware for the Arduino Mega 2560.
 * See README.md for wiring and the serial protocol.
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "devices.h"

#if !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__)
#error "This firmware needs an Arduino Mega: monitor mode uses USART2."
#endif

/* ------------------------------------------------------------------------- *
 * Pin map - everything on PORTH
 *
 * SCLK and SDATA must share a port: data_send() toggles both with one
 * write.  PH0/PH1 are also RXD2/TXD2, so monitor mode can bridge the same
 * two wires through the hardware USART2.
 *
 *   SDATA = target P1[0] = target TX  ->  D17  PH0  RXD2
 *   SCLK  = target P1[1] = target RX  <-  D16  PH1  TXD2
 *   XRES                              <-  D6   PH3
 *   Vdd switch                        <-  D7   PH4
 * ------------------------------------------------------------------------- */

#define ISSP_PORT PORTH
#define ISSP_DDR  DDRH
#define ISSP_IN   PINH
#define SDA_PIN   BIT0
#define SCL_PIN   BIT1
#define RES_PIN   BIT3
#define PWR_PIN   BIT4

#define LED_PORT PORTB
#define LED_DDR  DDRB
#define LED_PIN  BIT7 /* D13, on board LED */

/* Define PWR_ACTIVE_LOW when the Vdd switch is a high side p-channel MOSFET,
 * so that a low pin powers the target. */
#ifdef PWR_ACTIVE_LOW
#define TARGET_POWER_ON  ISSP_PORT &= ~PWR_PIN
#define TARGET_POWER_OFF ISSP_PORT |= PWR_PIN
#else
#define TARGET_POWER_ON  ISSP_PORT |= PWR_PIN
#define TARGET_POWER_OFF ISSP_PORT &= ~PWR_PIN
#endif

#define LED_ON  LED_PORT |= LED_PIN
#define LED_OFF LED_PORT &= ~LED_PIN

#define DELAY for(i=0;i<1;i++);
#define _CLK1 ISSP_PORT |= SCL_PIN; DELAY
#define _CLK0 ISSP_PORT &= ~SCL_PIN; DELAY

/* ------------------------------------------------------------------------- *
 * Settings
 * ------------------------------------------------------------------------- */

#define INIT_MODE_AUTO  0 /* try XRES, fall back to a power cycle    */
#define INIT_MODE_RESET 1 /* XRES pulse (needs the XRES pin wired)   */
#define INIT_MODE_POWER 2 /* power cycle (for packages without XRES) */

#define TVDDWAIT_US 500    /* default delay from Vdd on to polling SDATA */
#define POWER_OFF_MS 500   /* hold-down time for the target supply       */

/* Bound every wait on SDATA (about a second at 16 MHz). */
#define SDATA_TIMEOUT 2000000UL

#define MON_BAUD 19200UL  /* default monitor line rate */
#define MON_ESCAPE 0x1D   /* Ctrl-], leaves monitor mode */

const uint16_t fw = 0x0005;  /* firmware version */

volatile uint16_t i;
uint16_t dev_id = 0;
int8_t dev_index = -1;                 /* index into psoc_devices, -1 if unknown */
uint16_t blocks_override = 0;          /* 0 = use the device table */
uint8_t init_mode = INIT_MODE_AUTO;
uint8_t active_mode = INIT_MODE_RESET; /* mode that last acquired the target */
uint8_t vdd_5v = 1;                    /* which INITIALIZE-3 vector to send */
uint8_t pull_timeouts = 0;             /* sticky count of WAIT-AND-POLL timeouts */
uint16_t tvddwait_us = TVDDWAIT_US;
uint32_t mon_baud = MON_BAUD;

uint8_t ram_blk[64];

/* ------------------------------------------------------------------------- *
 * Bit banging
 * ------------------------------------------------------------------------- */

/* Clock out len bits, MSB first.  The clock is low on entry and exit. */
void data_send(const uint8_t *data, uint16_t len) {
  uint8_t prev = 0;

  ISSP_DDR |= SDA_PIN | SCL_PIN;
  ISSP_PORT |= SDA_PIN;  /* SDATA starts high */
  LED_ON;
  for (uint16_t k = 0; k < len; k++) {
    uint8_t curr = (data[k / 8] & (BIT7 >> k % 8)) == 0;
    if (curr != prev) {
      prev = curr;
      ISSP_PORT ^= SDA_PIN | SCL_PIN;
    } else {
      ISSP_PORT ^= SCL_PIN;
    }
    DELAY;
    ISSP_PORT ^= SCL_PIN;
    DELAY;
  }
}

/* Poll SDATA until it reaches the wanted level.  Returns 0 on timeout. */
uint8_t wait_sdata(uint8_t high) {
  uint32_t n = SDATA_TIMEOUT;
  while (((ISSP_IN & SDA_PIN) != 0) != high)
    if (!--n) return 0;
  return 1;
}

void sdata_release() {
  LED_OFF;
  ISSP_PORT &= ~SDA_PIN;
  ISSP_DDR &= ~SDA_PIN;
}

/* WAIT-AND-POLL: wait for the target to finish, then 40 clocks. */
uint8_t data_pull() {
  sdata_release();
  if (!wait_sdata(0)) goto fail;
  _CLK1;
  _CLK0;
  if (!wait_sdata(1)) goto fail;  /* target drives SDATA high ... */
  if (!wait_sdata(0)) goto fail;  /* ... and then low again        */
  for (uint8_t j = 0; j < 40; j++) {
    _CLK1;
    _CLK0;
  }
  return 1;
fail:
  if (pull_timeouts < 255) pull_timeouts++;
  return 0;
}

void data_read(uint8_t *buf, uint8_t bits) {
  sdata_release();
  _CLK1;
  _CLK0;
  for (uint8_t k = 0; k < bits; k++) {
    _CLK1;
    _CLK0;
    if (k % 8 == 0) buf[k / 8] = 0;
    if (ISSP_IN & SDA_PIN) buf[k / 8] |= BIT7 >> k % 8;
  }
  _CLK1;
  _CLK0;
}

uint8_t read_byte(uint8_t addr) {
  uint8_t buf[2] = {(uint8_t)(0b10100000 | (addr >> 3)), (uint8_t)(addr << 5)};
  data_send(buf, 11);
  data_read(&buf[1], 8);
  data_send(buf, 1);
  return buf[1];
}

/* ------------------------------------------------------------------------- *
 * Device table
 * ------------------------------------------------------------------------- */

int8_t device_lookup(uint16_t id) {
  for (uint8_t k = 0; k < PSOC_DEVICE_COUNT; k++)
    if (pgm_read_word(&psoc_devices[k].id) == id) return (int8_t)k;
  return -1;
}

/* Flash size in 64 byte blocks, 0 when the target is unknown. */
uint16_t device_blocks() {
  if (blocks_override) return blocks_override;
  if (dev_index < 0) return 0;
  return pgm_read_word(&psoc_devices[dev_index].blocks);
}

uint8_t device_flags() {
  if (dev_index < 0) return 0;
  return pgm_read_byte(&psoc_devices[dev_index].flags);
}

/* ------------------------------------------------------------------------- *
 * Line and supply control
 * ------------------------------------------------------------------------- */

/* Float SCLK, SDATA and XRES: before cutting the supply, so nothing back
 * feeds an unpowered chip, and to hand P1[0]/P1[1] to the application. */
void issp_idle() {
  ISSP_DDR &= ~(SCL_PIN | SDA_PIN | RES_PIN);
  ISSP_PORT &= ~(SCL_PIN | SDA_PIN | RES_PIN);
  LED_OFF;
}

/* Drive the ISSP lines, clock and XRES low. */
void issp_drive() {
  ISSP_PORT &= ~(SCL_PIN | RES_PIN);
  ISSP_DDR |= SCL_PIN | SDA_PIN | RES_PIN;
}

/* Cut the target supply and hold it down for a fixed time. */
void power_off() {
  issp_idle();
  TARGET_POWER_OFF;
  delay(POWER_OFF_MS);
}

void xres_pulse() {
  issp_drive();
  ISSP_PORT |= RES_PIN;
  for (i = 0; i < 500; i++);   /* Txres, 10 us minimum */
  ISSP_PORT &= ~RES_PIN;
}

/* Power cycle the target and catch the high-to-low edge it puts on SDATA
 * once it is ready for programming.  Returns 0 if the edge never came. */
uint8_t power_cycle() {
  power_off();
  TARGET_POWER_ON;
  /* Do not wait for the rail: the target raises SDATA at its POR trip and
   * drops it when ready (2.6 ms / 7.1 ms on the Lishui board), and waiting
   * for a settled rail risks missing that fall. */
  delayMicroseconds(tvddwait_us);
  return wait_sdata(1) && wait_sdata(0);
}

/* Restart the target and give P1[0]/P1[1] back to the application. */
void reset_device() {
  if (active_mode == INIT_MODE_POWER) {
    power_off();
    TARGET_POWER_ON;
  } else {
    xres_pulse();
  }
  issp_idle();
}

/* ------------------------------------------------------------------------- *
 * Programming primitives
 * ------------------------------------------------------------------------- */

void init_vectors() {
  data_send(init1, init1_len);
  data_pull();
  data_send(init2, init2_len);
  data_pull();
  if (vdd_5v)
    data_send(init3_5V, init3_5V_len);
  else
    data_send(init3, init3_len);
}

uint16_t read_id() {
  data_send(id_setup, id_setup_len);
  data_pull();
  return (read_byte(0b11111000) << 8) | read_byte(0b11111001);
}

/* Patch the BLOCKID of CHECKSUM-SETUP, the number of blocks the SROM sums. */
void csum_set_blocks(uint16_t blocks) {
  uint8_t b = (uint8_t)blocks;
  checksum_setup[CSUM_BLOCKID_BYTE] =
      (checksum_setup[CSUM_BLOCKID_BYTE] & 0x80) | (b >> 1);
  checksum_setup[CSUM_BLOCKID_BYTE + 1] =
      (checksum_setup[CSUM_BLOCKID_BYTE + 1] & 0x7F) | ((b & 1) << 7);
}

uint16_t read_checksum() {
  csum_set_blocks(device_blocks());  /* unknown target: 0 = 256 blocks */
  data_send(checksum_setup, checksum_setup_len);
  data_pull();
  return (read_byte(0b11111001) << 8) | read_byte(0b11111000);
}

void erase_all_memory() {
  data_send(bulk_erase, bulk_erase_len);
  data_pull();
}

/* WRITE-BYTE for all 64 bytes of ram_blk:  1001 0aaa  aaad dddd  ddd1 11xx */
void write_ram_block() {
  uint8_t payload[3];
  for (uint8_t addr = 0; addr < 64; addr++) {
    uint8_t data = ram_blk[addr];
    payload[0] = 0b10010000 | (addr >> 3);
    payload[1] = (addr << 5) | (data >> 3);
    payload[2] = (data << 5) | 0b00011100;
    data_send(payload, 22);
  }
}

/* SET-BLOCK-NUM:  1001 1111  010d dddd  ddd1 11xx */
void set_block_num(uint8_t block) {
  uint8_t payload[3] = {0b10011111, (uint8_t)(0b01000000 | (block >> 3)),
                        (uint8_t)(0b00011100 | (block << 5))};
  data_send(payload, 22);
}

void write_block(uint8_t block) {
  write_ram_block();
  set_block_num(block);
  if (device_flags() & DEV_PROGBLK_27X43)
    data_send(program_block_1, program_block_len);
  else
    data_send(program_block_0, program_block_len);
  data_pull();
}

void write_secure() {
  write_ram_block();
  data_send(secure, secure_len);
  data_pull();
}

void read_block(uint8_t block, uint8_t offset) {
  set_block_num(block);
  data_send(verify_setup, verify_setup_len);
  data_pull();
  for (uint8_t addr = 0; addr < 64; addr++)
    ram_blk[addr] = read_byte(addr + offset);
}

/* ------------------------------------------------------------------------- *
 * Acquire the target
 * ------------------------------------------------------------------------- */

uint16_t acquire_reset() {
  active_mode = INIT_MODE_RESET;
  TARGET_POWER_ON;
  delay(10);   /* in case we switch Vdd, let it settle */
  xres_pulse();
  init_vectors();
  return read_id();
}

uint16_t acquire_power() {
  active_mode = INIT_MODE_POWER;
  if (!power_cycle()) return 0;
  issp_drive();
  init_vectors();   /* Tacq allows 3 ms between the edge and INITIALIZE-1 */
  return read_id();
}

uint16_t initialise() {
  if (init_mode == INIT_MODE_POWER) {
    dev_id = acquire_power();
  } else {
    dev_id = acquire_reset();
    /* Auto mode and no known ID: the package may have no XRES pin (the
     * CY8C24533 has none), so try a power cycle, and if that finds nothing
     * either go back to reset mode so the result is repeatable. */
    if (init_mode == INIT_MODE_AUTO && device_lookup(dev_id) < 0) {
      dev_id = acquire_power();
      if (device_lookup(dev_id) < 0 && (dev_id == 0x0000 || dev_id == 0xFFFF))
        dev_id = acquire_reset();
    }
  }
  dev_index = device_lookup(dev_id);
  return dev_id;
}

/* ------------------------------------------------------------------------- *
 * Monitor mode
 *
 * Bridges the target UART on P1[0]/P1[1] to the host port through the
 * hardware USART2, until the host sends MON_ESCAPE.  While USART2 is on it
 * owns PH0/PH1, so no ISSP code may run - the blocking loop ensures that.
 * ------------------------------------------------------------------------- */

void monitor() {
  issp_idle();
  ISSP_PORT |= SDA_PIN;  /* pull-up on RXD2, so an absent target reads idle */
  TARGET_POWER_ON;
  LED_ON;
  Serial2.begin(mon_baud);

  for (;;) {
    while (Serial2.available()) Serial.write(Serial2.read());
    if (Serial.available()) {
      int b = Serial.read();
      if (b == MON_ESCAPE) break;
      Serial2.write(b);
    }
  }

  Serial2.flush();
  Serial2.end();
  issp_idle();
}

/* ------------------------------------------------------------------------- *
 * Host protocol - one byte per command, every reply ends with "\r\n> "
 * ------------------------------------------------------------------------- */

void write16(uint16_t v) {
  Serial.write((uint8_t)v);
  Serial.write((uint8_t)(v >> 8));
  Serial.println();
}

uint16_t read16() {
  uint8_t args[2] = {0, 0};
  Serial.readBytes(args, 2);
  return args[0] | ((uint16_t)args[1] << 8);
}

void setup() {
  LED_DDR |= LED_PIN;
  LED_OFF;
  ISSP_DDR |= PWR_PIN;
  TARGET_POWER_ON;
  Serial.begin(115200);
  Serial.print("> ");
}

void loop() {
  uint8_t c, arg;

  if (Serial.readBytes(&c, 1) == 0) return;

  switch (c) {
    case 'i':  /* reinitialise using the current mode */
      initialise();
      Serial.println();
      break;
    case 'I':  /* reinitialise, next byte selects the mode */
      if (Serial.readBytes(&arg, 1) > 0 && arg <= INIT_MODE_POWER) {
        init_mode = arg;
        initialise();
      }
      Serial.println();
      break;
    case 'd': Serial.println(dev_id, HEX); break;
    case 'D': write16(dev_id); break;
    case 'm': Serial.println(active_mode, HEX); break;
    case 'E':  /* read and clear the WAIT-AND-POLL timeout count */
      Serial.println(pull_timeouts, HEX);
      pull_timeouts = 0;
      break;
    case 'W':  /* Tvddwait in microseconds */
      tvddwait_us = read16();
      Serial.println();
      break;
    case 'n': Serial.println(device_blocks(), HEX); break;
    case 'N': write16(device_blocks()); break;
    case 'B':  /* override the flash size in blocks */
      blocks_override = read16();
      Serial.println();
      break;
    case 'v':  /* supply voltage: 0 = 3 V vectors, 1 = 5 V vectors */
      if (Serial.readBytes(&arg, 1) > 0) vdd_5v = arg ? 1 : 0;
      Serial.println();
      break;
    case 'r': {  /* read a flash block into the ram block: block, offset */
      uint8_t args[2] = {0x00, 0x80};
      Serial.readBytes(args, 2);
      read_block(args[0], args[1]);
      Serial.println();
      break;
    }
    case 'e':
      erase_all_memory();
      Serial.println();
      break;
    case 'w':  /* write the ram block to the flash block in the next byte */
      if (Serial.readBytes(&arg, 1) > 0) write_block(arg);
      Serial.println();
      break;
    case 'x':
      write_secure();
      Serial.println();
      break;
    case 'f': Serial.println(fw, HEX); break;
    case 'F': write16(fw); break;
    case 'c': Serial.println(read_checksum(), HEX); break;
    case 'C': write16(read_checksum()); break;
    case 's':  /* read the ram block */
      Serial.write(ram_blk, 64);
      Serial.println();
      break;
    case 't':  /* write the ram block */
      Serial.readBytes(ram_blk, 64);
      Serial.println();
      break;
    case 'a':
      reset_device();
      Serial.println();
      break;
    case 'k': {  /* monitor line rate as ASCII digits, e.g. k19200<enter> */
      uint32_t baud = 0;
      while (Serial.readBytes(&arg, 1) > 0 && arg >= '0' && arg <= '9')
        baud = baud * 10 + (arg - '0');
      if (baud >= 300 && baud <= 1000000) mon_baud = baud;
      Serial.println(mon_baud, DEC);
      break;
    }
    case 'M':
      Serial.println(mon_baud, DEC);
      monitor();
      Serial.println();
      break;
    case '\n':
    case '\r':
      Serial.println();
      break;
    default:
      Serial.println("invalid");
  }
  Serial.print("> ");
}
