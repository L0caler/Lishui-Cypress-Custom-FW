#include "lishui_link.h"

namespace lishui {

static uint8_t sum(const uint8_t *p, size_t n) {
  uint8_t s = 0;
  while (n--) s += *p++;
  return s;
}

static uint16_t le16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

size_t Parser::frameSize(uint8_t type) {
  if (type == STATUS_TYPE) return STATUS_SIZE;
  if (type == INPUTS_TYPE) return INPUTS_SIZE;
  if (type == LIMITS_ACK_TYPE) return LIMITS_ACK_SIZE;
  return 0;
}

// Same resync rule as the controller's own parser: wait for AA, then a known
// type, otherwise start over (AA AA keeps the second AA as a new start).
Parser::Result Parser::feed(uint8_t c) {
  if (pos_ == 0) {
    if (c == SYNC) buf_[pos_++] = c;
    return NONE;
  }
  if (pos_ == 1) {
    if (frameSize(c)) {
      buf_[pos_++] = c;
    } else {
      pos_ = (c == SYNC) ? 1 : 0;
    }
    return NONE;
  }
  buf_[pos_++] = c;
  if (pos_ < frameSize(buf_[1])) return NONE;
  pos_ = 0;
  return decode();
}

Parser::Result Parser::decode() {
  size_t n = frameSize(buf_[1]);
  if (sum(buf_, n - 1) != buf_[n - 1]) {
    badFrames_++;
    return NONE;
  }
  const uint8_t *p = buf_ + 2;
  if (buf_[1] == INPUTS_TYPE) {
    inputs_.throttle = p[0];
    inputs_.brake = p[1];
    inputs_.pasActive = p[2] != 0;
    return INPUTS;
  }
  if (buf_[1] == LIMITS_ACK_TYPE) {
    limitsAck_ = p[0];
    return LIMITS_ACK;
  }
  status_.enabled = p[0] != 0;
  status_.duty = p[1];
  status_.limiting = p[2];
  status_.hallFault = p[3] != 0;
  status_.ibat_mA = (int16_t)le16(p + 4);
  status_.vbat_mV = le16(p + 6);
  status_.erpm = le16(p + 8);
  status_.iph_mA = le16(p + 10);
  status_.currentRel = p[12];
  status_.pasActive = p[13] != 0;
  status_.pasRpm = le16(p + 14);
  status_.throttle = p[16];
  status_.brake = p[17];
  status_.stackFree = p[18];
  return STATUS;
}

size_t makeCurrentRel(uint8_t rel, uint8_t *out) {
  out[0] = SYNC;
  out[1] = CURRENT_REL_TYPE;
  out[2] = rel;
  out[3] = sum(out, 3);
  return CURRENT_REL_SIZE;
}

static void putLe16(uint8_t *p, uint16_t w) {
  p[0] = (uint8_t)w;
  p[1] = (uint8_t)(w >> 8);
}

// Same field order as the controller's config frame, offsets 2..16.
size_t makeLimits(const Limits &l, uint8_t *out) {
  out[0] = SYNC;
  out[1] = LIMITS_TYPE;
  putLe16(out + 2, l.pmax_W);
  out[4] = l.dutyMax;
  putLe16(out + 5, l.ibatMax_mA);
  putLe16(out + 7, l.iphMax_mA);
  putLe16(out + 9, l.erpmMax);
  putLe16(out + 11, l.erpmLimStart);
  putLe16(out + 13, l.vbatMin_mV);
  putLe16(out + 15, l.vbatLimStart_mV);
  out[17] = sum(out, 17);
  out[18] = out[17] ^ 0xFF;
  return LIMITS_SIZE;
}

size_t makeSource(bool external, uint8_t *out) {
  out[0] = SYNC;
  out[1] = SOURCE_TYPE;
  out[2] = external ? 1 : 0;
  out[3] = out[2] ^ 0xFF;
  out[4] = sum(out, 4);
  return SOURCE_SIZE;
}

}  // namespace lishui
