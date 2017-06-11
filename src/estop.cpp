#include "estop.h"

EStop::EStop() {
  _estop = ESTOP_ON;
};

byte EStop::set_estop(byte severity) {
  _estop = severity;
  return severity;
};

byte EStop::get_estop() {
  if ((millis() - _last_beat_time) > (unsigned long)(HEARTBEAT_TIMEOUT)) {
    set_estop(ESTOP_HEARTBEAT);
  };
  return _estop;
};

bool EStop::check_estop() {
  return get_estop() != ESTOP_OFF;
};

void EStop::set_heartbeat() {
  _last_beat_time = millis();
};
