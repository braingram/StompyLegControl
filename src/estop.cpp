#include "estop.h"

EStop::EStop() {
  _estop = ESTOP_ON;
  _last_estop == ESTOP_ON;
  //pinMode(ESTOP_STATUS_PIN, OUTPUT);
};

byte EStop::set_estop(byte severity) {
  _last_estop = _estop;
  _estop = severity;
  /*
  if (_estop == ESTOP_OFF) {
    digitalWrite(ESTOP_STATUS_PIN, HIGH);
  } else {
    digitalWrite(ESTOP_STATUS_PIN, LOW);
  };
  */
  return severity;
};

byte EStop::get_estop() {
  if ((millis() - _last_beat_time) > (unsigned long)(HEARTBEAT_TIMEOUT)) {
    set_estop(ESTOP_HEARTBEAT);
  };
  return _estop;
};

bool EStop::is_stopped() {
  return get_estop() != ESTOP_OFF;
};

bool EStop::just_stopped() {
  return ((get_estop() == ESTOP_OFF) && (_last_estop != ESTOP_OFF));
};

bool EStop::just_released() {
  return ((get_estop() != ESTOP_OFF) && (_last_estop == ESTOP_OFF));
};

void EStop::set_heartbeat() {
  _last_beat_time = millis();
};
