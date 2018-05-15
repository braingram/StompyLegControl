#include "estop.h"

EStop::EStop() {
  _estop = ESTOP_ON;
  _last_estop == ESTOP_ON;
  _cb = NULL;
};

byte EStop::set_estop(byte severity) {
  _last_estop = _estop;
  _estop = severity;
  if (_cb != NULL) {
    (*_cb)(severity);
  };
  return severity;
};

byte EStop::get_estop() {
  //if ((millis() - _last_beat_time) > (unsigned long)(HEARTBEAT_TIMEOUT)) {
  //if (!valid_heartbeat()) {
  //  set_estop(ESTOP_HEARTBEAT);
  //};
  return _estop;
};

bool EStop::is_stopped() {
  return get_estop() != ESTOP_OFF;
};

bool EStop::just_changed() {
  return (get_estop() != _last_estop);
};

bool EStop::just_stopped() {
  return ((get_estop() == ESTOP_OFF) && (_last_estop != ESTOP_OFF));
};

bool EStop::just_released() {
  return ((get_estop() != ESTOP_OFF) && (_last_estop == ESTOP_OFF));
};

void EStop::check_heartbeat() {
  if (!valid_heartbeat()) {
    set_estop(ESTOP_HEARTBEAT);
  };
};

bool EStop::valid_heartbeat() {
  if ((millis() - _last_beat_time) > (unsigned long)(HEARTBEAT_TIMEOUT)) {
    return false;
  };
  return true;
};

void EStop::set_heartbeat() {
  _last_beat_time = millis();
};

void EStop::register_callback(estop_callback cb) {
  _cb = cb;
};
