/*
	estop.h - Estop library.
	Created by Brett Graham, June 10, 2017.
	Released into the public domain -- so help you God.

  Combine estop, heartbeat and time?

  cpu -> teensy

  time:
    -> get time, <- report millis

  heartbeat:
    -> heartbeat <- heartbeat

  estop:
    -> estop [severity]
    <- estop [severity]
*/

#ifndef ESTOP_H
#define ESTOP_H

#include "Arduino.h"

#define ESTOP_OFF 0
#define ESTOP_SOFT 1
#define ESTOP_HARD 2
#define ESTOP_HOLD 3
#define ESTOP_SENSOR_LIMIT 4

#define ESTOP_ON 2
#define ESTOP_HEARTBEAT 2

#define ESTOP_DEFAULT 2

#define HEARTBEAT_TIMEOUT 1000  // milliseconds


extern "C" {
  typedef void (*estop_callback) (byte severity);
}


class EStop {
  public:
    // TODO give this access to enable pins?
    // TODO enable emitting signals
    EStop();

    byte set_estop(byte severity);
    byte get_estop();
    bool is_stopped();
    bool just_stopped();
    bool just_released();

    void set_heartbeat();

    void register_callback(estop_callback cb);
  private:
    estop_callback _cb;
    unsigned long _last_beat_time;
    byte _estop;
    byte _last_estop;
};

#endif
