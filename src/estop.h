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

#define ESTOP_ON 2
#define ESTOP_HEARTBEAT 2

#define HEARTBEAT_TIMEOUT 1000  // milliseconds


class EStop {
  public:
    // TODO give this access to enable pins?
    EStop();

    byte set_estop(byte severity);
    byte get_estop();
    bool is_stopped();

    void set_heartbeat();
  private:
    unsigned long _last_beat_time;
    byte _estop;
};

#endif
