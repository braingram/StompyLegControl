/*
	pid.h - PID library.
	Created by Brett Graham, June 12, 2017.
	Released into the public domain -- so help you God.
*/

#ifndef PID_H
#define PID_H

#include "Arduino.h"


class PID {
  public:
    PID(float p, float i, float d);

    void set_output_limits(float min_output, float max_output);

    void set_setpoint(float setpoint);

    float update(float input);
    float get_output();
    float get_error();

    void reset();

    float get_p();
    float get_i();
    float get_d();

    void set_p(float p);
    void set_i(float i);
    void set_d(float d);

  private:
    float _p;
    float _i;
    float _d;

    float _i_term;

    float _min_output;
    float _max_output;

    float _setpoint;
    float _output;
    float _error;
    float _previous_error;

    unsigned long _last_update;
};

#endif
