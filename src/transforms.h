/*
	transforms.h - Library for transforming sensor readings etc.
	Created by Brett Graham, May 8, 2017.
	Released into the public domain -- so help you God.
*/


#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include "Arduino.h"


class Transform {
  public:
    virtual float src_to_dst(float src_value) = 0;
    virtual float dst_to_src(float dst_value) = 0;
};


class LinearTransform : public Transform {
  public:
    LinearTransform(float src_min, float src_max, float dst_min, float dst_max);

    float get_src_min();
    void set_src_min(float src_min);

    float get_src_max();
    void set_src_max(float src_max);

    float get_dst_min();
    void set_dst_min(float dst_min);

    float get_dst_max();
    void set_dst_max(float dst_max);

    float src_to_dst(float src_value);
    float dst_to_src(float dst_value);

  private:
    float _src_min;
    float _src_max;
    float _dst_min;
    float _dst_max;

    float _slope;

    void compute_slope();
};


class LinearDeadbandTransform : public Transform {
  public:
    LinearDeadbandTransform(float src_min, float src_deadband_min, float src_deadband_max, float src_max, float dst_min, float dst_mid, float dst_max);

    float get_src_min();
    void set_src_min(float src_min);

    float get_src_deadband_min();
    void set_src_deadband_min(float src_deadband_min);

    float get_src_max();
    void set_src_max(float src_max);

    float get_src_deadband_max();
    void set_src_deadband_max(float src_deadband_max);

    float get_dst_min();
    void set_dst_min(float dst_min);

    float get_dst_mid();
    void set_dst_mid(float dst_mid);

    float get_dst_max();
    void set_dst_max(float dst_max);

    float src_to_dst(float src_value);
    float dst_to_src(float dst_value);

  private:
    float _src_min;
    float _src_deadband_min;
    float _src_deadband_max;
    float _src_max;

    float _dst_min;
    float _dst_mid;
    float _dst_max;

    float _min_slope;
    float _max_slope;

    void compute_slopes();
};

#endif
