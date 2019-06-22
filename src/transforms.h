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
    LinearDeadbandTransform(float src_min, float src_deadband_min, float src_mid, float src_deadband_max, float src_max, float dst_min, float dst_mid, float dst_max);

    float get_src_min();
    void set_src_min(float src_min);

    float get_src_deadband_min();
    void set_src_deadband_min(float src_deadband_min);

    float get_src_mid();
    void set_src_mid(float src_mid);

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
    float _src_mid;
    float _src_deadband_max;
    float _src_max;

    float _dst_min;
    float _dst_mid;
    float _dst_max;

    float _min_slope;
    float _max_slope;

    void compute_slopes();
};


class InterpolatedTransform : public Transform {
  public:
    InterpolatedTransform(float src_min, float src_max, float* dst_pts, int n_dst_pts);

    float get_src_min();
    void set_src_min(float src_min);

    float get_src_max();
    void set_src_max(float src_max);

    void set_dst_pts(float* dst_pts, int n_dst_pts);

    float src_to_dst(float src_value);
    float dst_to_src(float dst_value);

  private:
    float _src_min;
    float _src_max;
    float* _dst_pts;
    int _n_dst_pts;
};


class JointAngleTransform : public Transform {
  public:
    JointAngleTransform(float a, float b, float zero);

    float get_a();
    float get_b();
    float get_zero();

    void set_a(float a);
    void set_b(float b);
    void set_zero(float zero);

    float src_to_dst(float src_value);
    float length_to_angle(float length);

    float dst_to_src(float dst_value);
    float angle_to_length(float angle);


  private:
    void _compute_ab();
    float _zero;
    float _a;
    float _b;
    float _ab2;
    float _2ab;
};

/*
 * Rather than do perfect trig, the 4-bar linkage
 * made up of the ride height sensor can be approximated
 * with a linear transform with <1% error
 * the final angle to spring length (and leg load) then gives
 * approximately 30 lbs of error
 *
 *
 */
class CalfLoadTransform : public Transform {
  public:
    CalfLoadTransform(
        float a, float b, float slope, float offset,
        float base_length, float inches_to_lbs);

    float get_a();
    float get_b();
    float get_base_length();
    float get_inches_to_lbs();
    float get_slope();
    float get_offset();
    float get_spring_length();
    float get_compression();

    void set_a(float a);
    void set_b(float b);
    void set_base_length(float base_length);
    void set_inches_to_lbs(float inches_to_lbs);
    void set_slope(float slope);
    void set_offset(float offset);

    float src_to_dst(float src_value);
    float value_to_load(float value);

    float dst_to_src(float dst_value);
    float load_to_value(float load);

  private:
    float _slope;  // sensor to angle slope
    float _offset;
    float _sl;
    float _compression;
    void _compute_ab();
    float _a;
    float _b;
    float _ab2;
    float _2ab;
    float _base_length;
    float _inches_to_lbs;
};

#endif
