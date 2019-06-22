#include "transforms.h"
#include <math.h>


/* ========================================================
 *                      LinearTransform
 * ========================================================*/

LinearTransform::LinearTransform(float src_min, float src_max, float dst_min, float dst_max) {
  _src_min = src_min;
  _src_max = src_max;
  _dst_min = dst_min;
  _dst_max = dst_max;
  compute_slope();
}

float LinearTransform::get_src_min() {
  return _src_min;
}

void LinearTransform::set_src_min(float src_min) {
  _src_min = src_min;
  compute_slope();
}

float LinearTransform::get_src_max() {
  return _src_max;
}

void LinearTransform::set_src_max(float src_max) {
  _src_max = src_max;
  compute_slope();
}

float LinearTransform::get_dst_min() {
  return _dst_min;
}

void LinearTransform::set_dst_min(float dst_min) {
  _dst_min = dst_min;
  compute_slope();
}

float LinearTransform::get_dst_max() {
  return _dst_max;
}

void LinearTransform::set_dst_max(float dst_max) {
  _dst_max = dst_max;
  compute_slope();
}

float LinearTransform::src_to_dst(float src_value) {
  if (src_value <= _src_min) {
    return _dst_min;
  } else if (src_value >= _src_max) {
    return _dst_max;
  } else {
    return (src_value - _src_min) * _slope + _dst_min;
  }
}

float LinearTransform::dst_to_src(float dst_value) {
  if (dst_value <= _dst_min) {
    return _src_min;
  } else if (dst_value >= _dst_max) {
    return _src_max;
  } else {
    return (dst_value - _dst_min) / _slope + _src_min;
  }
}

void LinearTransform::compute_slope() {
  _slope = (_dst_max - _dst_min) / (_src_max - _src_min);
}


/* ========================================================
 *                LinearDeadbandTransform
 * ========================================================*/

LinearDeadbandTransform::LinearDeadbandTransform(float src_min, float src_deadband_min, float src_mid, float src_deadband_max, float src_max, float dst_min, float dst_mid, float dst_max) {
  _src_min = src_min;
  _src_deadband_min = src_deadband_min;
  _src_mid = src_mid;
  _src_deadband_max = src_deadband_max;
  _src_max = src_max;
  _dst_min = dst_min;
  _dst_mid = dst_mid;
  _dst_max = dst_max;
  compute_slopes();
}

LinearDeadbandTransform::LinearDeadbandTransform(float src_min, float src_deadband_min, float src_deadband_max, float src_max, float dst_min, float dst_mid, float dst_max) : LinearDeadbandTransform(src_min, src_deadband_min, (src_deadband_max + src_deadband_min) / 2., src_deadband_max, src_max, dst_min, dst_mid, dst_max) {
};
 
float LinearDeadbandTransform::get_src_min() {
  return _src_min;
}

void LinearDeadbandTransform::set_src_min(float src_min) {
  _src_min = src_min;
  compute_slopes();
}

float LinearDeadbandTransform::get_src_deadband_min() {
  return _src_deadband_min;
}

void LinearDeadbandTransform::set_src_deadband_min(float src_deadband_min) {
  _src_deadband_min = src_deadband_min;
  compute_slopes();
}

float LinearDeadbandTransform::get_src_mid() {
  return _src_mid;
}

void LinearDeadbandTransform::set_src_mid(float src_mid) {
  _src_mid = src_mid;
  // compute_slopes();
}

float LinearDeadbandTransform::get_src_deadband_max() {
  return _src_deadband_max;
}

void LinearDeadbandTransform::set_src_deadband_max(float src_deadband_max) {
  _src_deadband_max = src_deadband_max;
  compute_slopes();
}

float LinearDeadbandTransform::get_src_max() {
  return _src_max;
}

void LinearDeadbandTransform::set_src_max(float src_max) {
  _src_max = src_max;
  compute_slopes();
}

float LinearDeadbandTransform::get_dst_min() {
  return _dst_min;
}

void LinearDeadbandTransform::set_dst_min(float dst_min) {
  _dst_min = dst_min;
  compute_slopes();
}

float LinearDeadbandTransform::get_dst_mid() {
  return _dst_mid;
}

void LinearDeadbandTransform::set_dst_mid(float dst_mid) {
  _dst_mid = dst_mid;
  compute_slopes();
}

float LinearDeadbandTransform::get_dst_max() {
  return _dst_max;
}

void LinearDeadbandTransform::set_dst_max(float dst_max) {
  _dst_max = dst_max;
  compute_slopes();
}

void LinearDeadbandTransform::compute_slopes() {
  _min_slope = (_dst_mid - _dst_min) / (_src_deadband_min - _src_min);
  _max_slope = (_dst_max - _dst_mid) / (_src_max - _src_deadband_max);
}

float LinearDeadbandTransform::src_to_dst(float src_value) {
  if (src_value <= _src_min) {
    return _dst_min;
  } else if (src_value >= _src_max) {
    return _dst_max;
  } else if (src_value < _src_deadband_min) {  // below deadband
    return (src_value - _src_min) * _min_slope + _dst_min;
  } else if (src_value > _src_deadband_max) {  // above deadband
    return (src_value - _src_deadband_max) * _max_slope + _dst_mid;
  } else {  // deadband
    return _dst_mid;
  }
}

float LinearDeadbandTransform::dst_to_src(float dst_value) {
  if (dst_value <= _dst_min) {
    return _src_min;
  } else if (dst_value >= _dst_max) {
    return _src_max;
  } else if (dst_value == _dst_mid) {
    return _src_mid;
  } else if (dst_value < _dst_mid) {  // below deadband
    return (dst_value - _dst_min) / _min_slope + _src_min;
  } else {  // above deadband
    return (dst_value - _dst_mid) / _max_slope + _src_deadband_max;
  }
}


/* ========================================================
 *                  InterpolatedTransform
 * ========================================================*/
InterpolatedTransform::InterpolatedTransform(float src_min, float src_max, float* dst_pts, int n_dst_pts) {
  _src_min = src_min;
  _src_max = src_max;
  _dst_pts = dst_pts;
  _n_dst_pts = n_dst_pts;
}

float InterpolatedTransform::get_src_min() {
  return _src_min;
}

void InterpolatedTransform::set_src_min(float src_min) {
  _src_min = src_min;
}

float InterpolatedTransform::get_src_max() {
  return _src_max;
}

void InterpolatedTransform::set_src_max(float src_max) {
  _src_max = src_max;
}

void InterpolatedTransform::set_dst_pts(float* dst_pts, int n_dst_pts) {
  _dst_pts = dst_pts;
  _n_dst_pts = n_dst_pts;
}

float InterpolatedTransform::src_to_dst(float src_value) {
  if (src_value <= _src_min) return _dst_pts[0];
  if (src_value >= _src_max) return _dst_pts[_n_dst_pts - 1];
  // convert src from src to 0 to n
  float fi = (src_value - _src_min) / (_src_max - _src_min) * (_n_dst_pts - 1);
  // find closest, to the left, index
  int i = round(fi - 0.5);
  // compute offset from that index
  float o = fi - i;

  // compute result using index and offset
  return _dst_pts[i] + (_dst_pts[i+1] - _dst_pts[i]) * o;
}

float InterpolatedTransform::dst_to_src(float dst_value) {
  // only really works if function is 1-to-1
  if (dst_value <= _dst_pts[0]) return _src_min;
  if (dst_value >= _dst_pts[_n_dst_pts - 1]) return _src_max;
  int i;
  for (i=0; i<_n_dst_pts; i++) {
    if (_dst_pts[i] > dst_value) {
      break;
    }
  }
  /*
  float d0 = _dst_pts[i-1];
  float d1 = _dst_pts[i];
  float fo = (dst_value - d0) / (d1 - d0) / (_n_dst_pts - 1.0);
  float fi = (i - 1.0) / (_n_dst_pts - 1.0);
  return (fi + fo) * (_src_max - _src_min) + _src_min;
  */
  return (
      (i - 1.0) / (_n_dst_pts - 1.0) +
      ((dst_value - _dst_pts[i-1]) / (_dst_pts[i] - _dst_pts[i-1])
        / (_n_dst_pts - 1.0))
      ) * (_src_max - _src_min) + _src_min;
}


/* ========================================================
 *                 JointAngleTransform
 * ========================================================*/

JointAngleTransform::JointAngleTransform(float a, float b, float zero) {
  _a = a;
  _b = b;
  _zero = zero;
  _compute_ab();
}

float JointAngleTransform::get_a() {
  return _a;
}

float JointAngleTransform::get_b() {
  return _b;
}

float JointAngleTransform::get_zero() {
  return _zero;
}

void JointAngleTransform::set_a(float a) {
  _a = a;
  _compute_ab();
}

void JointAngleTransform::set_b(float b) {
  _b = b;
  _compute_ab();
}

void JointAngleTransform::set_zero(float zero) {
  _zero = zero;
}

float JointAngleTransform::src_to_dst(float src_value) {
  // length to angle
  // cos(C) = (a * a + b * b - c * c) / (2 * a * b)
  return acosf((_ab2 - (src_value * src_value)) / _2ab) - _zero;
}

float JointAngleTransform::length_to_angle(float length) {
  return src_to_dst(length);
}

float JointAngleTransform::dst_to_src(float dst_value) {
  // angle to length
  // c * c = a * a + b * b - (2 * a * b) * cos(C)
  return sqrtf(_ab2 - _2ab * cosf(dst_value + _zero));
}

float JointAngleTransform::angle_to_length(float angle) {
  return dst_to_src(angle);
}

void JointAngleTransform::_compute_ab() {
  _ab2 = _a * _a + _b * _b;
  _2ab = 2 * _a * _b;
}


/* ========================================================
 *                 CalfLoadTransform
 * ========================================================*/

CalfLoadTransform::CalfLoadTransform(
    float a, float b, float slope, float offset,
    float base_length, float inches_to_lbs) {
  _a = a;
  _b = b;
  _slope = slope;
  _sl = -1.0;  // start as invalid
  _offset = offset;
  _base_length = base_length;
  _inches_to_lbs = inches_to_lbs;
  _compute_ab();
}

float CalfLoadTransform::get_a() {
  return _a;
}

float CalfLoadTransform::get_b() {
  return _b;
}

float CalfLoadTransform::get_base_length() {
  return _base_length;
}

float CalfLoadTransform::get_inches_to_lbs() {
  return _inches_to_lbs;
}

float CalfLoadTransform::get_slope() {
  return _slope;
}

float CalfLoadTransform::get_offset() {
  return _offset;
}

float CalfLoadTransform::get_spring_length() {
  return _sl;
}

float CalfLoadTransform::get_compression() {
  return _compression;
}

void CalfLoadTransform::set_a(float a) {
  _a = a;
  _compute_ab();
}

void CalfLoadTransform::set_b(float b) {
  _b = b;
  _compute_ab();
}

void CalfLoadTransform::set_base_length(float base_length) {
  _base_length = base_length;
}

void CalfLoadTransform::set_inches_to_lbs(float inches_to_lbs) {
  _inches_to_lbs = inches_to_lbs;
}

void CalfLoadTransform::set_slope(float slope) {
  _slope = slope;
}

void CalfLoadTransform::set_offset(float offset) {
  _offset = offset;
}

float CalfLoadTransform::src_to_dst(float src_value) {
  // linear model for 4 bar linkage
  // upper triangle angle to opposite side
  _sl = sqrtf(_ab2 - _2ab * cosf((src_value * _slope + _offset)));
  _compression = (_base_length - _sl);
  // convert spring length change to lbs
  return _compression * _inches_to_lbs;
}

float CalfLoadTransform::value_to_load(float value) {
  return src_to_dst(value);
}

float CalfLoadTransform::dst_to_src(float dst_value) {
  float sl = _base_length - dst_value / _inches_to_lbs;
  return (acosf((_ab2 - sl * sl) / _2ab) - _offset) / _slope;
}

float CalfLoadTransform::load_to_value(float load) {
  return dst_to_src(load);
}

void CalfLoadTransform::_compute_ab() {
  _ab2 = _a * _a + _b * _b;
  _2ab = 2 * _a * _b;
}
