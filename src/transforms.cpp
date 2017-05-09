#include "transforms.h"


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

LinearDeadbandTransform::LinearDeadbandTransform(float src_min, float src_deadband_min, float src_deadband_max, float src_max, float dst_min, float dst_mid, float dst_max) {
  _src_min = src_min;
  _src_deadband_min = src_deadband_min;
  _src_deadband_max = src_deadband_max;
  _src_max = src_max;
  _dst_min = dst_min;
  _dst_mid = dst_mid;
  _dst_max = dst_max;
  compute_slopes();
}

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

float LinearDeadbandTransform::get_src_max() {
  return _src_max;
}

void LinearDeadbandTransform::set_src_max(float src_max) {
  _src_max = src_max;
  compute_slopes();
}

float LinearDeadbandTransform::get_src_deadband_max() {
  return _src_deadband_max;
}

void LinearDeadbandTransform::set_src_deadband_max(float src_deadband_max) {
  _src_deadband_max = src_deadband_max;
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
  } else if (dst_value == _dst_mid) {  // TODO this isn't defined
    return (_src_deadband_min + _src_deadband_max) / 2.;
  } else if (dst_value < _dst_mid) {  // below deadband
    return (dst_value - _dst_min) / _min_slope + _src_min;
  } else {  // above deadband
    return (dst_value - _dst_mid) / _max_slope + _src_deadband_max;
  }
}
