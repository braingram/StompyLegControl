#include "transforms.h"

LinearTransform::LinearTransform(float src_min, float src_max, float dst_min, float dst_max) {
  _src_min = src_min;
  _src_max = src_max;
  _dst_min = dst_min;
  _dst_max = dst_max;
  compute_src_to_dst_ratio();
}

float LinearTransform::get_src_min() {
  return _src_min;
}

void LinearTransform::set_src_min(float src_min) {
  _src_min = src_min;
}

float LinearTransform::get_src_max() {
  return _src_max;
}

void LinearTransform::set_src_max(float src_max) {
  _src_max = src_max;
}

float LinearTransform::get_dst_min() {
  return _dst_min;
}

void LinearTransform::set_dst_min(float dst_min) {
  _dst_min = dst_min;
}

float LinearTransform::get_dst_max() {
  return _dst_max;
}

void LinearTransform::set_dst_max(float dst_max) {
  _dst_max = dst_max;
}

void LinearTransform::src_to_dst(float src_value) {
  if (src_value <= _src_min) {
    return _dst_min;
  } else if (src_value >= _src_max) {
    return _dst_max;
  } else {
    return (src_value - _src_min) * _src_to_dst_ratio + _dst_min;
  }
}

void LinearTransform::dst_to_src(float dst_value) {
  if (dst_value <= _dst_min) {
    return _src_min;
  } else if (dst_value >= _dst_max) {
    return _src_max;
  } else {
    return (dst_value - _dst_min) / _src_to_dst_ratio + _src_min;
  }
}

void LinearTransform::compute_src_to_dst_ratio() {
  _src_to_dst_ratio = (_src_max - _src_min) / (_dst_max - _dst_min);
}
