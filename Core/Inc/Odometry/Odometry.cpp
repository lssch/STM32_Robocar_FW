//
// Created by Lars Schwarz on 15.12.2023.
//

#include "Odometry.h"

Odometry::Odometry(TIM_HandleTypeDef &htim, Sensor::Sensor &sensor, Data::Data &data, Parameter::Parameter &parameter)
  : _htim(htim),
    _sensor(sensor),
    _data(data),
    _parameter(parameter),
    _position_old({0,0}) {
}

void Odometry::update() {
  positioning_error();
  // Summing of the individual displacements
  _data.position.x += _sensor.vfs.motion.x;
  _data.position.y += _sensor.vfs.motion.y;

  // Calculate the current speed an update the shadow register
  _data.velocity = {(_data.position.x - _position_old.x)*1000000 / __HAL_TIM_GET_COUNTER(&_htim),
                    (_data.position.y - _position_old.y)*1000000 / __HAL_TIM_GET_COUNTER(&_htim)};
  reset_timer();
  _position_old = _data.position;
}

int16_t Odometry::distance_to_target() {
  return static_cast<int16_t>(_sensor.tof_spot.distance) - _parameter.odometry.origin_to_front + _parameter.odometry.tof_spot_link.x;
}

void Odometry::positioning_error() {
  _data.distance_positioning_error = distance_to_target() - _parameter.operating_modes.distance.setpoint_distance_to_target;
}

void Odometry::reset() {
  _data.position = {0,0};
  reset_timer();
}

void inline Odometry::reset_timer() {
  // set the counter value a 1 to compensate for a zero division
  __HAL_TIM_SET_COUNTER(&_htim, 1);
}
