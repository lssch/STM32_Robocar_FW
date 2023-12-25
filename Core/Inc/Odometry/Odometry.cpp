//
// Created by Lars Schwarz on 15.12.2023.
//

#include "Odometry.h"

Odometry::Odometry(TIM_HandleTypeDef &htim, Sensor::Sensor &sensor, Data::Data &data, Parameter::Parameter &parameter)
  : _htim(htim),
    _sensor(sensor),
    _data(data),
    _parameter(parameter),
    _vfs_motion_old({0,0}) {
}

void Odometry::update() {
  // Calculate the real distance from the origin to the target object
  _data.distance_to_target = _sensor.tof_spot.distance - (_parameter.odometry.origin_to_front + _parameter.odometry.tof_spot_link.x);

  // Summing of the individual displacements
  _data.position.x += _sensor.vfs.motion.x;
  _data.position.y += _sensor.vfs.motion.y;

  // Calculate the current speed an update the shadow register
  _data.velocity = {(_sensor.vfs.motion.x - _vfs_motion_old.x)*10000000 / __HAL_TIM_GET_COUNTER(&_htim),
                    (_sensor.vfs.motion.y - _vfs_motion_old.y)*10000000 / __HAL_TIM_GET_COUNTER(&_htim)};
  _vfs_motion_old = _sensor.vfs.motion;
  reset_timer();
}

void Odometry::reset() {
  _data.position = {0,0};
  reset_timer();
}

void inline Odometry::reset_timer() {
  // set the counter value a 1 to compensate for a zero division
  __HAL_TIM_SET_COUNTER(&_htim, 1);
}
