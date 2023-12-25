//
// Created by Lars Schwarz on 25.12.2023.
//

#include <iostream>
#include "SpeedController.h"

SpeedController::SpeedController(TIM_HandleTypeDef &htim, TB6612FNG &motordriver, Data::Data &data)
  : _htim(htim),
    _motordriver(motordriver),
    _data(data),
    _target_speed(0) {
}

void SpeedController::move(float speed) {
  _target_speed = speed;
}

void SpeedController::update() {
  if (_target_speed == 0) {
    _motordriver.ch_a.brake();
    _integral = 0;
    return;
  }
  // 100% -> 2m/s
  float error = _target_speed - (_data.velocity.x + _data.velocity.y);
  _integral += error*__HAL_TIM_GET_COUNTER(&_htim)/1000000;
  reset_timer();
  _proportional = error/std::abs(_target_speed);

  float command = (_proportional+2*_integral)*100/(std::abs(_target_speed)*3);
  if (command > 100) command = 100;
  if (command < -100) command = -100;

  _motordriver.ch_a.move(command);
}

void SpeedController::reset_timer() {
  // set the counter value a 1 to compensate for a zero division
  __HAL_TIM_SET_COUNTER(&_htim, 0);
}


