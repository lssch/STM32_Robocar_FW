//
// Created by Lars Schwarz on 25.12.2023.
//

#ifndef STM32_ROBOCAR_FW_SPEEDCONTROLLER_H
#define STM32_ROBOCAR_FW_SPEEDCONTROLLER_H

#include "TB6612FNG/TB6612FNG.h"
#include "types/data.h"

class SpeedController {
public:
  explicit SpeedController(TIM_HandleTypeDef &htim, TB6612FNG &motordriver, Data::Data &data);
  void move(float speed);
  void update();

private:
  void reset_timer();

  TIM_HandleTypeDef &_htim;
  TB6612FNG &_motordriver;
  Data::Data &_data;
  float _target_speed;
  float _integral;
  float _proportional;
};

#endif //STM32_ROBOCAR_FW_SPEEDCONTROLLER_H
