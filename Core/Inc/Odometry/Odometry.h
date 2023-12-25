//
// Created by Lars Schwarz on 15.12.2023.
//

#ifndef STM32_ROBOCAR_FW_ODOMETRY_H
#define STM32_ROBOCAR_FW_ODOMETRY_H

#include "stm32f4xx_hal.h"
#include "types/sensor.h"
#include "types/data.h"
#include "types/parameter.h"

class Odometry {
public:
  Odometry(TIM_HandleTypeDef &htim, Sensor::Sensor &sensor, Data::Data &data, Parameter::Parameter &parameter);
  int16_t distance_to_target();
  void positioning_error();
  void update();
  void reset();
private:
  void reset_timer();

  TIM_HandleTypeDef &_htim;
  Sensor::Sensor &_sensor;
  Data::Data &_data;
  Parameter::Parameter &_parameter;
  Cartesian2<float> _position_old;
};


#endif //STM32_ROBOCAR_FW_ODOMETRY_H
