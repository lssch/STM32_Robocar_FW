//
// Created by Lars Schwarz on 15.12.2023.
//

#ifndef STM32_ROBOCAR_FW_ODOMETRY_H
#define STM32_ROBOCAR_FW_ODOMETRY_H

#include "stm32f4xx_hal.h"
#include "types/sensor/sensor.h"
#include "types/data/data.h"
#include "types/parameter/parameter.h"

class Odometry {
public:
  Odometry(TIM_HandleTypeDef &htim, Sensor::Sensor &sensor, Data::Data &data, Parameter::Parameter &parameter);
  void update();
  void reset();
private:
  void reset_timer();

  TIM_HandleTypeDef &_htim;
  Sensor::Sensor &_sensor;
  Data::Data &_data;
  Parameter::Parameter &_parameter;
  Cartesian2<float> _vfs_motion_old;
};


#endif //STM32_ROBOCAR_FW_ODOMETRY_H
