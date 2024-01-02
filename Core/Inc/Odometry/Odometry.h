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
  /// @brief Odometry object for all kinematic calculations
  /// @param htim The timer to get the timer between the current an the last update.
  /// @param sensor The senor object to get the latest measurements.
  /// @param data The data object to store the calculated results.
  /// @param parameter The parameter to change the current settings.
  Odometry(TIM_HandleTypeDef &htim, const Sensor::Sensor &sensor, Data::Data &data, const Parameter::Parameter &parameter);

  /// @brief Calculates the shortest distance from the bumper of the car to the object.
  /// @return The shortest distance.
  int16_t distance_to_target();

  /// @brief Calculates the positioning error between the object and the car.
  void positioning_error();

  /// @brief Updates the calculations.
  void update();

  /// @brief Resets all calculations
  /// @attention All summed and integrated values can't be recreated.
  void reset();
private:

  /// @brief Resets the timer to measure the next cycle.
  /// @attention The timer is set to zero to catch a possible division by zero operation.
  void reset_timer();

  TIM_HandleTypeDef &_htim;
  const Sensor::Sensor &_sensor;
  Data::Data &_data;
  const Parameter::Parameter &_parameter;
  Cartesian2<float> _position_old;
};


#endif //STM32_ROBOCAR_FW_ODOMETRY_H
