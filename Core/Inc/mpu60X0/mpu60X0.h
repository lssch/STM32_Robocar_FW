//
// Created by Lars Schwarz on 25.07.23.
//

#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include "types/parameter.h"
#include "types/state.h"
#include "types/sensor.h"
#include "types/data.h"

#define MPU_ADDRESS 0x68 << 1
#define MPU_PWR_MGMT_1_REG 0x6B
#define MPU_ACCEL_CONFIG_REG 0x1C
#define MPU_ACCEL_X_REG 0x3B
#define MPU_GYRO_CONFIG_REG 0x1B

#define MPU_COMMS_TIMEOUT 100

class MPU60X0 {
public:
  /// @brief MPU6050 and MPU 60000 object based on the I2C protocol.
  /// @param[in,out] hi2c I2C interface to communicate with the connected sensor.
  /// @param[in] parameter Parameters to configure the sensor.
  /// @param[out] state State to response the current state.
  /// @param[out] data Data to store the current calibration values to.
  /// @todo data can be changed to a reference.
  MPU60X0(I2C_HandleTypeDef &hi2c, const Parameter::Imu &parameter, State::Imu &state, Data::Imu *data);

  /// @brief Initializer to setup the imu.
  /// @param trials Number of tries to connect to the imu.
  /// @return SUCCESS if the imu is ready to use
  /// @todo The return value can be changed to bool.
  uint8_t init(const uint8_t trials);

  /// @brief Set the maximum rotation acceleration.
  /// @return SUCCESS if the change is valid.
  /// @todo The return value can be changed to bool.
  uint8_t SetGyroMaxDps();

  /// @brief 8-bit unsigned value. The Sample Rate is determined by dividing the gyroscope output rate by this value.
  /// @return SUCCESS if the change is valid.
  /// @todo The return value can be changed to bool.
  uint8_t SetGyroSamplerateDivisor();

  /// @brief Set the maximum translation acceleration.
  /// @return SUCCESS if the change is valid.
  /// @todo The return value can be changed to bool.
  uint8_t SetAccelerometerMaxG();

  /// @brief Get the current accelerometer, gyroscope and temperature values from the sensor.
  /// @param[in] imu Container to store the values to.
  /// @return SUCCESS if the reading is valid.
  /// @todo Proper name for this function.
  uint8_t GetValues(Sensor::Imu &imu);

  /// @brief Calibrate the gyro to compensate the constant offset \f$c_{bias}\f$.
  /// Formula for real the real output value: \f$y = m_{bias} \cdot x + q_{bias}\f$
  /// @attention The liner offset \f$m_{bias}\f$ isn't removed.
  void CalibrateGyro();

private:
  I2C_HandleTypeDef &_hi2c;
  const Parameter::Imu &_parameter;
  State::Imu &_state;
  Data::Imu *_data;
};

#endif //MPU6050_H
