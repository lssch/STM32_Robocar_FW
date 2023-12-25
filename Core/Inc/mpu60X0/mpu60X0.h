//
// Created by Lars Schwarz on 25.07.23.
//

#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include "types/parameter/parameter.h"
#include "types/state/state.h"
#include "types/sensor/sensor.h"
#include "types/data/data.h"

#define MPU_ADDRESS 0x68 << 1
#define MPU_PWR_MGMT_1_REG 0x6B
#define MPU_ACCEL_CONFIG_REG 0x1C
#define MPU_ACCEL_X_REG 0x3B
#define MPU_GYRO_CONFIG_REG 0x1B

#define MPU_COMMS_TIMEOUT 100

class MPU60X0 {
public:
  /// @brief MPU6050 and MPU 60000 object based on the I2C protocol.
  /// @param hi2c I2C connection.
  /// @param parameter IMU parameters to configure the sensor.
  /// @param state IMU state to response the current state.
  /// @param data Data to store the current calibration values to
  MPU60X0(I2C_HandleTypeDef &hi2c, const Parameter::Imu &parameter, State::Imu &state, Data::Imu *data);

  uint8_t init(uint8_t trials);
  uint8_t SetGyroMaxDps();
  uint8_t SetGyroSamplerateDivisor();
  uint8_t SetAccelerometerMaxG();
  uint8_t GetValues(Sensor::Imu &imu);
  void CalibrateGyro();

private:
  I2C_HandleTypeDef &_hi2c;
  const Parameter::Imu &_parameter;
  State::Imu &_state;
  Data::Imu *_data;
};

#endif //MPU6050_H
