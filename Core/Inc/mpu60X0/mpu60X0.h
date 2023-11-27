//
// Created by Lars Schwarz on 25.07.23.
//

#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include "types/parameter.h"
#include "types/state.h"
#include "types/sensor.h"

#define MPU_ADDRESS 0x68 << 1
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_X_REG 0x3B
#define TEMP_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_X_REG 0x43

#define MPU_COMMS_TIMEOUT 100

extern I2C_HandleTypeDef hi2c1;

class MPU60X0 {
public:
  MPU60X0(Parameter::Imu* parameter_, State::Imu* state_);

  uint8_t init(uint8_t trials);
  uint8_t SetGyroMaxDps();
  uint8_t SetGyroSamplerateDivisor();
  uint8_t SetAccelerometerMaxG();
  uint8_t GetValues(Sensor::Imu* mpu);
  void CalibrateGyro(Sensor::Imu* mpu);

private:
  Parameter::Imu* parameter;
  State::Imu* state;
  Sensor::Cartesian gyro_calibration_values {0};
};

#endif //MPU6050_H
