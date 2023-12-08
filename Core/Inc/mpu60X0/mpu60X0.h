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
  MPU60X0(Parameter::Imu* parameter_, State::Imu* state_, Data::Imu* data_);

  uint8_t init(uint8_t trials);
  uint8_t SetGyroMaxDps();
  uint8_t SetGyroSamplerateDivisor();
  uint8_t SetAccelerometerMaxG();
  uint8_t GetValues(Sensor::Imu* mpu);
  void CalibrateGyro();

private:
  Parameter::Imu* parameter;
  State::Imu* state;
  Data::Imu* data;
};

#endif //MPU6050_H
