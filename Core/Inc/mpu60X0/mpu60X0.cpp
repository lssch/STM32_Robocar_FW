//
// Created by Lars Schwarz on 25.07.23.
//

#include <stdexcept>
#include "mpu60X0.h"

MPU60X0::MPU60X0(Parameter::Imu* parameter_, State::Imu* state_)
  : parameter(parameter_),
    state(state_) {
  *state = State::Imu::DISCONNECTED;
}

uint8_t MPU60X0::init(uint8_t trials) {
  uint8_t tx_data;

  // Check if MPU is available on I2C
  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_ADDRESS, trials, MPU_COMMS_TIMEOUT) != HAL_OK)
    throw std::runtime_error(std::string("No MPU-device available at address: 0x") + std::to_string(MPU_ADDRESS));

  *state = State::Imu::UNCALIBRATED;

  // Set power management register to 0 to wake the sensor up
  tx_data = 0x01;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, PWR_MGMT_1_REG, 1, &tx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK) {
    throw std::runtime_error(std::string("Device does not response"));
  }

  if (MPU60X0::SetGyroSamplerateDivisor() != SUCCESS) return ERROR;
  if (MPU60X0::SetGyroMaxDps() != SUCCESS) return ERROR;
  if (MPU60X0::SetAccelerometerMaxG() != SUCCESS) return ERROR;

  return SUCCESS;
}

uint8_t MPU60X0::SetGyroMaxDps() {
  uint8_t rxtx_data;

  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, GYRO_CONFIG_REG, 1, &rxtx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  // Set all DPS bits to zero
  rxtx_data &= 0b11100000;

  switch (parameter->gyro_max_dps) {
    case Parameter::ImuGyproMaxDps::PM_250_DPS:
      rxtx_data |= 0b00000000;
      break;
    case Parameter::ImuGyproMaxDps::PM_500_DPS:
      rxtx_data |= 0b00001000;
      break;
    case Parameter::ImuGyproMaxDps::PM_1000_DPS:
      rxtx_data |= 0b00010000;
      break;
    case Parameter::ImuGyproMaxDps::PM_2000_DPS:
      rxtx_data |= 0b00011000;
      break;
  }

  if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, GYRO_CONFIG_REG, 1, &rxtx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  return SUCCESS;
}

uint8_t MPU60X0::SetGyroSamplerateDivisor() {
  uint8_t tx_data = parameter->gyro_samplerate_divisor;

  if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, GYRO_CONFIG_REG, 1, &tx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  return SUCCESS;
}

uint8_t MPU60X0::SetAccelerometerMaxG() {
  uint8_t rxtx_data;

  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, ACCEL_CONFIG_REG, 1, &rxtx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  // Set all DPS bits to zero
  rxtx_data &= 0b1110000;

  switch (parameter->accel_max_g) {
    case Parameter::ImuAccelMaxG::PM_2G:
      rxtx_data |= 0b00000000;
      break;
    case Parameter::ImuAccelMaxG::PM_4G:
      rxtx_data |= 0b00001000;
      break;
    case Parameter::ImuAccelMaxG::PM_8G:
      rxtx_data |= 0b00010000;
      break;
    case Parameter::ImuAccelMaxG::PM_16G:
      rxtx_data |= 0b00011000;
      break;
  }

  if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, ACCEL_CONFIG_REG, 1, &rxtx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  return SUCCESS;
}



uint8_t MPU60X0::GetValues(Sensor::Imu* mpu) {
  uint8_t rx_data[14];

  // Get the current values
  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, ACCEL_X_REG, 1, rx_data, 14, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  int16_t accel[3] = {
          (int16_t)(rx_data[0] << 8 | rx_data[1]),
          (int16_t)(rx_data[2] << 8 | rx_data[3]),
          (int16_t)(rx_data[4] << 8 | rx_data[5])};
  int16_t temp = (int16_t)(rx_data[6] << 8 | rx_data[7]);
  int16_t gyro[3] = {
          (int16_t)(rx_data[8] << 8 | rx_data[9]),
          (int16_t)(rx_data[10] << 8 | rx_data[11]),
          (int16_t)(rx_data[12] << 8 | rx_data[13]),
  };

  mpu->accelerometer.x = (float) accel[0] / 0x7FFF * (float) parameter->accel_max_g;
  mpu->accelerometer.y = (float) accel[1] / 0x7FFF * (float) parameter->accel_max_g;
  mpu->accelerometer.z = (float) accel[2] / 0x7FFF * (float) parameter->accel_max_g;

  mpu->temperature = (float) temp / 340 + 36.53f;

  mpu->gyroscope.x = (float) gyro[0] / 0x7FFF * (float) parameter->gyro_max_dps - gyro_calibration_values.x;
  mpu->gyroscope.y = (float) gyro[1] / 0x7FFF * (float) parameter->gyro_max_dps - gyro_calibration_values.y;
  mpu->gyroscope.z = (float) gyro[2] / 0x7FFF * (float) parameter->gyro_max_dps - gyro_calibration_values.z;

  return SUCCESS;
}


void MPU60X0::CalibrateGyro(Sensor::Imu* mpu) {
  Sensor::Cartesian sum{};

  *state = State::Imu::CALIBRATING;

  for (uint8_t i = 0; i < parameter->gyro_calibration_samples; ++i) {
    MPU60X0::GetValues(mpu);

    sum.x += mpu->gyroscope.x;
    sum.y += mpu->gyroscope.y;
    sum.z += mpu->gyroscope.z;

    HAL_Delay(20);
  }

  gyro_calibration_values.x = sum.x / (float) parameter->gyro_calibration_samples + gyro_calibration_values.x;
  gyro_calibration_values.y = sum.y / (float) parameter->gyro_calibration_samples + gyro_calibration_values.y;
  gyro_calibration_values.z = sum.z / (float) parameter->gyro_calibration_samples + gyro_calibration_values.z;

  *state = State::Imu::CALIBRATED;
}
