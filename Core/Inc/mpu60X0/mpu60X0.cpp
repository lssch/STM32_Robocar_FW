//
// Created by Lars Schwarz on 25.07.23.
//

//#include <stdexcept>
#include "mpu60X0.h"

MPU60X0::MPU60X0(I2C_HandleTypeDef &hi2c, const Parameter::Imu &parameter, State::Imu &state, Data::Imu *data)
  :  _hi2c(hi2c),
    _parameter(parameter),
    _state(state),
    _data(data) {
  _state = State::Imu::DISCONNECTED;
}

uint8_t MPU60X0::init(const uint8_t trials) {
  uint8_t tx_data;

  // Check if MPU is available on I2C
  if (HAL_I2C_IsDeviceReady(&_hi2c, MPU_ADDRESS, trials, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  _state = State::Imu::UNCALIBRATED;

  // Set power management register to 0 to wake the sensor up
  tx_data = 0x01;
  if (HAL_I2C_Mem_Write(&_hi2c, MPU_ADDRESS, MPU_PWR_MGMT_1_REG, 1, &tx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  if (MPU60X0::SetGyroSamplerateDivisor() != SUCCESS) return ERROR;
  if (MPU60X0::SetGyroMaxDps() != SUCCESS) return ERROR;
  if (MPU60X0::SetAccelerometerMaxG() != SUCCESS) return ERROR;

  return SUCCESS;
}

uint8_t MPU60X0::SetGyroMaxDps() {
  uint8_t rxtx_data;

  if (HAL_I2C_Mem_Read(&_hi2c, MPU_ADDRESS, MPU_GYRO_CONFIG_REG, 1, &rxtx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  // Set all DPS bits to zero
  rxtx_data &= 0b11100000;

  switch (_parameter.gyro_max_dps) {
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

  if (HAL_I2C_Mem_Write(&_hi2c, MPU_ADDRESS, MPU_GYRO_CONFIG_REG, 1, &rxtx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  return SUCCESS;
}

uint8_t MPU60X0::SetGyroSamplerateDivisor() {
  uint8_t tx_data = _parameter.gyro_samplerate_divisor;

  if (HAL_I2C_Mem_Write(&_hi2c, MPU_ADDRESS, MPU_GYRO_CONFIG_REG, 1, &tx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  return SUCCESS;
}

uint8_t MPU60X0::SetAccelerometerMaxG() {
  uint8_t rxtx_data;

  if (HAL_I2C_Mem_Read(&_hi2c, MPU_ADDRESS, MPU_ACCEL_CONFIG_REG, 1, &rxtx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  // Set all DPS bits to zero
  rxtx_data &= 0b1110000;

  switch (_parameter.accel_max_g) {
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

  if (HAL_I2C_Mem_Write(&_hi2c, MPU_ADDRESS, MPU_ACCEL_CONFIG_REG, 1, &rxtx_data, 1, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  return SUCCESS;
}



uint8_t MPU60X0::GetValues(Sensor::Imu &imu) {
  std::array<uint8_t, 14> rx_data{};

  // Get the current values
  if (HAL_I2C_Mem_Read(&_hi2c, MPU_ADDRESS, MPU_ACCEL_X_REG, 1, rx_data.data(), 14, MPU_COMMS_TIMEOUT) != HAL_OK)
    return ERROR;

  Cartesian3<float> accelerometer{
    .x = static_cast<float>(static_cast<int16_t>(rx_data.at(0) << 8 | rx_data.at(1))),
    .y = static_cast<float>(static_cast<int16_t>(rx_data.at(2) << 8 | rx_data.at(3))),
    .z = static_cast<float>(static_cast<int16_t>(rx_data.at(4) << 8 | rx_data.at(5))),
  };
  float temperature = static_cast<float>(static_cast<int16_t>(rx_data.at(6) << 8 | rx_data.at(7)));
  Cartesian3<float> gyroscope{
    .x = static_cast<float>(static_cast<int16_t>(rx_data.at(8) << 8 | rx_data.at(9))),
    .y = static_cast<float>(static_cast<int16_t>(rx_data.at(10) << 8 | rx_data.at(11))),
    .z = static_cast<float>(static_cast<int16_t>(rx_data.at(12) << 8 | rx_data.at(13))),
  };

  imu.accelerometer.x = accelerometer.x * static_cast<float>(_parameter.accel_max_g) / 0x7FFF;
  imu.accelerometer.y = accelerometer.y * static_cast<float>(_parameter.accel_max_g) / 0x7FFF;
  imu.accelerometer.z = accelerometer.z * static_cast<float>(_parameter.accel_max_g) / 0x7FFF;

  imu.temperature = temperature / 340 + 36.53f;

  imu.gyroscope.x = gyroscope.x * (static_cast<float>(_parameter.gyro_max_dps) - _data->gyro_calibration_values.x) / 0x7FFF;
  imu.gyroscope.y = gyroscope.y * (static_cast<float>(_parameter.gyro_max_dps) - _data->gyro_calibration_values.y) / 0x7FFF;
  imu.gyroscope.z = gyroscope.z * (static_cast<float>(_parameter.gyro_max_dps) - _data->gyro_calibration_values.z) / 0x7FFF;

  return SUCCESS;
}


void MPU60X0::CalibrateGyro() {
  Sensor::Imu imu{};
  Cartesian3<float> sum{};

  _state = State::Imu::CALIBRATING;

  for (uint8_t i = 0; i < _parameter.gyro_calibration_samples; ++i) {
    MPU60X0::GetValues(imu);

    sum.x += imu.gyroscope.x;
    sum.y += imu.gyroscope.y;
    sum.z += imu.gyroscope.z;

    HAL_Delay(10);
  }

  _data->gyro_calibration_values.x = sum.x / (static_cast<float>(_parameter.gyro_calibration_samples) + _data->gyro_calibration_values.x);
  _data->gyro_calibration_values.y = sum.y / (static_cast<float>(_parameter.gyro_calibration_samples) + _data->gyro_calibration_values.y);
  _data->gyro_calibration_values.z = sum.z / (static_cast<float>(_parameter.gyro_calibration_samples) + _data->gyro_calibration_values.z);

  _state = State::Imu::CALIBRATED;
}
