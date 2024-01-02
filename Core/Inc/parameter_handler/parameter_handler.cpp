//
// Created by Lars Schwarz on 02.08.23.
//

#include <iostream>
#include "parameter_handler.h"
#include "stm32f4xx_hal.h"

ParameterHandler::ParameterHandler(Parameter::Parameter& parameter)
  : _parameter(parameter) {
}

void ParameterHandler::InitParameter() {
  _parameter.car.chassis_length = 224;
  _parameter.car.chassis_width = 146;
  _parameter.car.wheel_diameter = 61;

  _parameter.imu.gyro_max_dps = Parameter::ImuGyproMaxDps::PM_500_DPS;
  _parameter.imu.gyro_samplerate_divisor = 0;
  _parameter.imu.accel_max_g = Parameter::ImuAccelMaxG::PM_4G;
  _parameter.imu.gyro_calibration_samples = 100;

  _parameter.vfs.height = 18;
  _parameter.vfs.measured_target_length = 4;
  _parameter.vfs.led_shutter = 1;
  _parameter.vfs.high_resolution = 1;
  _parameter.vfs.light_color = {255, 0, 0, 20};

  _parameter.servo.zero_position = 0;
  _parameter.servo.max_steering_angle = 6000;
  _parameter.servo.steering_limits = 2500;

  _parameter.navlight.color_front = {255, 255, 255, 100};
  _parameter.navlight.color_back = {255, 0, 0, 100};
  _parameter.navlight.color_back = {3*255/4, 255/4, 0, 100};

  _parameter.odometry.origin_to_front = 135;
  _parameter.odometry.origin_to_back = 50;
  _parameter.odometry.imu_link = {-50,0,0};
  _parameter.odometry.tof_spot_link = {100,0,0};
  _parameter.odometry.tof_cam_link = {50,0,50};
  _parameter.odometry.vfs_link = {-5,0,0};

  _parameter.operating_modes.distance.setpoint_distance_to_target = 200;
  _parameter.operating_modes.distance.positioning_error_boundaries = 1;
  _parameter.operating_modes.distance.threshold_fine_positioning = 300;
}

uint8_t ParameterHandler::GetParameter() {
  uint8_t* parameter_uint8_t = (uint8_t *) 0x08000000;
  uint16_t* parameter_uint16_t = (uint16_t *) 0x08000000;
  uint32_t* parameter_uint32_t = (uint32_t *) 0x08000000;

  for (int i = 0; i < 20; ++i) {
    std::cout << "Parameter " << i << ": "<< +*parameter_uint8_t++ << std::endl;
  }

  parameter_uint8_t = (uint8_t *) 0x08060000;

  // read bytes
  // Compare the first value to a fix value to check if the read access is successful
  if (*parameter_uint8_t++ != check_value) return ERROR;


  _parameter.car.chassis_width = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.car.wheel_diameter = *(parameter_uint8_t);
  parameter_uint8_t += 1;

  _parameter.imu.gyro_samplerate_divisor = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.imu.accel_max_g = static_cast<Parameter::ImuAccelMaxG>(*(parameter_uint8_t));
  parameter_uint8_t += 1;
  _parameter.imu.gyro_calibration_samples = *(parameter_uint8_t);
  parameter_uint8_t += 1;

  _parameter.vfs.light_color.red = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.vfs.light_color.green = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.vfs.light_color.blue = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.vfs.light_color.alpha = *(parameter_uint8_t);
  parameter_uint8_t += 1;

  _parameter.navlight.color_front.red = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.navlight.color_front.green = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.navlight.color_front.blue = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.navlight.color_back.red = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.navlight.color_back.green = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.navlight.color_back.blue = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.navlight.color_blinker.red = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.navlight.color_blinker.green = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  _parameter.navlight.color_blinker.blue = *(parameter_uint8_t);
  parameter_uint8_t += 1;

  // read words
  parameter_uint16_t = (uint16_t*) parameter_uint8_t;

  _parameter.car.chassis_length = *(parameter_uint16_t);
  parameter_uint16_t += 2;

  _parameter.imu.gyro_max_dps = static_cast<Parameter::ImuGyproMaxDps>(*parameter_uint16_t);
  parameter_uint16_t += 2;

  _parameter.servo.zero_position = static_cast<int16_t>(*parameter_uint16_t);
  parameter_uint16_t += 2;
  _parameter.servo.max_steering_angle = static_cast<int16_t>(*parameter_uint16_t);
  parameter_uint16_t += 2;
  _parameter.servo.steering_limits = static_cast<int16_t>(*parameter_uint16_t);
  parameter_uint16_t += 2;

  // read dwords
  parameter_uint32_t = (uint32_t*) parameter_uint16_t;
  //parameter_uint32_t += 4;

  return SUCCESS;
}

uint8_t ParameterHandler::SetParameter() {
  uint8_t access_error = 0;
  uint32_t memory_address = 0x08060000;
  uint32_t sector_error;
  FLASH_EraseInitTypeDef flashErase;
  flashErase.TypeErase = TYPEERASE_SECTORS;
  flashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  flashErase.Sector = FLASH_SECTOR_7;
  flashErase.NbSectors = 1;

  // Data to write to flash
  uint8_t parameter_uint8_t[] = {
          check_value,

          _parameter.car.chassis_width,
          _parameter.car.wheel_diameter,

          _parameter.imu.gyro_samplerate_divisor,
          static_cast<uint8_t>(_parameter.imu.accel_max_g),
          _parameter.imu.gyro_calibration_samples,

          _parameter.vfs.light_color.red,
          _parameter.vfs.light_color.green,
          _parameter.vfs.light_color.blue,
          _parameter.vfs.light_color.alpha,

          _parameter.navlight.color_front.red,
          _parameter.navlight.color_front.green,
          _parameter.navlight.color_front.blue,
          _parameter.navlight.color_back.red,
          _parameter.navlight.color_back.green,
          _parameter.navlight.color_back.blue,
          _parameter.navlight.color_blinker.red,
          _parameter.navlight.color_blinker.green,
          _parameter.navlight.color_blinker.blue,
  };

  uint16_t parameter_uint16_t[] = {
          _parameter.car.chassis_length,

          static_cast<uint16_t>(_parameter.imu.gyro_max_dps),

          static_cast<uint16_t>(_parameter.servo.zero_position),
          static_cast<uint16_t>(_parameter.servo.max_steering_angle),
          static_cast<uint16_t>(_parameter.servo.steering_limits),
  };

  uint32_t parameter_uint32_t[] = {
  };


  // Unlock flash to enable write mode
  if (HAL_FLASH_Unlock() != HAL_OK) return ERROR;

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

  // Erase flash memory to rewrite
  if (HAL_FLASHEx_Erase(&flashErase, &sector_error) != HAL_OK) {
    // Lock flash to protect against unwanted writes
    HAL_FLASH_Lock();
    return ERROR;
  }

  // Write parameters to flash
  for (unsigned char i : parameter_uint8_t) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                          memory_address,
                          i) != HAL_OK)
      access_error = ERROR;
    memory_address +=1;
    // Protect against overflow
    if (memory_address > 0x0807FFFF) {
      // Lock flash to protect against unwanted writes
      HAL_FLASH_Lock();
      return ERROR;
    }
  }

  for (unsigned short i : parameter_uint16_t) {
    // Program value using two bytes
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          memory_address,
                          i) != HAL_OK)
      access_error = ERROR;
    memory_address +=2;
    // Protect against overflow
    if (memory_address > 0x0807FFFF) {
      // Lock flash to protect against unwanted writes
      HAL_FLASH_Lock();
      return ERROR;
    }
  }

  for (unsigned long i : parameter_uint32_t) {
    // Program value using four bytes
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          memory_address,
                          i) != HAL_OK)
      access_error = ERROR;
    memory_address += 4;
    // Protect against overflow
    if (memory_address > 0x0807FFFF) {
      // Lock flash to protect against unwanted writes
      HAL_FLASH_Lock();
      return ERROR;
    }
  }

  // Lock flash to protect against unwanted writes
  if (HAL_FLASH_Lock() != HAL_OK) return ERROR;


  uint8_t* parameter_uint8_t_in_mem = (uint8_t *) 0x08060000;

  for (int i = 0; i < 20; ++i) {
    std::cout << "Parameter " << i << ": "<< +*(parameter_uint8_t_in_mem++) << std::endl;
  }

  return access_error;
}