//
// Created by Lars Schwarz on 02.08.23.
//

#include "parameter_handler.h"
#include "stm32f4xx_hal.h"

ParameterHandler::ParameterHandler(Parameter::Parameter* parameter_)
  : parameter(parameter_) {
}

void ParameterHandler::InitParameter() {
  parameter->car.chassis_length = 224;
  parameter->car.chassis_width = 146;
  parameter->car.wheel_diameter = 61;

  parameter->imu.gyro_max_dps = Parameter::ImuGyproMaxDps::PM_500_DPS;
  parameter->imu.gyro_samplerate_divisor = 0;
  parameter->imu.accel_max_g = Parameter::ImuAccelMaxG::PM_4G;
  parameter->imu.gyro_calibration_samples = 100;

  parameter->vfs.neopixel.enable = true;
  parameter->vfs.neopixel.color.red = 255;
  parameter->vfs.neopixel.color.green = 0;
  parameter->vfs.neopixel.color.blue = 0;
  parameter->vfs.neopixel.brightness = 5;

  parameter->servo.zero_position = 0;
  parameter->servo.max_steering_angle = 6000;
  parameter->servo.steering_limits = 3000;

  parameter->navlight.color_front.red = 255;
  parameter->navlight.color_front.green = 255;
  parameter->navlight.color_front.blue= 255;
  parameter->navlight.color_back.red = 255;
  parameter->navlight.color_back.green = 0;
  parameter->navlight.color_back.blue= 0;
  parameter->navlight.color_blinker.red = 3*255/4;
  parameter->navlight.color_blinker.green = 255/4;
  parameter->navlight.color_blinker.blue= 0;
}

uint8_t ParameterHandler::GetParameter() {
  uint8_t* parameter_uint8_t = (uint8_t *) 0x08060000;
  uint16_t* parameter_uint16_t = (uint16_t *) 0x08060000;
  uint32_t* parameter_uint32_t = (uint32_t *) 0x08060000;

  // read bytes
  // Compare the first value to a fix value to check if the read access is successful
  if (*parameter_uint8_t++ != check_value) return ERROR;

  parameter->imu.accel_max_g = static_cast<Parameter::ImuAccelMaxG>(*(parameter_uint8_t));
  parameter_uint8_t += 1;
  parameter->imu.gyro_samplerate_divisor = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  parameter->imu.gyro_calibration_samples = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  parameter->car.chassis_width = *(parameter_uint8_t);
  parameter_uint8_t += 1;
  parameter->car.wheel_diameter = *(parameter_uint8_t);
  parameter_uint8_t += 1;

  // read words
  parameter_uint16_t = (uint16_t*) parameter_uint8_t;
  parameter->car.chassis_length = *(parameter_uint16_t);
  parameter_uint16_t += 2;
  parameter->imu.gyro_max_dps = static_cast<Parameter::ImuGyproMaxDps>(*parameter_uint16_t);
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
          static_cast<uint8_t>(parameter->imu.accel_max_g),
          parameter->imu.gyro_samplerate_divisor,
          parameter->imu.gyro_calibration_samples,
          parameter->car.chassis_width,
          parameter->car.wheel_diameter,
  };

  uint16_t parameter_uint16_t[] = {
          parameter->car.chassis_length,
          static_cast<uint16_t>(parameter->imu.gyro_max_dps),
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
                          memory_address++,
                          i) != HAL_OK)
      access_error = 1;

    // Protect against overflow
    if (memory_address > 0x0807FFFF) {
      // Lock flash to protect against unwanted writes
      HAL_FLASH_Lock();
      return ERROR;
    }
  }

  for (unsigned short i : parameter_uint16_t) {
    // Program value using two bytes
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                          memory_address++,
                          i) != HAL_OK)
      access_error = 1;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                          memory_address++,
                          i >> 8) != HAL_OK)
      access_error = 1;

    // Protect against overflow
    if (memory_address > 0x0807FFFF) {
      // Lock flash to protect against unwanted writes
      HAL_FLASH_Lock();
      return ERROR;
    }
  }

  for (unsigned long i : parameter_uint32_t) {
    // Program value using four bytes
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                          memory_address++,
                          i) != HAL_OK)
      access_error = 1;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                          memory_address++,
                          i >> 8) != HAL_OK)
      access_error = 1;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                          memory_address++,
                          i >> 16) != HAL_OK)
      access_error = 1;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                          memory_address++,
                          i >> 24) != HAL_OK)
      access_error = 1;

    // Protect against overflow
    if (memory_address > 0x0807FFFF) {
      // Lock flash to protect against unwanted writes
      HAL_FLASH_Lock();
      return ERROR;
    }
  }

  // Lock flash to protect against unwanted writes
  if (HAL_FLASH_Lock() != HAL_OK) return ERROR;

  return access_error;
}