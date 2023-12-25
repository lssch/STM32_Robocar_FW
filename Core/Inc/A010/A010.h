//
// Created by Lars Schwarz on 16.12.2023.
//

#ifndef STM32_ROBOCAR_FW_A010_H
#define STM32_ROBOCAR_FW_A010_H

#include <vector>
#include "stm32f4xx_hal.h"
#include "types/sensor.h"
#include "types/parameter.h"

#define A010_PACKET_HEADER_1 0x00
#define A010_PACKET_HEADER_2 0xFF
#define A010_PACKET_END 0xDD
#define A010_COMMS_TIMEOUT 100

class A010 {
public:
  A010(UART_HandleTypeDef &huart);

  void init();
  void get_frame(Sensor::TofCamera &sensor);
private:
  UART_HandleTypeDef &_huart;
  std::vector<uint8_t> response;

  void transmit(const std::string& command);
  void receive();
};


#endif //STM32_ROBOCAR_FW_A010_H
