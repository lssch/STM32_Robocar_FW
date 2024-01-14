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
  /// @brief A010 3D-Depth cam object
  /// @param[in,out] huart Uart interface to communicate with the connected camera.
  A010(UART_HandleTypeDef &huart);

  /// @brief Initializer to setup the camera
  /// @todo This function can be eliminated if the Constructor is called inside the main function and after the initialisation of the huart
  void init();

  /// @brief Get the next full frame from the buffer.
  /// @param[in] sensor Frame to store the current image.
  void get_frame(Sensor::TofCamera &sensor);
private:
  UART_HandleTypeDef &_huart;
  std::vector<uint8_t> response;

  /// @brief Wrapper for the built in HAL_UART_Transmit function for better ease of use.
  /// @param[in] command Request which needs to be sent to the camera.
  void transmit(const std::string& command);

  /// @brief Wrapper for the built in HAL_UART_Receive function for better ease of use.
  void receive();
};


#endif //STM32_ROBOCAR_FW_A010_H
