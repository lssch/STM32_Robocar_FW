//
// Created by Lars Schwarz on 16.12.2023.
//

#include <iostream>
#include <cstring>
#include "A010.h"

A010::A010(UART_HandleTypeDef &huart)
  : _huart(huart) {
}

void A010::init() {}

void A010::get_frame(Sensor::TofCamera &sensor) {
  std::array<uint8_t , 645> rx_data{0};

  // Catch the beginning of the next package
  while (not(rx_data.at(0) == A010_PACKET_HEADER_1 and rx_data.at(1) == A010_PACKET_HEADER_2)) {
    rx_data.at(0) = rx_data.at(1);
    HAL_UART_Receive(&_huart, &rx_data.at(1), 1, A010_COMMS_TIMEOUT);
  }

  response.clear();

  HAL_UART_Receive(&_huart, &rx_data.at(2), 2, A010_COMMS_TIMEOUT);
  for (int i = 2; i < 4; ++i)
    response.push_back(rx_data.at(i));

  uint16_t rx_length = rx_data.at(2) << 8 | rx_data.at(3);

  HAL_UART_Receive(&_huart, &rx_data.at(4), rx_length, A010_COMMS_TIMEOUT);
  for (int i = 0; i < rx_length; ++i)
    response.push_back(rx_data.at(i + 4));

  memcpy(sensor.pixels.data(), &response.at(20), 25*25);

  std::cout << "rx_data = ";
  for (auto byte: response) {
    std::cout << std::hex << +byte << " ";
  }
  std::cout << std::endl;
}

void inline A010::transmit(const std::string& command) {
  std::vector<uint8_t> tx_data{A010_PACKET_HEADER_1, A010_PACKET_HEADER_2};
  for (const auto &item: command)
    tx_data.push_back(item);
  tx_data.push_back(A010_PACKET_END);

  std::cout << "tx_data = ";
  for (auto byte: tx_data) {
    std::cout << std::hex << +byte << " ";
  }
  std::cout << std::endl;

  HAL_UART_Transmit(&_huart, tx_data.data(), tx_data.size(), A010_COMMS_TIMEOUT);
}

void inline A010::receive() {
  std::array<uint8_t , 645> rx_data{0};
  uint8_t rx_length = 4;

  response.clear();

  HAL_UART_Receive(&_huart, rx_data.data(), rx_length, A010_COMMS_TIMEOUT);
  for (int i = 0; i < rx_length; ++i)
    response.push_back(rx_data.at(i));

  rx_length = rx_data.at(3) + 1;

  HAL_UART_Receive(&_huart, &rx_data.at(4), rx_length, A010_COMMS_TIMEOUT);
  for (int i = 0; i < rx_length; ++i)
    response.push_back(rx_data.at(i + 4));

  std::cout << "rx_data = ";
  for (auto byte: response) {
    std::cout << std::hex << +byte << " ";
  }
  std::cout << std::endl;

}
