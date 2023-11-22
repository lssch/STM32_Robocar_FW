//
// Created by Lars Schwarz on 21.11.2023.
//

#ifndef STM32_ROBOCAR_FW_STATE_H
#define STM32_ROBOCAR_FW_STATE_H


#include "NeoPixel/NeoPixel.h"
#include "stm32f4xx_hal.h"

class __attribute__((__packed__)) GPIO {
public:
  GPIO_TypeDef* port;
  uint16_t pin;
};


enum class FsmState : uint8_t {
  UNDEFINED = 0,
  INITIALISING,
  RUNNING,
  INFO_ACTIVE,
  WARNING_ACTIVE,
  ERROR_ACTIVE
};

class State {
public:
  State(NeoPixel::Group* state_led_, GPIO esp32_ok_state_, GPIO esp32_reset_state_, GPIO esp32_reset_req_);
  void update();
  void set_state(FsmState state_);

private:
  FsmState state;
  NeoPixel::Group* state_led;
  GPIO esp32_ok_state;
  GPIO esp32_error_state;
  GPIO esp32_reset_req;

  void reset_esp32();
};


#endif //STM32_ROBOCAR_FW_STATE_H
