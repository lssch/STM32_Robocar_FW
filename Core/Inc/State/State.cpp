//
// Created by Lars Schwarz on 21.11.2023.
//

#include "State.h"

State::State(NeoPixel::Group *state_led_, GPIO esp32_ok_state_, GPIO esp32_reset_state_, GPIO esp32_reset_req_)
  : state_led(state_led_),
    esp32_ok_state(esp32_ok_state_),
    esp32_error_state(esp32_ok_state_),
    esp32_reset_req(esp32_reset_req_){
}

void State::set_state(FsmState state_) {
  state = state_;
}

void State::reset_esp32() {
  HAL_GPIO_WritePin(esp32_reset_req.port, esp32_reset_req.pin, GPIO_PIN_SET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(esp32_reset_req.port, esp32_reset_req.pin, GPIO_PIN_RESET);
}

void State::update() {
  if (state == FsmState::RUNNING &&
      !HAL_GPIO_ReadPin(esp32_error_state.port, esp32_error_state.pin) &&
      !HAL_GPIO_ReadPin(esp32_ok_state.port, esp32_ok_state.pin))
    state = FsmState::INFO_ACTIVE;

  if ((state == FsmState::RUNNING || state == FsmState::INFO_ACTIVE || state == FsmState::WARNING_ACTIVE) &&
      HAL_GPIO_ReadPin(esp32_error_state.port, esp32_error_state.pin) &&
      !HAL_GPIO_ReadPin(esp32_ok_state.port, esp32_ok_state.pin))
    state = FsmState::ERROR_ACTIVE;

  if ((state == FsmState::RUNNING || state == FsmState::INFO_ACTIVE || state != FsmState::ERROR_ACTIVE) &&
      !HAL_GPIO_ReadPin(esp32_error_state.port, esp32_error_state.pin) &&
      HAL_GPIO_ReadPin(esp32_ok_state.port, esp32_ok_state.pin))
    state = FsmState::WARNING_ACTIVE;

  switch (state) {
    case FsmState::UNDEFINED:
      state_led->set_color(255, 0, 0);
      break;
    case FsmState::INITIALISING:
      state_led->set_color(0, 0, 255);
      break;
    case FsmState::RUNNING:
      state_led->set_color(0, 255, 0);
      break;
    case FsmState::INFO_ACTIVE:
      state_led->set_color(255/2, 255/2, 0);
      break;
    case FsmState::WARNING_ACTIVE:
      state_led->set_color(255*3/4, 255/4, 0);
      break;
    case FsmState::ERROR_ACTIVE:
      state_led->set_color(255, 0, 0);
      break;
  }
}

