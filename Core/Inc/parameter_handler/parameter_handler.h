//
// Created by Lars Schwarz on 02.08.23.
//

#ifndef PARAMETER_H
#define PARAMETER_H

#include "types/parameter.h"

class ParameterHandler {
public:
  /// @brief Parameter handler object to safe an restore values from the flash.
  /// @param parameter Container which contains all variables to safe.
  explicit ParameterHandler(Parameter::Parameter &parameter);

  /// @brief Initialises the parameter fields in the flash memory.
  /// @attention This function will override all values in the flash.
  void InitParameter();

  /// @brief Get the current parameters from the flash memory.
  /// @return SUCCESS if the operation was valid.
  uint8_t GetParameter();

  /// @brief Write the current parameters to the flash memory.
  /// @return SUCCESS if the operation was valid.
  uint8_t SetParameter();
private:
  Parameter::Parameter &_parameter;
  Parameter::Parameter _new_parameter;
  const uint8_t check_value = 123;
};


#endif //PARAMETER_H
