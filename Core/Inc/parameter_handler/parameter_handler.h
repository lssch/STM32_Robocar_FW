//
// Created by Lars Schwarz on 02.08.23.
//

#ifndef PARAMETER_H
#define PARAMETER_H

#include "types/parameter.h"

class ParameterHandler {
public:
  ParameterHandler(Parameter::Parameter* parameter_);

  void InitParameter();
  uint8_t GetParameter();
  uint8_t SetParameter();
private:
  Parameter::Parameter* parameter;
  Parameter::Parameter new_parameter;
  uint8_t check_value = 123;
};


#endif //PARAMETER_H
