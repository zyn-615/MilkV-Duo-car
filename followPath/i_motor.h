#pragma once

#include <string>
#include "motor_types.h"

namespace MotorControl {
  class IMotor {
    public:
      virtual ~IMotor() = default;
      virtual void forward() = 0;
      virtual void backward() = 0;
      virtual void stop() = 0;
      virtual void setSpeed(PWMValue duty) = 0;
      virtual const std::string& getName() const = 0;
  };
}