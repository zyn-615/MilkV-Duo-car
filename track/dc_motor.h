#pragma once

#include <wiringx.h>
#include <string>
#include <chrono>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <memory>
#include "motor_types.h"
#include "i_motor.h"

namespace MotorControl {
  class DCMotor : public IMotor, protected MotorPins {
    private:
      std::string name_;

    public:
      DCMotor(PinId forward, PinId backward, PinId speed, const std::string name);
      DCMotor() {}
      ~DCMotor();
      void forward() override;
      void backward() override;
      void stop() override;
      void setSpeed(PWMValue duty) override;
      const std::string& getName() const override;
  };
}