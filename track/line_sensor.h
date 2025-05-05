#pragma once

#include <wiringx.h>
#include <string>
#include <chrono>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <memory>
#include "motor_types.h"
#include <concepts>
#include <span>

namespace Tracking {
  class ISensorState {
    public:
      virtual ~ISensorState() = default;
      virtual bool isOnLine() const = 0;
  };

  class DigitalSensorState : public ISensorState {
    private:
      bool onLine_;
    
    public:
      explicit DigitalSensorState(bool onLine) : onLine_(onLine) {}
      bool isOnLine() const override {
        return onLine_;
      }
  };

  class ISensor {
    public:
      virtual ~ISensor() = default;
      virtual std::unique_ptr <ISensorState> getNowState() const = 0;
      
  };
}