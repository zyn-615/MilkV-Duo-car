#pragma once

#include <wiringx.h>
#include <string>
#include <chrono>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <memory>
#include "i_motor.h"
#include "motor_types.h"
#include <concepts>
#include <span>

namespace MotorControl {
  template <typename T>
  concept MotorType = std::derived_from <T, IMotor>;

  template <MotorType T>
  class CarController {
    private:
      std::vector <std::shared_ptr <T>> motors_;
    
    public:
      template <typename... Motor>
      CarController(Motor... motor);
      CarController(std::span <T> motors);
      CarController(std::span <std::shared_ptr <T>> motors);
      void checkValid() {}
      void move(Direction dir, Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75);
      void forward(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75);
      void backward(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75);
      void turnLeft(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75);
      void turnRight(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75);
      void setMotorSpeeds(PWMValue leftSpeed, PWMValue rightSpeed);
      void stop();
  };
}
