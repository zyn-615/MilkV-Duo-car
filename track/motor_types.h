#pragma once

#include <string>
#include <chrono>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <memory>
#include <wiringx.h>

namespace MotorControl {
  using PinId = int;
  using Duration = std::chrono::seconds;
  using PWMValue = long;

  constexpr PWMValue PWM_PERIOD = 100000;
  constexpr PWMValue PWM_EMPTY = 0;

  enum class Direction {
    Forward,
    Backward,
    Left,
    Right,
    Stop
  };

  enum class MotorNameTWD {
    Left,
    Right
  };

  enum class MotorNameFWD {
    LeftFront,
    LeftBack,
    RightFront,
    RightBack
  };

  enum class PinType {
    GPIO,
    PWM,
    I2C
  };

  enum class GPIOType {
    OUTPUT,
    INPUT
  };

  class Pin {
    protected:
      PinId pinId_;
      PinType pinType_;

    public:
      Pin(PinId pinId, PinType pinType);
      virtual ~Pin() = default;

      int checkValid();
      PinId getPinId();
      PinType getPinType();
      virtual void initialize() = 0;
  };

  class GPIOPin : public Pin {
    private:
      std::string name_;
      GPIOType GPIOType_;
      
    public:
      void setOUTPUT();
      void setINPUT();
      int getInputDigital();
      void setHIGH();
      void setLOW();
      void initialize() override;
      GPIOPin(PinId pinId, PinType pinType, GPIOType GPIOType_, const std::string name);
  };

  class PWMPin : public Pin {
    private:
      PWMValue period_;
      PWMValue duty_;
      std::string name_;

    public:
      int setPeriod(PWMValue period);
      PWMValue getPeriod();
      PWMValue getDuty();
      void openPWM();
      void closePWM();
      void setDuty(PWMValue duty);
      void initialize() override;
      PWMPin(PinId pinId, PinType pinType, PWMValue period, PWMValue duty = 0, const std::string name);
      ~PWMPin();
  };

  class MotorPins {
    protected:
      GPIOPin forward_;
      GPIOPin backward_;
      PWMPin speed_;

    public:
      MotorPins();
      MotorPins(PinId forwardPinId, PinId backwardPinId, PinId speedPinId, std::string name);
      virtual ~MotorPins() = default;
  };
};