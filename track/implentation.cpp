#include <string>
#include <chrono>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <memory>
#include <wiringx.h>
#include "motor_types.h"
#include "dc_motor.h"
#include "milkv_platform.h"

namespace MotorControl {
  Pin::Pin(PinId pinId, PinType pinType) : pinId_(pinId), pinType_(pinType) {}

  int Pin::checkValid() {
    return wiringXValidGPIO(pinId_);
  }

  PinId Pin::getPinId() {
    return pinId_;
  }

  PinType Pin::getPinType() {
    return pinType_;
  }

  GPIOPin::GPIOPin(PinId pinId, PinType pinType, GPIOType GPIOType_, const std::string name) 
    : Pin(pinId, pinType), name_(name), GPIOType_(GPIOType_) {
    initialize();
  }

  void GPIOPin::setOUTPUT() {
    pinMode(pinId_, PINMODE_OUTPUT);
  }

  void GPIOPin::setINPUT() {
    pinMode(pinId_, PINMODE_INPUT);
  }

  int GPIOPin::getInputDigital() {
    return digitalRead(pinId_);
  }

  void GPIOPin::setHIGH() {
    digitalWrite(pinId_, HIGH);
  }

  void GPIOPin::setLOW() {
    digitalWrite(pinId_, LOW);
  }

  GPIOPin::~GPIOPin() {
    setLOW();
    std::cout << "GPIOPin : " << pinId_ << " closed" << std::endl; 
  }

  void GPIOPin::initialize() {
    if (checkValid() != 0) {
      throw std::runtime_error("Invalid GPIO pinId : " + std::to_string(pinId_) + " for " + name_);
    }

    if (GPIOType_ == GPIOType::OUTPUT) {
      setOUTPUT();
      setLOW();
    } else {
      setINPUT();
    }

    std::cout << "Valid GPIO pinId : " << pinId_ << " for " << name_ << std::endl;
  }

  PWMPin::PWMPin(PinId pinId, PinType pinType, PWMValue period, PWMValue duty, const std::string name) : 
    Pin(pinId, pinType), period_(period), duty_(duty), name_(name) {
    initialize();
  }

  int PWMPin::setPeriod(PWMValue period) {
    period_ = period;
    return wiringXPWMSetPeriod(pinId_, period);
  }

  PWMValue PWMPin::getPeriod() {
    return period_;
  }

  PWMValue PWMPin::getDuty() {
    return duty_;
  }

  void PWMPin::openPWM() {
    wiringXPWMEnable(pinId_, 1);
  }

  void PWMPin::closePWM() {
    wiringXPWMEnable(pinId_, 0);
  }

  void PWMPin::setDuty(PWMValue duty) {
    duty_ = duty;
    wiringXPWMSetDuty(pinId_, duty_);
  }

  void PWMPin::initialize() {
    if (checkValid() != 0) {
      throw std::runtime_error("Invalid PWM pinId : " + std::to_string(pinId_) + " for " + name_);
    }
    
    if (setPeriod(period_) != 0) {
      throw std::runtime_error("Pin " + std::to_string(pinId_) + " does not support PWM for " + name_);
    }

    setDuty(duty_);
    openPWM();
    std::cout << "Valid PWM pinId : " << pinId_ << " for " << name_ << std::endl;
  }

  PWMPin::~PWMPin() {
    closePWM();
    std::cout << "PWM pin " << pinId_ << " closed" << std::endl;
  }

  MotorPins::MotorPins(PinId forwardPinId, PinId backwardPinId, PinId speedPinId, std::string name) :
    forward_(forwardPinId, PinType::GPIO, GPIOType::OUTPUT, name + "-forward"),
    backward_(backwardPinId, PinType::GPIO, GPIOType::OUTPUT, name + "-backward"),
    speed_(speedPinId, PinType::PWM, PWM_PERIOD, PWM_EMPTY, name + "-speed") {}
  
  DCMotor::DCMotor(PinId forward, PinId backward, PinId speed, const std::string name) : name_(name), MotorPins(forward, backward, speed, name) {
    std::cout << "DCMotor " << name_ << " initialized" << std::endl;
  }

  void DCMotor::forward() {
    std::cout << name_ << " moving forward" << std::endl;
    forward_.setHIGH();
    backward_.setLOW();
  }

  void DCMotor::backward()  {
    std::cout << name_ << " moving backward" << std::endl;
    forward_.setLOW();
    backward_.setHIGH();
  }

  void DCMotor::stop() {
    std::cout << name_ << " stopping" << std::endl;
    forward_.setLOW();
    backward_.setLOW();
  }

  void DCMotor::setSpeed(PWMValue duty) {
    std::cout << name_ << " setting speed to " << duty << std::endl;
    speed_.setDuty(duty);
  }

  const std::string& DCMotor::getName() const {
    return name_;
  }

  DCMotor::~DCMotor() {
    stop();
    std::cout << "DCMotor " << name_ << " destroyed" << std::endl;
  }

  bool MilkVPlatform::initialized_ = false;
}