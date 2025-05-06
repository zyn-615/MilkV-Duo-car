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
#include <thread>
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
      CarController() {}
      void checkValid();
      void move(Direction dir, Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75, bool setSpeed = true);
      void forwardWithSpeed(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75);
      void backwardWithSpeed(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75);
      void turnLeftWithSpeed(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75);
      void turnRightWithSpeed(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75);
      void forward(Duration duration = Duration(1));
      void backward(Duration duration = Duration(1));
      void turnLeft(Duration duration = Duration(1));
      void turnRight(Duration duration = Duration(1));
      void setMotorSpeeds(PWMValue leftSpeed, PWMValue rightSpeed);
      void stop();
  };
}

namespace MotorControl {
  template <MotorType T>
  template <typename... Motor>
  CarController <T>::CarController(Motor... motor) {
      (motors_.emplace_back(std::make_shared <T> (motor)), ...);
      checkValid();
    }

  template <MotorType T>
  CarController <T>::CarController(std::span <T> motors) {
    for (const auto& motor : motors) {
      motors_.emplace_back(std::make_shared <T> (motor));
    }
    checkValid();
  }

  template <MotorType T>
  CarController <T>::CarController(std::span <std::shared_ptr <T>> motors) : motors_(motors.begin(), motors.end()) {
    checkValid();
  }

  template <MotorType T>
  void CarController <T>::checkValid() {
    if (motors_.size() != 2 && motors_.size() != 4) {
      throw std::runtime_error("invalid motor number");
    } else {
      std::cout << "valid motor number" << std::endl;
    }
  }

  template <MotorType T>
  void CarController <T>::move(Direction dir, Duration duration, PWMValue speed, bool setSpeed) {
    auto initializeSpeed = [this, speed]() noexcept -> void {
      for (const auto& motor : motors_) {
        motor->setSpeed(speed);
      }
    };

    auto excuteMove = [this, dir]() noexcept -> void {
      switch (dir) {
        case Direction::Forward:
          std::cout << "car is moving forward" << std::endl;
          motors_[static_cast<std::size_t>(MotorNameTWD::Left)]->forward();
          motors_[static_cast<std::size_t>(MotorNameTWD::Right)]->forward();
          break;
        case Direction::Backward:
          std::cout << "car is moving backward" << std::endl;
          motors_[static_cast<std::size_t>(MotorNameTWD::Left)]->backward();
          motors_[static_cast<std::size_t>(MotorNameTWD::Right)]->backward();
          break;
        case Direction::Left:
          std::cout << "car is turning left" << std::endl;
          motors_[static_cast<std::size_t>(MotorNameTWD::Left)]->backward();
          motors_[static_cast<std::size_t>(MotorNameTWD::Right)]->forward();
          break;
        case Direction::Right:
          std::cout << "car is turning right" << std::endl;
          motors_[static_cast<std::size_t>(MotorNameTWD::Left)]->forward();
          motors_[static_cast<std::size_t>(MotorNameTWD::Right)]->backward();
          break;
        case Direction::Stop:
          std::cout << "car is stop" << std::endl;
          motors_[static_cast<std::size_t>(MotorNameTWD::Left)]->stop();
          motors_[static_cast<std::size_t>(MotorNameTWD::Right)]->stop();
          break;
      }
    };

    if (setSpeed) {
      initializeSpeed();
    }

    excuteMove();

    if (duration.count() > 0) {
      std::this_thread::sleep_for(duration);
      excuteMove();
    }
  }

  template <MotorType T>
  void CarController <T>::forwardWithSpeed(Duration duration, PWMValue speed) {
    move(Direction::Forward, duration, speed, true);
  }

  template <MotorType T>
  void CarController <T>::backwardWithSpeed(Duration duration, PWMValue speed) {
    move(Direction::Backward, duration, speed, true);
  }

  template <MotorType T>
  void CarController <T>::turnLeftWithSpeed(Duration duration, PWMValue speed) {
    move(Direction::Left, duration, speed, true);
  }

  template <MotorType T>
  void CarController <T>::turnRightWithSpeed(Duration duration, PWMValue speed) {
    move(Direction::Right, duration, speed, true);
  }

  template <MotorType T>
  void CarController <T>::forward(Duration duration) {
    move(Direction::Forward, duration, 0, false);
  }

  template <MotorType T>
  void CarController <T>::backward(Duration duration) {
    move(Direction::Backward, duration, 0, false);
  }

  template <MotorType T>
  void CarController <T>::turnLeft(Duration duration) {
    move(Direction::Left, duration, 0, false);
  }

  template <MotorType T>
  void CarController <T>::turnRight(Duration duration) {
    move(Direction::Right, duration, 0, false);
  }

  template <MotorType T>
  void CarController <T>::setMotorSpeeds(PWMValue leftSpeed, PWMValue rightSpeed) {
    motors_[static_cast<std::size_t>(MotorNameTWD::Left)]->setSpeed(leftSpeed);
    motors_[static_cast<std::size_t>(MotorNameTWD::Right)]->setSpeed(rightSpeed);
    motors_[static_cast<std::size_t>(MotorNameTWD::Left)]->moveForward();

  }

  template <MotorType T>
  void CarController <T>::stop() {
    move(Direction::Stop, Duration(0), PWM_EMPTY);
  }
}
