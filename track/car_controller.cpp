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
#include "car_controller.h"

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
  void CarController <T>::move(Direction dir, Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75, bool setSpeed = true) {
    auto initializeSpeed = [this, speed]() noexcept -> void {
      for (const auto& motor : motors_) {
        motor->setSpeed(speed);
      }
    };

    auto excuteMove = [this, dir]() noexcept -> void {
      switch (dir) {
        case Direction::Forward:
          std::cout << "car is moving forward" << std::endl;
          motors_[MotorNameTWD::Left]->moveForward();
          motors_[MotorNameTWD::Right]->moveForward();
          break;
        case Direction::Backward:
          std::cout << "car is moving backward" << std::endl;
          motors_[MotorNameTWD::Left]->moveBackward();
          motors_[MotorNameTWD::Right]->moveBackward();
          break;
        case Direction::Left:
          std::cout << "car is turning left" << std::endl;
          motors_[MotorNameTWD::Left]->stop();
          motors_[MotorNameTWD::Right]->moveForward();
          break;
        case Direction::Right:
          std::cout << "car is turning right" << std::endl;
          motors_[MotorNameTWD::Left]->moveForward();
          motors_[MotorNameTWD::Right]->stop();
          break;
        case Direction::Stop:
          std::cout << "car is stop" << std::endl;
          motors_[MotorNameTWD::Left]->stop();
          motors_[MotorNameTWD::Right]->stop();
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
  void CarController <T>::forwardWithSpeed(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75) {
    move(Direction::Forward, duration, speed);
  }

  template <MotorType T>
  void CarController <T>::backwardWithSpeed(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75) {
    move(Direction::Backward, duration, speed);
  }

  template <MotorType T>
  void CarController <T>::turnLeftWithSpeed(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75) {
    move(Direction::Left, duration, speed);
  }

  template <MotorType T>
  void CarController <T>::turnRightWithSpeed(Duration duration = Duration(1), PWMValue speed = PWM_PERIOD * 0.75) {
    move(Direction::Right, duration, speed);
  }

  template <MotorType T>
  void CarController <T>::forward(Duration duration = Duration(1)) {
    move(Direction::Forward, duration, 0, false);
  }

  template <MotorType T>
  void CarController <T>::backward(Duration duration = Duration(1)) {
    move(Direction::Backward, duration, 0, false);
  }

  template <MotorType T>
  void CarController <T>::turnLeft(Duration duration = Duration(1)) {
    move(Direction::Left, duration, 0, false);
  }

  template <MotorType T>
  void CarController <T>::turnRight(Duration duration = Duration(1)) {
    move(Direction::Right, duration, 0, false);
  }

  template <MotorType T>
  void CarController <T>::setMotorSpeeds(PWMValue leftSpeed, PWMValue rightSpeed) {
    motors_[MotorNameTWD::Left]->setSpeed(leftSpeed);
    motors_[MotorNameTWD::Right]->setSpeed(rightSpeed);
  }

  template <MotorType T>
  void CarController <T>::stop() {
    move(Direction::Stop, Duration(0), PWM_EMPTY);
  }
}