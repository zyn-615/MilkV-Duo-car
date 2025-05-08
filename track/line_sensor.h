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
      virtual std::shared_ptr <ISensorState> getNowState() = 0;
      virtual std::string getName() const = 0;
  };

  class DigitalSensor : public ISensor {
    private:
      std::string name_;
      MotorControl::GPIOPin pin_;
      bool invert_;

    public:
      DigitalSensor(std::string name, MotorControl::PinId pinId, bool invert = false) : name_(name), 
      pin_(pinId, MotorControl::PinType::GPIO, MotorControl::GPIOType::INPUT, name), invert_(invert) {
      }

      std::shared_ptr <ISensorState> getNowState() override {
        bool nowState = pin_.getInputDigital() == HIGH;
        return std::make_shared <DigitalSensorState> (nowState);
      }

      std::string getName() const override {
        return name_;
      }
  };

  template <typename T>
  concept SensorType = std::derived_from <T, ISensor>;

  template <SensorType T>
  class SensorArray {
    private:
      std::vector <std::shared_ptr <T>> sensors_;
    
    public:
      // SensorArray(std::vector <std::shared_ptr <T>> sensors) : sensors_(std::move(sensors)) {}

      // template <typename... Sensors>
      // SensorArray(Sensors... sensor) {
      //   (sensors_.emplace_back(sensor), ...);
      // }

      // SensorArray(std::span <T> sensors) {
      //   for (const auto& sensor : sensors) {
      //     sensors_.emplace_back(std::make_shared <T> (sensor));
      //   }
      // }

      template <typename U> requires std::derived_from<U, T>
      SensorArray(std::span <std::shared_ptr <U>> sensors) : sensors_(sensors.begin(), sensors.end()) {}

      std::vector <std::shared_ptr <ISensorState>> getAllState() const {
        std::vector <std::shared_ptr <ISensorState>> state;
        state.reserve(sensors_.size());

        for (const auto& sensor : sensors_) {
          state.emplace_back(sensor->getNowState());
        }

        return state;
      }

      std::size_t getSize() const {
        return sensors_.size();
      }

      std::shared_ptr <T> getSensor(std::size_t sensorId) const {
        if (sensorId >= sensors_.size()) {
          throw std::out_of_range("sensorId out of range");
        }

        return sensors_.at(sensorId);
      }
  };
}