#pragma once

#include <wiringx.h>
#include <string>
#include <chrono>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <memory>
#include <concepts>
#include <span>
#include "line_sensor.h"
#include "pid_control.h"

bool Break = false;

namespace Tracking {
  class ITrackingControl {
    public:
      virtual ~ITrackingControl() = default;
      virtual void update() = 0;
      virtual double getValue() const = 0;
      virtual double getPositon() const = 0;
  };
  
  template <typename SensorT> requires std::derived_from<SensorT, ISensor>
  class PIDTrackingControl : public ITrackingControl {
    private:
      std::string name_;
      std::shared_ptr <SensorArray <SensorT>> sensors_;
      std::shared_ptr <PIDController> PIDController_;
      double positon_{0.0};
      double value_{0.0};

      double processData(const std::vector <std::shared_ptr <ISensorState>>& state) {
        if (state.empty()) return 0.0;

        const int sensorNum = state.size();
        int onLineNum = 0;
        double baseValue = 0;
        double value = 0;

        for (int i = 0; i < sensorNum; ++i) {
          if (Break) {
            if (i == 1 || i == 6) continue;
          }
          
          if (state[i]->isOnLine()) {
            ++onLineNum;
            double weight = 2.0 * i / (sensorNum - 1) - 1.0;
            value += weight;
            baseValue += 1;
          }
        }

        if(!onLineNum) {
          return 0;
        }

        return value / baseValue;
      }
    
    public:
      PIDTrackingControl(std::string name, std::shared_ptr <SensorArray <SensorT>> sensors, std::shared_ptr <PIDController> _PIDController)
      : name_(name), sensors_(sensors), PIDController_(_PIDController) {
        PIDController_->setSetpoint(0.0);
      }

      void update() override {
        auto curState = sensors_->getAllState();
        positon_ = processData(curState);
        value_ = PIDController_->calc(positon_);

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "update : " << name_ << " " << "position : " << positon_ << " ";
        std::cout << "Value : " << value_ << std::endl;
        std::cout << std::defaultfloat; 
      }

      double getValue() const override {
        return value_;
      }

      double getPositon() const override {
        return positon_;
      }
  };

  
  
}