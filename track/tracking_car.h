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
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "line_sensor.h"
#include "pid_control.h"
#include "tracking_control.h"
#include "car_controller.h"

namespace Tracking {
  enum class SteeringMode {
    Diff,
    HigherDiff,
    Counter,
    Dynamic
  };

  template <typename T>
  concept TrackingControlType = std::derived_from <T, ITrackingControl> ;

  template <MotorControl::MotorType MotorType, TrackingControlType TrackingType>
  class TrackingCar {
    private:
      std::shared_ptr <MotorControl::CarController <MotorType>> carController_;
      std::shared_ptr <TrackingType> track_;
      double baseSpeed_;
      double minSpeed_;
      double maxValue_;
      double turningSpeed_;
      double lastPositon_{0.0};
      double curvature_{0.0};
      double reduction_{0.1};
      Vector <double> coefficient{0.7, 0.3};
      bool isStarted_{false};
      SteeringMode steeringMode_;

      double speedMap(double preSpeed) {
        const double low = 0.2;
        if (preSpeed < low) return 0.0;
        double map = (preSpeed - low) / (1.0 - low);//to [0,1]
        double power = 2.0;
        double mappedSpeed = 0.65 + 0.35 * std::pow(map, power);
        return mappedSpeed * MotorControl::PWM_PERIOD;
      }

      double mapMotorResponse(double value) {
        double normalizedSpeed = value / MotorControl::PWM_PERIOD;
        normalizedSpeed = std::clamp(normalizedSpeed, 0.0, 1.0);
        return speedMap(normalizedSpeed);
      }

      double calcNewSpeed() {
        double factor = 1.0 - (curvature_ * reduction_);
        factor = std::clamp(factor, 0.3, 1.0);
        return minSpeed_ + (baseSpeed_ - minSpeed_) * factor;
      }

      void applyDiff(const double value) {
        double absValue = std::abs(value);

        double turnValue = value * 2.0;
        turnValue = std::clamp(turnValue, -1.0, 1.0);
        
        double adjustedBase = baseSpeed_ * (1.0 - curvature_ * reduction_);
        adjustedBase = std::max(adjustedBase, minSpeed_);
        
        double leftSpeed = adjustedBase;
        double rightSpeed = adjustedBase;
        
        if (turnValue > 0) {
          rightSpeed = adjustedBase * (1.0 - turnValue);
        } else { 
          leftSpeed = adjustedBase * (1.0 + turnValue);
        }
        
        leftSpeed = std::clamp(leftSpeed, minSpeed_, baseSpeed_);
        rightSpeed = std::clamp(rightSpeed, minSpeed_, baseSpeed_);
        
        leftSpeed = mapMotorResponse(leftSpeed);
        rightSpeed = mapMotorResponse(rightSpeed);
        
        carController_->setMotorSpeeds(leftSpeed, rightSpeed);
        carController_->forward(std::chrono::seconds(0));

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Diff MODE   leftSpeed :  " << leftSpeed << " " << " rightSpeed : " << rightSpeed << std::endl;
        std::cout << std::defaultfloat;
      }

      void applyHigherDiff(const double value) {
        double abs_value = std::abs(value);
        
        std::cout << std::fixed << std::setprecision(4);
        if (value > 0) { 
          double leftSpeed = turningSpeed_;
          double rightSpeed = minSpeed_ * (1.0 - abs_value);
          
          leftSpeed = mapMotorResponse(leftSpeed);
          rightSpeed = mapMotorResponse(rightSpeed);
          
          carController_->setMotorSpeeds(leftSpeed, rightSpeed);
          carController_->forward(std::chrono::seconds(0));
          
          std::cout << "Rotational - Left: " << leftSpeed << ", Right: " << rightSpeed << std::endl;
        } else { 
          double leftSpeed = minSpeed_ * (1.0 - abs_value);
          double rightSpeed = turningSpeed_;
          
          leftSpeed = mapMotorResponse(leftSpeed);
          rightSpeed = mapMotorResponse(rightSpeed);
          
          carController_->setMotorSpeeds(leftSpeed, rightSpeed);
          carController_->forward(std::chrono::seconds(0));
          std::cout << "Rotational - left: " << leftSpeed << ", Right: " << rightSpeed << std::endl;
        }

        std::cout << std::defaultfloat;
        // std::cout << std::fixed << std::setprecision(4);
        // std::cout << "HigherDiff MODE   leftSpeed :  " << leftSpeed << " " << " rightSpeed : " << rightSpeed << std::endl;
        // std::cout << std::defaultfloat;
      }

      void applyCounter(const double value) {
        double motor_speed = turningSpeed_;
        double abs_value = std::abs(value);
        
        double turning_intensity = std::min(abs_value * 2.0, 1.0);
        
        if (value > 0) {
          if (abs_value > 0.3) { 
            carController_->turnRightWithSpeed(std::chrono::seconds(0), motor_speed);
            std::cout << "Counter Right Turn - Full Speed: " << motor_speed << std::endl;
          } else {
            applyHigherDiff(value);
          }
        } else {
          if (abs_value > 0.3) { 
            carController_->turnLeftWithSpeed(std::chrono::seconds(0), motor_speed);
            std::cout << "Counter Left Turn - Full Speed: " << motor_speed << std::endl;
          } else {
            applyHigherDiff(value);
          }
        }
        
        // std::cout << std::fixed << std::setprecision(4);
        // std::cout << "Counter - Left: " << leftSpeed << ", Right: " << rightSpeed << std::endl;
        // std::cout << std::defaultfloat;
      }

      void applyDynamic(const double value, double position) {
        double absValue = std::abs(value);
        
         if (absValue > 0.6 || curvature_ > 0.15) {
          std::cout << "Dynamic Using Counter Steering for sharp turn" << std::endl;
          applyCounter(value);
        } else if (absValue > 0.3 || curvature_ > 0.05) {
          std::cout << "Dynamic Using Higher Diff for medium turn" << std::endl;
          applyHigherDiff(value);
        } else {
          std::cout << "Dynamic Using Diff for slight value" << std::endl;
          applyDiff(value);
        }
      }
    
    public:
      TrackingCar(
        std::shared_ptr <MotorControl::CarController <MotorType>> carController, std::shared_ptr <TrackingType> track,
        double baseSpeed = MotorControl::PWM_PERIOD * 0.5, double turningSpeed = MotorControl::PWM_PERIOD, 
        double minSpeed = MotorControl::PWM_PERIOD * 0.65, double maxValue = 1.0,
        SteeringMode steeringMode = SteeringMode::Dynamic) 
        : carController_(carController), track_(track), baseSpeed_(baseSpeed), turningSpeed_(turningSpeed),
        minSpeed_(minSpeed), maxValue_(maxValue), steeringMode_(steeringMode) {}
      
      void start() {
        isStarted_ = true;
        lastPositon_ = 0.0;
        curvature_ = 0.0;
        std::cout << "Tracking started" << std::endl;
      }

      void end() {
        isStarted_ = false;
        carController_->stop();
        std::cout << "Traking ended" << std::endl;
      }

      void update() {
        if (!isStarted_) return ;

        track_->update();
        double curPostion = track_->getPositon(); 
        double curValue = track_->getValue();

        curValue = std::clamp(curValue, -maxValue_, maxValue_);
        double deltaPositon = std::abs(curPostion - lastPositon_);
        Vector <double> nowVector{curvature_, deltaPositon};
        curvature_ = nowVector * coefficient;
        lastPositon_ = curPostion;

        // double updateSpeed = calcNewSpeed();
        switch (steeringMode_) {
          case SteeringMode::Diff:
            applyDiff(curValue);
            break;
          case SteeringMode::HigherDiff:
            applyHigherDiff(curValue);
            break;
          case SteeringMode::Counter:
            applyCounter(curValue);
            break;
          case SteeringMode::Dynamic:
            applyDynamic(curValue, curPostion);
            break;
        }
        
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "pos : " << curPostion << "  value : " << curValue << std::endl;
        std::cout << "curvatrue :  " << curvature_ << "   speed : nn " << std::endl;  
        std::cout << std::defaultfloat; 
      }
  };
};