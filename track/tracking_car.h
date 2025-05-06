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
      double lastPositon_{0.0};
      double curvature_{0.0};
      double reduction_{0.7};
      Vector <double> coefficient{0.7, 0.3};
      bool isStarted_{false};
      SteeringMode steeringMode_;

      double calcNewSpeed() {
        double factor = 1.0 - (curvature_ * reduction_);
        factor = std::clamp(factor, 0.3, 1.0);
        return minSpeed_ + (baseSpeed_ - minSpeed_) * factor;
      }

      void applyDiff(const double value, const double speed) {
        double leftSpeed = speed;
        double rightSpeed = speed;

        if (value > 0) {
          rightSpeed = speed * (1 - value);
        }

        if (value < 0) {
          leftSpeed = speed * (1 + value);
        }

        leftSpeed = std::max(leftSpeed, minSpeed_);
        rightSpeed = std::max(rightSpeed, minSpeed_);
        carController_->setMotorSpeeds(leftSpeed, rightSpeed);
        carController_->forward(std::chrono::seconds(0));

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Diff MODE   leftSpeed :  " << leftSpeed << " " << " rightSpeed : " << rightSpeed << std::endl;
        std::cout << std::defaultfloat;
      }

      void applyHigherDiff(const double value, const double speed) {
        double leftSpeed = speed;
        double rightSpeed = speed;

        if (value > 0) {
          rightSpeed = speed * (1 - value * 1.5);
        }

        if (value < 0) {
          leftSpeed = speed * (1 + value * 1.5);
        }

        leftSpeed = (leftSpeed < 0) ? std::max(leftSpeed, -minSpeed_) : std::max(leftSpeed, minSpeed_);
        rightSpeed = (rightSpeed < 0) ? std::max(rightSpeed, -minSpeed_) : std::max(rightSpeed, minSpeed_);

        carController_->setMotorSpeeds(std::abs(leftSpeed), std::abs(rightSpeed));
        
        //左反，右正
        if (leftSpeed < 0 && rightSpeed >= 0) {
          carController_->turnLeft(std::chrono::seconds(0));
        } else if (leftSpeed >= 0 && rightSpeed < 0) {
          carController_->turnRight(std::chrono::seconds(0));
        } else {
          carController_->forward(std::chrono::seconds(0));
        }

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "HigherDiff MODE   leftSpeed :  " << leftSpeed << " " << " rightSpeed : " << rightSpeed << std::endl;
        std::cout << std::defaultfloat;
      }

      void applyCounter(const double value, const double speed) {
        double leftSpeed = speed;
        double rightSpeed = speed;

        double absValue = std::abs(value);
        if (absValue > 0.4) {
          if (value > 0) { // 需要向右修正
              leftSpeed = speed;
              rightSpeed = -speed * (absValue - 0.4) * 2.5; // 将修正值转换为反向速度
          } else { // 需要向左修正
              leftSpeed = -speed * (absValue - 0.4) * 2.5; // 将修正值转换为反向速度
              rightSpeed = speed;
          }
        } else { // 低修正值使用常规差速
          if (value > 0) { // 需要向右修正
              rightSpeed = speed * (1.0 - value * 2);
          } else { // 需要向左修正
              leftSpeed = speed * (1.0 + value * 2);
          }
        }
      
        carController_->setMotorSpeeds(std::abs(leftSpeed), std::abs(rightSpeed));
      
        if (leftSpeed < 0 && rightSpeed >= 0) {
          // 左轮后退，右轮前进
          carController_->turnLeft(std::chrono::seconds(0));
        } else if (leftSpeed >= 0 && rightSpeed < 0) {
          // 左轮前进，右轮后退
          carController_->turnRight(std::chrono::seconds(0));
        } else {
          carController_->forward(std::chrono::seconds(0));
        }
        
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Counter - Left: " << leftSpeed << ", Right: " << rightSpeed << std::endl;
        std::cout << std::defaultfloat;
      }

      void applyDynamic(const double value, const double speed, double position) {
        double absValue = std::abs(value);
        
        // 判断当前线的形状和需求
        if (absValue > 0.7 || curvature_ > 0.2) {
            // 急弯或大修正：使用反向转向
            applyCounter(value, speed);
        } else if (absValue > 0.3 || curvature_ > 0.1) {
            // 中等弯道：使用旋转差速
            applyHigherDiff(value, speed);
        } else {
            // 直线或小弯：使用普通差速
            applyDiff(value, speed);
        }
      }
    
    public:
      TrackingCar(
        std::shared_ptr <MotorControl::CarController <MotorType>> carController, std::shared_ptr<TrackingType> track,
        double baseSpeed = MotorControl::PWM_PERIOD / 2, double minSpeed = MotorControl::PWM_PERIOD / 5, double maxValue = 1.0,
        SteeringMode steeringMode = SteeringMode::Dynamic) 
        : carController_(std::move(carController)), track_(std::move(track)), baseSpeed_(baseSpeed),
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

        double updateSpeed = calcNewSpeed();
        switch (steeringMode_) {
          case SteeringMode::Diff:
            applyDiff(curValue, updateSpeed);
            break;
          case SteeringMode::HigherDiff:
            applyHigherDiff(curValue, updateSpeed);
            break;
          case SteeringMode::Counter:
            applyCounter(curValue, updateSpeed);
            break;
          case SteeringMode::Dynamic:
            applyDynamic(curValue, updateSpeed, curPostion);
            break;
        }
        
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "pos : " << curPostion << "  value : " << curValue << std::endl;
        std::cout << "curvatrue :  " << curvature_ << "   speed :  " << updateSpeed << std::endl;  
        std::cout << std::defaultfloat; 
      }
  };
};