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
      double reduction_{0.7};
      Vector <double> coefficient{0.7, 0.3};
      bool isStarted_{false};
      SteeringMode steeringMode_;

      double speedMap(double preSpeed) {
        const double low = 0.2;
        if (preSpeed < low) return 0.0;
        double map = (preSpeed - low) / (1.0 - low);//to [0,1]
        double power = 1.5;
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

        double turn_ratio = value * 2.0; // 放大修正值的影响
        turn_ratio = std::clamp(turn_ratio, -1.0, 1.0);
        
        // 基础速度根据曲率动态调整
        double adjusted_base = baseSpeed_ * (1.0 - curvature_ * reduction_);
        adjusted_base = std::max(adjusted_base, minSpeed_);
        
        // 计算左右轮速度
        double leftSpeed = adjusted_base;
        double rightSpeed = adjusted_base;
        
        if (turn_ratio > 0) { // 向右转
          // 向右转时减少右轮速度
          rightSpeed = adjusted_base * (1.0 - turn_ratio);
        } else { // 向左转
          // 向左转时减少左轮速度
          leftSpeed = adjusted_base * (1.0 + turn_ratio);
        }
        
        // 确保速度在有效范围内
        leftSpeed = std::clamp(leftSpeed, minSpeed_, baseSpeed_);
        rightSpeed = std::clamp(rightSpeed, minSpeed_, baseSpeed_);
        
        // 映射到非线性电机响应
        leftSpeed = mapMotorResponse(leftSpeed);
        rightSpeed = mapMotorResponse(rightSpeed);
        
        // 应用速度到电机
        carController_->setMotorSpeeds(leftSpeed, rightSpeed);
        carController_->forward(std::chrono::seconds(0));

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Diff MODE   leftSpeed :  " << leftSpeed << " " << " rightSpeed : " << rightSpeed << std::endl;
        std::cout << std::defaultfloat;
      }

      void applyHigherDiff(const double value) {
        double abs_value = std::abs(value);
        
        // 以较大的速度差进行转向
        if (value > 0) { // 向右转
          double leftSpeed = turningSpeed_;
          double rightSpeed = minSpeed_ * (1.0 - abs_value);
          
          // 映射到非线性电机响应
          leftSpeed = mapMotorResponse(leftSpeed);
          rightSpeed = mapMotorResponse(rightSpeed);
          
          carController_->setMotorSpeeds(leftSpeed, rightSpeed);
          carController_->forward(std::chrono::seconds(0));
          
          std::cout << "Rotational - Left: " << leftSpeed << ", Right: " << rightSpeed << std::endl;
        } else { // 向左转
          double leftSpeed = minSpeed_ * (1.0 - abs_value);
          double rightSpeed = turningSpeed_;
          
          // 映射到非线性电机响应
          leftSpeed = mapMotorResponse(leftSpeed);
          rightSpeed = mapMotorResponse(rightSpeed);
          
          carController_->setMotorSpeeds(leftSpeed, rightSpeed);
          carController_->forward(std::chrono::seconds(0));
          std::cout << "Rotational - left: " << leftSpeed << ", Right: " << rightSpeed << std::endl;
        }


        // std::cout << std::fixed << std::setprecision(4);
        // std::cout << "HigherDiff MODE   leftSpeed :  " << leftSpeed << " " << " rightSpeed : " << rightSpeed << std::endl;
        // std::cout << std::defaultfloat;
      }

      void applyCounter(const double value) {
        double motor_speed = turningSpeed_;
        double abs_value = std::abs(value);
        
        // 修正值越大，转向力度越大
        double turning_intensity = std::min(abs_value * 2.0, 1.0);
        
        if (value > 0) { // 向右转
          if (abs_value > 0.3) { // 只有在足够大的修正值时才使用反向转向
            // 右轮反转
            carController_->turnRightWithSpeed(std::chrono::seconds(0), motor_speed);
            std::cout << "Counter Right Turn - Full Speed: " << motor_speed << std::endl;
          } else {
            // 小修正使用差速
            applyHigherDiff(value);
          }
        } else { // 向左转
          if (abs_value > 0.3) { // 只有在足够大的修正值时才使用反向转向
            // 左轮反转
            carController_->turnLeftWithSpeed(std::chrono::seconds(0), motor_speed);
            std::cout << "Counter Left Turn - Full Speed: " << motor_speed << std::endl;
          } else {
            // 小修正使用差速
            applyHigherDiff(value);
          }
        }
        
        // std::cout << std::fixed << std::setprecision(4);
        // std::cout << "Counter - Left: " << leftSpeed << ", Right: " << rightSpeed << std::endl;
        // std::cout << std::defaultfloat;
      }

      void applyDynamic(const double value, double position) {
        double absValue = std::abs(value);
        
         // 急弯检测 - 根据修正值和曲率选择策略
         if (absValue > 0.6 || curvature_ > 0.15) {
          // 明显偏离 - 使用反向转向以快速修正
          std::cout << "Dynamic - Using Counter Steering for sharp turn" << std::endl;
          applyCounter(value);
        } else if (absValue > 0.3 || curvature_ > 0.05) {
          // 中等偏离 - 旋转差速提供强力转向
          std::cout << "Dynamic - Using Rotational Differential for medium turn" << std::endl;
          applyHigherDiff(value);
        } else {
          // 轻微偏离 - 普通差速足够
          std::cout << "Dynamic - Using Differential for slight value" << std::endl;
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