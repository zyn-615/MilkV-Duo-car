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

namespace Tracking {
  constexpr double EPS = 1e-4;

  template <typename T>
  struct Vector {
    std::vector <T> vec;
    Vector() {}
    Vector(std::vector <T> _vec) : vec(vec) {}

    template <typename... Params>
    Vector(Params... params) {
      (vec.emplace_back(params), ...);
    }

    friend double operator * (const Vector& a, const Vector& b) {
      double res = 0;
      int n = a.vec.size(), m = b.vec.size();
      if (n != m) {
        throw std::runtime_error("invalid dot");
      }

      for (int i = 0; i < n; ++i) {
        res += a.vec[i] * b.vec[i];
      }
      
      return res;
    }
  };

  struct PIDParams {
    Vector <double> pid;
    double maxLimit{std::numeric_limits <double>::infinity()};
    double minLimit{-std::numeric_limits <double>::infinity()};
    PIDParams() {}
    PIDParams(Vector <double> vec) : pid(vec) {}
  };

  class PIDController {
    private:
      PIDParams params_;
      double setpoint_{0.0};
      double lastError_{0.0};
      double integral_{0.0};
      std::chrono::steady_clock::time_point lastTime_;

    public:
      PIDController(PIDParams params) : params_(std::move(params)), lastTime_(std::chrono::steady_clock::now()) {}

      void setSetpoint(double setpoint) {
        setpoint_ = setpoint;
      }

      double calc(double nowState) {
        auto now = std::chrono::steady_clock::now();
        auto deltaT = std::chrono::duration <double> (now - lastTime_).count();
        lastTime_ = now;

        if (deltaT <= 0) {
          deltaT = EPS;
        }

        double nowError = setpoint_ - nowState;
        integral_ += deltaT * nowError;
        double derivative = (nowError - lastError_) / deltaT;
        lastError_ = nowError;

        auto nowVec = Vector <double> (nowError, integral_, derivative);
        double res = nowVec * params_.pid;

        return std::clamp(res, params_.minLimit, params_.maxLimit);
      }

      void reset() {
        lastTime_ = std::chrono::steady_clock::now();
        lastError_ = 0;
        integral_ = 0;
      }

      const PIDParams& getParams() const {
        return params_;
      }
      
      void setParams(const PIDParams& params) {
        params_ = params;
      }
    };
};