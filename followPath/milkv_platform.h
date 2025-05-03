#pragma once

#include <wiringx.h>
#include <iostream>
#include <stdexcept>

namespace MotorControl {
  class MilkVPlatform {
    private:
      static bool initialized_;

    public:
      static void initialize() {
        if (!initialized_) {
          if (wiringXSetup("milkv_duo", nullptr) == -1) {
            throw std::runtime_error("wiringX setup failed");
          }

          std::cout << "Platform: " << wiringXPlatform() << " initialized" << std::endl;
          initialized_ = true;
        }
      }

      static void cleanup() {
        if (initialized_) {
          wiringXGC();
          initialized_ = false;
          std::cout << "Platform resources released" << std::endl;
        }
      }
  };
}