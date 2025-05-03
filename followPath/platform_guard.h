#pragma once

#include "milkv_platform.h"

class PlatformGuard {
  public:
    PlatformGuard() {
      MotorControl::MilkVPlatform::initialize();
    }
    ~PlatformGuard() {
      MotorControl::MilkVPlatform::cleanup();
    }
};