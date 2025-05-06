#include "tracking_car.h"
#include "platform_guard.h"
#include "tracking_control.h"
#include "car_controller.h"
#include "motor_types.h"
#include "dc_motor.h"
#include <thread>

int main() {
  try {
      PlatformGuard platform;

      auto leftMotor = std::make_shared <MotorControl::DCMotor> (0, 1, 2, "leftMotor");
      auto rightMotor = std::make_shared <MotorControl::DCMotor> (3, 4, 5, "rightMotor");

      auto car = std::make_shared <MotorControl::CarController <MotorControl::DCMotor>> (*leftMotor, *rightMotor);

      car->forward();

      
  } catch (const std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl;
      return 1;
  }
  
  return 0;
}