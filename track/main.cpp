#include "tracking_car.h"
#include "platform_guard.h"
#include "tracking_control.h"
#include "car_controller.h"
#include "motor_types.h"
#include "dc_motor.h"
#include <thread>
#include <unistd.h>

int main() {
  try {
      PlatformGuard platform;
      std::cerr << "IJJI" << std::endl;
      auto leftMotor = MotorControl::DCMotor (0, 1, 2, "leftMotor");
      auto rightMotor = MotorControl::DCMotor (3, 4, 5, "rightMotor");

      auto car = std::make_shared <MotorControl::CarController <MotorControl::DCMotor>> (leftMotor, rightMotor);

      car->forward();
      sleep(1);
      car->turnLeft();
      sleep(1);
      car->backward();
      sleep(1);
      car->turnRight();
      
  } catch (const std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl;
      return 1;
  }
  
  return 0;
}