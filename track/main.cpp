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
      int pwmIdLeft = 0;
      int pwmIdRight = 0;
      std::cout << "input pwmleft and pwm right" << std::endl;
      std::cin >> pwmIdLeft >> pwmIdRight;
      std::cerr << "IJJI" << std::endl;
      auto leftMotor = std::make_shared <MotorControl::DCMotor> (0, 1, pwmIdLeft, "leftMotor");
      auto rightMotor = std::make_shared <MotorControl::DCMotor> (6, 7, pwmIdRight, "rightMotor");
      std::vector <std::shared_ptr <MotorControl::DCMotor>> Motors = {leftMotor, rightMotor};

      auto car = std::make_shared <MotorControl::CarController <MotorControl::DCMotor>> (Motors);
      // auto car = std::make_shared <MotorControl::CarController <MotorControl::DCMotor>> (leftMotor, rightMotor);

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