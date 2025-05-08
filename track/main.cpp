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
      int pwmIdLeft = 2;
      int pwmIdRight = 9;
      // double sp = 2.0;
      std::cout << "input pwmleft and pwm right" << std::endl;
      // std::cin >> pwmIdLeft >> pwmIdRight;
      std::cerr << "IJJI" << std::endl;
      std::cout << "input speed  base and turn" << std::endl;
      // std::cin >> sp;

      double base = 2, turn = 2;
      std::cin >> base >> turn;

      int _Broken = 0;
      std::cout << "Broken ?" << std::endl;
      std::cin >> _Broken;
      Broken = _Broken > 0;

      int seconds = 10;
      std::cout << "second : " << std::endl;
      std::cin >> seconds;

      auto leftMotor = std::make_shared <MotorControl::DCMotor> (0, 1, pwmIdLeft, "leftMotor");
      auto rightMotor = std::make_shared <MotorControl::DCMotor> (6, 7, pwmIdRight, "rightMotor");
      std::vector <std::shared_ptr <MotorControl::DCMotor>> Motors = {leftMotor, rightMotor};

      auto car = std::make_shared <MotorControl::CarController <MotorControl::DCMotor>> (Motors);
      
      Tracking::PIDParams pid(Tracking::Vector <double> (10, 0.01, 0.5));
      auto pidCon = std::make_shared <Tracking::PIDController> (pid);
      
      constexpr int sensorNum = 8;
      std::array <std::shared_ptr <Tracking::DigitalSensor>, sensorNum> sensors;

      MotorControl::PinId mapId[] = {26, 22, 21, 20, 19, 18, 17, 16};

      for (int i = 0; i < sensorNum; ++i) {
        sensors[i] = std::make_shared <Tracking::DigitalSensor> (std::to_string(i + 1) + " sensor", mapId[i]);
      }

      std::span <std::shared_ptr <Tracking::DigitalSensor>> sensorSpan(sensors);

      auto sensorsArray = std::make_shared <Tracking::SensorArray <Tracking::DigitalSensor>> (sensorSpan);
      auto pidTracking = std::make_shared <Tracking::PIDTrackingControl <Tracking::DigitalSensor>> ("pidTrackingControl", sensorsArray, pidCon);

      auto trackingCar = std::make_shared <Tracking::TrackingCar <MotorControl::DCMotor, Tracking::PIDTrackingControl <Tracking::DigitalSensor>>> 
      (car, pidTracking, MotorControl::PWM_PERIOD * base, MotorControl::PWM_PERIOD * turn, MotorControl::PWM_PERIOD * 0.65, 1.0);

      trackingCar->start();
      int testCase = 2;
      std::cout << "input testCase : " << std::endl;
      std::cin >> testCase;

      for (int t = 1; t <= testCase; ++t) {
        trackingCar->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(seconds));
      }

      trackingCar->end();

      // auto car = std::make_shared <MotorControl::CarController <MotorControl::DCMotor>> (leftMotor, rightMotor);

      // car->forwardWithSpeed(std::chrono::seconds(0), MotorControl::PWM_PERIOD * sp);
      // sleep(1);
      // car->turnLeftWithSpeed(std::chrono::seconds(0), MotorControl::PWM_PERIOD);
      // sleep(1);
      // car->backward();
      // sleep(1);
      // car->turnRight();
      
  } catch (const std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl;
      return 1;
  }
  
  return 0;
}