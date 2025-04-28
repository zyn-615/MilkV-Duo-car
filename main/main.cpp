#include <stdio.h>
#include <unistd.h>
#include <wiringx.h>
#include <stdlib.h>
#include "basic_config.h"
#include "motors_config.h"

int main(void) {
  setupMilkV();
  const int MODE = 2;

  printf("Platform: %s\n", wiringXPlatform());

  MotorPin* MotorPinList = initializeMotorPins(MODE);
  testGPIO(MotorPinList, MODE);
  initMotors(MotorPinList, MODE);
  setForwardFullSpeed(MotorPinList, MODE);

  moveAction(MotorPinList, MODE);

  deleteAllPointer(MotorPinList, MODE);
  closeMotor(MotorPinList, MODE);
  wiringXGC();
  return 0;
}