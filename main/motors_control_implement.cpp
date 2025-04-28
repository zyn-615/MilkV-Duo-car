#include <stdio.h>
#include <unistd.h>
#include <wiringx.h>
#include <stdlib.h>
#include <iostream>
#include "basic_config.h"
#include "motors_config.h"

MotorPin newMotorPin(int forwardId, int backwardId, int speedId) {
  int* fId = (int*) malloc(sizeof(int));
  int* bId = (int*) malloc(sizeof(int));
  int* sId = (int*) malloc(sizeof(int));

  if (fId == NULL || bId == NULL || sId == NULL) {
    fprintf(stderr, "Memory allocation failed\n");
    exit(1);
  }

  *fId = forwardId;
  *bId = backwardId;
  *sId = speedId;

  MotorPin result = {fId, bId, sId};
  return result;
}

void deleteMotorPin(MotorPin* p) {
  free((void*)p->forward);
  free((void*)p->backward);
  free((void*)p->speed);
}

void deleteAllPointer(PinParameter) {
  for (int i = 0; i < motorNum; ++i) {
    deleteMotorPin(PinList + i);
  }

  free(PinList);
}

MotorPin* initializeMotorPins(int motorNum) {
  if (motorNum != 2 && motorNum != 4) {
    fprintf(stderr, "??? unavailable");
    exit(1);
  }

  MotorPin* MotorPinList = (MotorPin*) malloc(sizeof(MotorPin) * motorNum);
  MotorPinList[0] = newMotorPin(4, 6, 5);
  MotorPinList[1] = newMotorPin(7, 9, 12);

  if (motorNum == 4) {
    // MotorPinList[2] = newMotorPin(9, 10);
    // MotorPinList[3] = newMotorPin(11, 12);
  }

  return MotorPinList;
}

void closeMotor(PinParameter) {
  for (int i = 0; i < motorNum; ++i) {
    closePWM(*PinList[i].speed);
  }
}

void testGPIO(PinParameter) {
  fprintf(stderr, "test start\n");

  for (int i = 0; i < motorNum; ++i) {
    checkVaildGPIO(*PinList[i].forward);
    checkVaildGPIO(*PinList[i].backward);
    checkVaildGPIO(*PinList[i].speed);
  }

  fprintf(stderr, "test finished\n");
}

void setMotorsPWMPeriod(PinParameter) {
  fprintf(stderr, "setPWM start\n");

  for (int i = 0; i < motorNum; ++i) {
    setPWMPeriod(*(PinList + i)->speed, PWMPeriod);
    openPWM(*(PinList + i)->speed);
  }
}

void setPinSpeed(int pinId, int duty) {
  setPWMDuty(pinId, duty);
  sleep(1);
  fprintf(stderr, "setPinSpeed finished\n");
}

void setForwardFullSpeed(PinParameter) {
  for (int i = 0; i < motorNum; ++i) {
    setPinSpeed(*PinList[i].speed, PWMPeriod);
  }
}

void setMotorGPIO(PinParameter) {
  for (int i = 0; i < motorNum; ++i) {
    setGPIOOUTPUT(*PinList[i].forward);
    setGPIOOUTPUT(*PinList[i].backward);
  }
}

void initMotors(PinParameter) {
  setMotorsPWMPeriod(PinList, motorNum);
  setMotorGPIO(PinList, motorNum);
  fprintf(stderr, "init Motors finished\n");
}

void setForward(const MotorPin* curMotor) {
  digitalWrite(*curMotor->backward, LOW);
  digitalWrite(*curMotor->forward, HIGH);
} 

void setBackward(const MotorPin* curMotor) {
  digitalWrite(*curMotor->forward, LOW);
  digitalWrite(*curMotor->backward, HIGH);
} 

void setMotorStop(const MotorPin* curMotor) {
  digitalWrite(*curMotor->forward, LOW);
  digitalWrite(*curMotor->backward, LOW);
}

void setAllForward(PinParameter) {
  for (int i = 0; i < motorNum; ++i) {
    setForward(PinList + i);
  }
}

void setAllBackward(PinParameter) {
  for (int i = 0; i < motorNum; ++i) {
    setBackward(PinList + i);
  }
}

void stopAllMotors(PinParameter) {
  for (int i = 0; i < motorNum; ++i) {
    setMotorStop(PinList + i);
  }
}

void moveAction(PinParameter) {
  int testCase = 5;
  while (testCase--) {
    printf("test forward\n");
    setAllForward(PinList, motorNum);

    sleep(5);

    printf("test backward\n");
    setAllBackward(PinList, motorNum);

    sleep(5);
  }

  stopAllMotors(PinList, motorNum);
}

void turnLeft(PinParameter) {
  std::cerr << "turnLeft" << std::endl;
  setMotorStop(&leftMotorPin);
  setForward(&rightMotorPin);
}

void turnRight(PinParameter) {
  std::cerr << "turnRight" << std::endl;
  setMotorStop(&rightMotorPin);
  setForward(&leftMotorPin);
}