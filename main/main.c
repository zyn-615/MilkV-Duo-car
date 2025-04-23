#include <stdio.h>
#include <unistd.h>

#include <wiringx.h>
#include <stdlib.h>

#define PinParameter MotorPin* PinList, int motorNum

typedef struct {
  const int* forward;
  const int* backward;
  const int* speed;
} MotorPin;

const long PWMPeriod = 10000;
const long PWMEmpty = 0;

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

void testGPIO(PinParameter) {
  fprintf(stderr, "test start\n");

  for (int i = 0; i < motorNum; ++i) {
    if (wiringXValidGPIO(*PinList[i].backward) != 0) {
      fprintf(stderr, "Invalid GPIO %d\n", *PinList[i].backward);
      exit(1);
    }

    if (wiringXValidGPIO(*PinList[i].forward) != 0) {
      fprintf(stderr, "Invalid GPIO %d\n", *PinList[i].forward);
      exit(1);
    }

    if (wiringXValidGPIO(*PinList[i].speed) != 0) {
      fprintf(stderr, "Invalid GPIO %d\n", *PinList[i].speed);
      exit(1);
    }
  }

  fprintf(stderr, "test finished\n");
}

void setMotorPWMPeriod(MotorPin* pin, int period) {
  if (wiringXPWMSetPeriod(*pin->speed, period) != 0) {
    fprintf(stderr, "Pin %d does not support PWM\n", *pin->speed);
    exit(-1);
  }

  wiringXPWMSetPeriod(*pin->speed, period);
}

void setPWMPeriod(PinParameter) {
  fprintf(stderr, "test start\n");

  for (int i = 0; i < motorNum; ++i) {
    setMotorPWMPeriod(PinList + i, PWMPeriod);
  }
}

void setPinSpeed(int pinId, int duty) {
  wiringXPWMSetDuty(pinId, duty);
}

void setForwardFullSpeed(PinParameter) {
  for (int i = 0; i < motorNum; ++i) {
    setPinSpeed(*PinList[i].speed, PWMPeriod);
  }
}

void setMotorGPIO(PinParameter) {
  for (int i = 0; i < motorNum; ++i) {
    pinMode(*PinList[i].backward, PINMODE_OUTPUT);
    pinMode(*PinList[i].forward, PINMODE_OUTPUT);
  }
}

void initMotors(PinParameter) {
  setPWMPeriod(PinList, motorNum);
  setMotorGPIO(PinList, motorNum);
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

#define TWD
#ifdef TWD

#define leftMotorPin MotorPinList[0]
#define rightMotorPin MotorPinList[1]

#endif

#ifdef FWD

#define leftFrontMotorPin MotorPinList[0]
#define leftBackMotorPin MotorPinList[1]
#define rightFrontMotorPin MotorPinList[2]
#define rightBackMotorPin MotorPinList[3]

#endif

int main(void) {
  if (wiringXSetup("milkv_duo", NULL) == -1) {
    fprintf(stderr, "wiringX setup failed\n");
    return -1;
  }

  const int MODE = 2;

  printf("Platform: %s\n", wiringXPlatform());

  MotorPin* MotorPinList = initializeMotorPins(MODE);
  testGPIO(MotorPinList, MODE);
  initMotors(MotorPinList, MODE);
  setForwardFullSpeed(MotorPinList, MODE);

  moveAction(MotorPinList, MODE);


  deleteAllPointer(MotorPinList, MODE);
  wiringXGC();
  return 0;
}