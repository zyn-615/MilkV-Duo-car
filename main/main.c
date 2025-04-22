#include <stdio.h>
#include <unistd.h>

#include <wiringx.h>

#define PinParameter MotorPin* PinList, int motorNum

typedef struct {
  const int* forward;
  const int* backward;
} MotorPin;

const long PWMPeriod = 100000;
const long PWMEmpty = 0;

MotorPin newMotorPin(int forwardId, int backwardId) {
  int* fId = (int*) malloc(sizeof(int));
  int* bId = (int*) malloc(sizeof(int));

  if (fId == NULL || bId == NULL) {
    fprintf(stderr, "Memory allocation failed\n");
    exit(1);
  }

  *fId = forwardId;
  *bId = backwardId;

  MotorPin result = {fId, bId};
  return result;
}

void deleteMotorPin(MotorPin* p) {
  free((void*)p->forward);
  free((void*)p->backward);
}

MotorPin* initializeMotorPins(int motorNum) {
  if (motorNum != 2 || motorNum != 4) {
    fprintf(stderr, "??? unavailable");
  }

  MotorPin* MotorPinList = (MotorPin*) malloc(sizeof(MotorPin) * motorNum);
  MotorPinList[0] = newMotorPin(4, 5);
  MotorPinList[1] = newMotorPin(6, 7);

  if (motorNum == 4) {
    MotorPinList[2] = newMotorPin(9, 10);
    MotorPinList[3] = newMotorPin(11, 12);
  }
}

void testGPIO(PinParameter) {
  fprintf(stderr, "test start\n");

  for (int i = 0; i < motorNum; ++i) {
    if (wiringXValidGPIO(*PinList[i].backward) != 0) {
      fprintf(stderr, "Invalid GPIO %d\n", *PinList[i].backward);
    }

    if (wiringXValidGPIO(*PinList[i].forward) != 0) {
      fprintf(stderr, "Invalid GPIO %d\n", *PinList[i].forward);
    }
  }

  fprintf(stderr, "test finished\n");
}

void setMotorPWMPeriod(MotorPin* pin, int period) {
  wiringXPWMSetPeriod(*pin->forward, period);
  wiringXPWMSetPeriod(*pin->backward, period);
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
    setPinSpeed(PinList[i].forward, PWMPeriod);
    setPinSpeed(PinList[i].backward, PWMEmpty);
  }
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
    wiringXGC();
    return -1;
  }

  const int MODE = 2;
  printf("Platform: %s\n", wiringXPlatform());

  auto MotorPinList = initializeMotorPins(MODE);
  testGPIO(MotorPinList, MODE);
  setPWMPeriod(MotorPinList, MODE);
  setForwardFullSpeed(MotorPinList, MODE);

  for (int i = 0; i < MODE; ++i) {
    deleteMotorPin(MotorPinList[i]);
  }

  return 0;
}