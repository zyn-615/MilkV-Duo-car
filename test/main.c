#include <stdio.h>
#include <unistd.h>
#include <wiringx.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/time.h>

#define PinParameter MotorPin* PinList, int motorNum

#define TWD
#ifdef TWD

#define leftMotorPin PinList[0]
#define rightMotorPin PinList[1]

#endif

const long PWMPeriod = 100000;
const long PWMEmpty = 0;

typedef struct {
  const int* forward;
  const int* backward;
  const int* speed;
} MotorPin;

void setupMilkv(void) {
  if (wiringXSetup("milkv_duo", NULL) == -1) {
    fprintf(stderr, "wiringX setup failed\n");
    wiringXGC();
    exit(1);
  }
}

void checkVaildGPIO(int pinId) {
  if (wiringXValidGPIO(pinId) != 0) {
    fprintf(stderr, "Invalid GPIO %d\n", pinId);
    exit(1);
  }
}

void openPWM(int pinId) {
  printf("open pinId : %d\n", pinId);
  wiringXPWMEnable(pinId, 1);
}

void closePWM(int pinId) {
  // wiringXPWMEnable(pinId, 0);
}

void setGPIOOUTPUT(int pinId) {
  pinMode(pinId, PINMODE_OUTPUT);
  digitalWrite(pinId, LOW);
}

void setGPIOINPUT(int pinId) {
  pinMode(pinId, PINMODE_INPUT);
}

void setPWMPeriod(int pinId, long period) {
  printf("setPWM -> %d %d\n", pinId, period);
  if (wiringXPWMSetPeriod(pinId, period) != 0) {
    fprintf(stderr, "Pin %d does not support PWM\n", pinId);
    exit(-1);
  }

  wiringXPWMSetPeriod(pinId, period);
}

void setPWMDuty(int pinId, long duty) {
  printf("setDuty : %d %ld\n", pinId, duty);
  wiringXPWMSetDuty(pinId, duty);
}

bool toNowTime(struct timeval* _time) {
  if (gettimeofday(_time, NULL) == -1) {
    perror("gettimeofday failed");
    return false;
  }
  return true;
}

struct timeval getNowTime(void) {
  struct timeval _nowTime;
  toNowTime(&_nowTime);
  return _nowTime;
}

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
  MotorPinList[1] = newMotorPin(3, 4, 2);
  MotorPinList[0] = newMotorPin(5, 6, 7);

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

void setForwardLowSpeed(PinParameter) {
  for (int i = 0; i < motorNum; ++i) {
    setPinSpeed(*PinList[i].speed, 1);
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
  int testCase = 1;
  while (testCase--) {
    printf("test forward\n");
    setAllForward(PinList, motorNum);
    printf("test finished\n");
    fflush(stdout);

    sleep(5);

    printf("test backward\n");
    setAllBackward(PinList, motorNum);
    printf("test finished\n");
    fflush(stdout);

    sleep(5);
  }

  stopAllMotors(PinList, motorNum);
}

void turnLeft(PinParameter) {
  // std::cerr << "turnLeft" << std::endl;
  setMotorStop(&leftMotorPin);
  setForward(&rightMotorPin);
}

void turnRight(PinParameter) {
  // std::cerr << "turnRight" << std::endl;
  setMotorStop(&rightMotorPin);
  setForward(&leftMotorPin);
}

int main(void) {
  setupMilkv();
  const int MODE = 2;

  printf("Platform: %s\n", wiringXPlatform());

  MotorPin* MotorPinList = initializeMotorPins(MODE);
  testGPIO(MotorPinList, MODE);
  initMotors(MotorPinList, MODE);
  setForwardLowSpeed(MotorPinList, MODE);

  stopAllMotors(MotorPinList, MODE);
  sleep(1);
  moveAction(MotorPinList, MODE);

  closeMotor(MotorPinList, MODE);
  deleteAllPointer(MotorPinList, MODE);
  wiringXGC();
  return 0;
}