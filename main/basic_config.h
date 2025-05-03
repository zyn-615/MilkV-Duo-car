#ifndef BASIC_H
#define BASIC_H

#include <stdbool.h>

#define PinParameter MotorPin* PinList, int motorNum

const long PWMPeriod = 10000;
const long PWMEmpty = 0;

typedef struct {
  const int* forward;
  const int* backward;
  const int* speed;
} MotorPin;

void setupMilkV(void);
void checkVaildGPIO(int pinId);
void openPWM(int pinId);
void closePWM(int pinId);
void setGPIOOUTPUT(int pinId);
void setGPIOINPUT(int pinId);
void setPWMDuty(int pinId, int duty);
void setPWMPeriod(int pinId, int period);
bool toNowTime(struct timeval* _time);
struct timeval getNowTime(void);

#define TWD
#ifdef TWD

#define leftMotorPin PinList[0]
#define rightMotorPin PinList[1]

#endif

#ifdef FWD

#define leftFrontMotorPin MotorPinList[0]
#define leftBackMotorPin MotorPinList[1]
#define rightFrontMotorPin MotorPinList[2]
#define rightBackMotorPin MotorPinList[3]

#endif

#endif