#ifndef BASIC_H
#define BASIC_H

#include <stdbool.h>

#define PinParameter MotorPin* PinList, int motorNum

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

#endif