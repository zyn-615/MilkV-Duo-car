#include <wiringx.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "basic_config.h"

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
  wiringXPWMEnable(pinId, 1);
}

void closePWM(int pinId) {
  wiringXPWMEnable(pinId, 0);
}

void setGPIOOUTPUT(int pinId) {
  pinMode(pinId, PINMODE_OUTPUT);
}

void setGPIOINPUT(int pinId) {
  pinMode(pinId, PINMODE_INPUT);
}

void setPWMPeriod(int pinId, int period) {
  if (wiringXPWMSetPeriod(pinId, period) != 0) {
    fprintf(stderr, "Pin %d does not support PWM\n", pinId);
    exit(-1);
  }

  wiringXPWMSetPeriod(pinId, period);
}

void setPWMDuty(int pinId, int duty) {
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