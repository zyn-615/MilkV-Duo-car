#include <stdio.h>
#include <unistd.h>
#include <wiringx.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/time.h>
#include "basic_config.h"
#include "ultrasonic.h"
#include "motors_config.h"
#include <iostream>
#include <cassert>

UltrasonicConfig newUltrasonic(int trigger, int echo, double timeout, double soundSpeed, double safeDisttance, double turnTime) {
  int* _trigger = (int*) malloc(sizeof (int));
  int* _echo = (int*) malloc(sizeof (int));
  double* _timeout = (double*) malloc(sizeof (double));
  double* _soundSpeed = (double*) malloc(sizeof (double));
  double* _safeDisttance = (double*) malloc(sizeof (double));
  double* _turnTime = (double*) malloc(sizeof (double));
  Direction* _direction = (Direction*) malloc(sizeof (Direction));

  if (_trigger == NULL || _echo == NULL || _timeout == NULL || _soundSpeed == NULL || _safeDisttance == NULL || _turnTime == NULL || _direction == NULL) {
    perror("Memory allocation failed\n");
    exit(1);
  }

  *_trigger = trigger;
  *_echo = echo;
  *_timeout = timeout;
  *_soundSpeed = soundSpeed;
  *_safeDisttance = safeDisttance;
  *_turnTime = turnTime;  
  *_direction = DIRECTION_NONE;

  UltrasonicConfig ultrasonic = {_trigger, _echo, _timeout, _soundSpeed, _safeDisttance, _turnTime, _direction};
  perror("Create ultrasonic successfully");
  return ultrasonic;
}

void initUltrasonic(const UltrasonicConfig* ultrasonic) {
  perror("InitUltrasonic begin\n");

  checkVaildGPIO(*ultrasonic->trigger);
  checkVaildGPIO(*ultrasonic->echo);
  setGPIOOUTPUT(*ultrasonic->trigger);
  setGPIOINPUT(*ultrasonic->echo);

  usleep(100);
  srand(time(NULL) ^ clock());
  perror("Initialize ultrasonic successfully\n");
}

void sendTriggerPulse(const UltrasonicConfig* ultrasonic) {
  digitalWrite(*ultrasonic->trigger, LOW);
  usleep(50);

  digitalWrite(*ultrasonic->trigger, HIGH);
  usleep(10);
  digitalWrite(*ultrasonic->trigger, LOW);
}

double getDuration(struct timeval start, struct timeval current) {
  return (current.tv_sec - start.tv_sec) * 1e6 + (current.tv_usec - start.tv_usec);
}

double measurePulseDuration(const UltrasonicConfig* ultrasonic) {
  struct timeval start, current;
  double duration = 0;

  if (!toNowTime(&start)) {
    return -1;
  }

  while (digitalRead(*ultrasonic->echo) == LOW) {
    if (!toNowTime(&current)) {
      return -1;
    }

    duration = getDuration(start, current);
    if (duration > *ultrasonic->timeout) {
      std::cerr << "wait too long" << std::endl;
      return -1;
    }
  }

  if (!toNowTime(&start)) {
    return -1;
  }

  while (digitalRead(*ultrasonic->echo) == HIGH) {
    if (!toNowTime(&current)) {
      return -1;
    }

    duration = getDuration(start, current);
    if (duration > *ultrasonic->timeout) {
      std::cerr << "maintain too long" << std::endl;
      return -1;
    }
  }

  return duration;
}

void turnLeftWithUltrasonic(UltrasonicConfig* ultrasonic, PinParameter) {
  turnLeft(PinList, motorNum);
  std::cerr << "turnLeftWithUltrasonic" << std::endl; 
  usleep(int((*ultrasonic->turnTime) * 1e6));
  stopAllMotors(PinList, motorNum);
  *ultrasonic->lastDirection = DIRECTION_LEFT;
}

void turnRightWithUltrasonic(UltrasonicConfig* ultrasonic, PinParameter) {
  turnRight(PinList, motorNum);
  std::cerr << "turnRightWithUltrasonic" << std::endl; 
  usleep(int((*ultrasonic->turnTime) * 1e6));
  stopAllMotors(PinList, motorNum);
  *ultrasonic->lastDirection = DIRECTION_RIGHT;
}

Direction getRandomDirection(void) {
  return (rand() & 1) ? DIRECTION_LEFT : DIRECTION_RIGHT;
}

void avoidObstacle(UltrasonicConfig* ultrasonic, PinParameter) {
  sendTriggerPulse(ultrasonic);
  double distance = calculateDistance(ultrasonic);

  if (distance < 0) {
    stopAllMotors(PinList, motorNum);
    std::cerr << "measure error" << std::endl;  
  } else if (distance <= *ultrasonic->safeDistance) {
    stopAllMotors(PinList, motorNum);
    usleep(500000);

    if (*ultrasonic->lastDirection == DIRECTION_NONE) {
      *ultrasonic->lastDirection = getRandomDirection();
    }

    if (*ultrasonic->lastDirection == DIRECTION_LEFT) {
      turnLeftWithUltrasonic(ultrasonic, PinList, motorNum);
    } else {
      assert(*ultrasonic->lastDirection != DIRECTION_NONE);
      turnRightWithUltrasonic(ultrasonic, PinList, motorNum);
    }
  } else {
    setAllForward(PinList, motorNum);
    *ultrasonic->lastDirection = DIRECTION_NONE;
  }
}

void runAction(UltrasonicConfig* ultrasonic, PinParameter) {
  int testRound = 100;
  while (testRound--) {
    avoidObstacle(ultrasonic, PinList, motorNum);
  }
}

void deleteUltrasonic(UltrasonicConfig* ultrasonic) {
  free((void*) ultrasonic->echo);
  free((void*) ultrasonic->safeDistance);
  free((void*) ultrasonic->soundSpeed);
  free((void*) ultrasonic->lastDirection);
  free((void*) ultrasonic->timeout);
  free((void*) ultrasonic->trigger);
  free((void*) ultrasonic->turnTime);
  free((void*) ultrasonic);
}