#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>

typedef enum {
  DIRECTION_LEFT,
  DIRECTION_RIGHT,
  DIRECTION_NONE
} Direction;

typedef struct {
  const int* trigger;
  const int* echo;
  const double* timeout;
  const double* soundSpeed;
  const double* safeDistance;
  const double* turnTime;
  Direction* lastDirection;
} UltrasonicConfig;


UltrasonicConfig newUltrasonic(int trigger, int echo, double timeout, double soundSpeed, double safeDistance, double turnTime);
void initUltrasonic(const UltrasonicConfig* ultrasonic);
void sendTriggerPulse(const UltrasonicConfig* ultrasonic);
double measurePulseDuration(const UltrasonicConfig* ultrasonic);
double calculateDistance(const UltrasonicConfig* ultrasonic);
double getDuration(struct timeval start, struct timeval current);
void turnLeftWithUltrasonic(UltrasonicConfig* ultrasonic, PinParameter);
void turnRightWithUltrasonic(UltrasonicConfig* ultrasonic, PinParameter);
Direction getRandomDirection(void);
void avoidObstacle(UltrasonicConfig* ultrasonic, PinParameter);
void runAction(UltrasonicConfig* ultrasonic, PinParameter);
void deleteUltrasonic(UltrasonicConfig* ultrasonic);

#endif