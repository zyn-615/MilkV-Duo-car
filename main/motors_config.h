#ifndef MOTOR_H
#define MOTOR_H

const long PWMPeriod = 10000;
const long PWMEmpty = 0;

MotorPin newMotorPin(int forwardId, int backwardId, int speedId);
void deleteMotorPin(MotorPin* p);
void deleteAllPointer(PinParameter);
MotorPin* initializeMotorPins(int motorNum);

void closeMotor(PinParameter);
void testGPIO(PinParameter);
void setMotorsPWMPeriod(PinParameter);
void setPinSpeed(int pinId, int duty);
void setForwardFullSpeed(PinParameter);
void setMotorGPIO(PinParameter);
void initMotors(PinParameter);
void setForward(const MotorPin* curMotor);
void setBackward(const MotorPin* curMotor);
void setMotorStop(const MotorPin* curMotor);
void setAllForward(PinParameter);
void setAllBackward(PinParameter);
void stopAllMotors(PinParameter);
void moveAction(PinParameter);
void turnLeft(PinParameter);
void turnRight(PinParameter);

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