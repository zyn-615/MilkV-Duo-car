#ifndef MOTOR_H
#define MOTOR_H

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

#endif