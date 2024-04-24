#ifndef __DISPENSING
#define __DISPENSING

#include <Arduino.h>
#include <AccelStepper.h>

#define MIRCROSTEPS 16
#define STEPS_PER_REV 200
#define MAX_SPEED 1000
#define ACCELERATION 200

class Dispensing
{
private:
    AccelStepper stepperX;
    AccelStepper stepperZ;
    AccelStepper stepperZp;

    byte stepPinX;
    byte dirPinX;
    byte enablePinX;
    byte ms1PinX;
    byte ms2PinX;

    byte stepPinZ;
    byte dirPinZ;
    byte enablePinZ;
    byte ms1PinZ;
    byte ms2PinZ;

    byte stepPinZp;
    byte dirPinZp;
    byte enablePinZp;
    byte ms1PinZp;
    byte ms2PinZp;

    int volume;
    int capsuleQty;
public:
    Dispensing(const byte pinsX[], const byte pinsZ[], const byte pinsZp[]);
    void init();
};

#endif