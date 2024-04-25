#ifndef __DISPENSING
#define __DISPENSING

#include <Arduino.h>
#include <AccelStepper.h>

#define MIRCROSTEPS 16
#define STEPS_PER_REV 200
#define MAX_SPEED 1000
#define ACCELERATION 200
#define HOMING_SPEED 200
#define HOME_POS ((STEPS_PER_REV * MIRCROSTEPS)/4)

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
    byte limitSwitchPinX;
    bool homedX = false; 

    byte stepPinZ;
    byte dirPinZ;
    byte enablePinZ;
    byte ms1PinZ;
    byte ms2PinZ;
    byte limitSwitchPinZ;
    bool homedZ = false;

    byte stepPinZp;
    byte dirPinZp;
    byte enablePinZp;
    byte ms1PinZp;
    byte ms2PinZp;
    byte limitSwitchPinZp;
    bool homedZp = false;

    int volume;
    int capsuleQty;
public:
    Dispensing(const byte pinsX[], const byte pinsZ[], const byte pinsZp[]);
    void init();
    void homing();
};

#endif