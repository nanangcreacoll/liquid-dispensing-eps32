#ifndef __DISPENSING
#define __DISPENSING

#include <Arduino.h>
#include <AccelStepper.h>

#define MIRCROSTEPS 16
#define STEPS_PER_REV 200
#define MAX_SPEED 1000
#define ACCELERATION 200
#define HOMING_SPEED 1000
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
    byte limitSwitchPinX;
    byte ledPinX;
    bool homedX = false; 

    byte stepPinZ;
    byte dirPinZ;
    byte enablePinZ;
    byte limitSwitchPinZ;
    byte ledPinZ;
    bool homedZ = false;

    byte stepPinZp;
    byte dirPinZp;
    byte enablePinZp;
    byte limitSwitchPinZp;
    byte ledPinZp;
    bool homedZp = false;

    byte ms1Pin;
    byte ms2Pin;

    int volume = 0;
    int capsuleQty = 0;
public:
    Dispensing(const byte pinsX[], const byte pinsZ[], const byte pinsZp[], const byte msPins[]);
    void init();
    void homing();
    bool readLimitSwitch(byte limitSwitchPin);
};

#endif