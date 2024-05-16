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

#define VIAL_X_POS 0
#define VIAL_Z_POS 0

#define CAPSULE_1_X_POS 0
#define CAPSULE_2_X_POS 0
#define CAPSULE_3_X_POS 0
#define CAPSULE_4_X_POS 0
#define CAPSULE_5_X_POS 0

#define CAPSULE_1_Z_POS 0
#define CAPSULE_2_Z_POS 0
#define CAPSULE_3_Z_POS 0
#define CAPSULE_4_Z_POS 0
#define CAPSULE_5_Z_POS 0

#define SYRINGE_MIN_POS 0
#define SYRINGE_MAX_POS 0

#define SYRINGE_MIN_VOLUME 0
#define SYRINGE_MAX_VOLUME 50

#define CAPSULE_MIN_QTY 0
#define CAPSULE_MAX_QTY 5

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

    byte solenoidPin;

    int volume = 0;
    int capsuleQty = 0;
    bool status = true;

    long sryingeFillingSteps = map(volume, SYRINGE_MIN_VOLUME, SYRINGE_MAX_VOLUME, SYRINGE_MIN_POS, SYRINGE_MAX_POS);
    
    bool fillSyringe();
    bool dispenseSyringe();
    bool emptySyringe();
    

public:
    Dispensing(const byte pinsX[], const byte pinsZ[], const byte pinsZp[], const byte msPins[], const byte solenoidPin);
    
    // proccess methods
    void init();
    void homing();
    void dispensing(int &volume, int &capsuleQty);

    // test methods
    void readLimitSwitch();
    void ledTest();
    void solenoidTest();
    void allLedAndSolenoidTest();

    // calibration methods
    void runAndFindPosX(long pos, unsigned long speed);
    void runAndFindPosZ(long pos, unsigned long speed);
    void runAndFindPosZp(long pos, unsigned long speed);
};

#endif