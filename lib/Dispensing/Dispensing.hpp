#ifndef __DISPENSING
#define __DISPENSING

#include <Arduino.h>
#include <AccelStepper.h>
#include <Mqtt.hpp>
#include <ArduinoJson.h>

#define DRIVER_TYPE 0 // 0 for TMC2208, 1 for DRV8825

#define MICROSTEPS 16 // 1/16 microsteps

#define X_OFF_SET 600 // 600 steps
#define Z_OFF_SET 4000 // 4000 steps
#define ZP_OFF_SET 4000 // 4000 steps

#define X_HOME_PULL_BACK 3200 // 3200 steps
#define Z_HOME_PULL_BACK 6400 // 6400 steps
#define ZP_HOME_PULL_BACK 6400 // 6400 steps

#define X_ACCELERATION 5000 // 5000 steps/s^2
#define Z_ACCELERATION 12000 // 8000 steps/s^2
#define ZP_ACCELERATION 12000 // 8000 steps/s^2

#define X_MAX_SPEED 7000 // 7000 steps/s
#define X_HOMING_SPEED 2000 // 4000 steps/s
#define X_SECOND_HOMING_SPEED 1000 // 1000 steps/s

#define Z_MAX_SPEED 15000 // 7000 steps/s
#define Z_HOMING_SPEED 8000 // 8000 steps/s
#define Z_SECOND_HOMING_SPEED 4000 // 4000 steps/s

#define ZP_MAX_SPEED 15000 // 7000 steps/s
#define ZP_HOMING_SPEED 8000 // 8000 steps/s
#define ZP_SECOND_HOMING_SPEED 4000 // 4000 steps/s

#define X_HOME_POS 800 // 800 steps
#define Z_HOME_POS 6400 // 6400 steps
#define ZP_HOME_POS 6400 // 6400 steps

#define X_STEPS_PER_MM 100 // 100 steps/mm
#define Z_STEPS_PER_MM 3200 // 3200 steps/mm
#define ZP_STEPS_PER_MM 3200 // 3200 steps/mm

#define VIAL_X_POS 1600 // stepper X on vial position
#define VIAL_Z_POS 230400 // stepper Z on vial position

#define CAPSULE_1_X_POS 4500 // stepper X on capsule 1 position
#define CAPSULE_2_X_POS 6000 // stepper X on capsule 2 position
#define CAPSULE_3_X_POS 7500 // stepper X on capsule 3 position
#define CAPSULE_4_X_POS 9000 // stepper X on capsule 4 position
#define CAPSULE_5_X_POS 10500 // stepper X on capsule 5 position

#define CAPSULE_Z_POS 118400 // stepper Z on capsule position

#define SYRINGE_MIN_POS 211200 // syringe on empty position or 0 uL volume
#define SYRINGE_MAX_POS 51200 // syringe on full position or 50 uL volume

#define SYRINGE_MIN_VOLUME 0 // 0 uL
#define SYRINGE_MAX_VOLUME 50 // 50 uL

#define CAPSULE_MIN_QTY 0 // 0 capsule
#define CAPSULE_MAX_QTY 5 // 5 capsules

#define DISPENSING_CHECK_PERIOD 1000 // 1 second

#if defined(DRIVER_TYPE) && DRIVER_TYPE == 0
    #define MS1_STATE HIGH // MS1 pin state
    #define MS2_STATE HIGH // MS2 pin state
    #define DIRECTION_INVERT false // invert direction
    #define ENABLE_INVERT true // invert enable
#elif defined(DRIVER_TYPE) && DRIVER_TYPE == 1
    #define MS1_STATE LOW // MS1 pin state
    #define MS2_STATE LOW // MS2 pin state
    #define DIRECTION_INVERT false // invert direction
    #define ENABLE_INVERT true // invert enable
#endif

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

    byte stepPinZ;
    byte dirPinZ;
    byte enablePinZ;
    byte limitSwitchPinZ;
    byte ledPinZ;

    byte stepPinZp;
    byte dirPinZp;
    byte enablePinZp;
    byte limitSwitchPinZp;
    byte ledPinZp;

    byte ms1Pin;
    byte ms2Pin;

    byte solenoidPin;

    int volume = 0;
    int capsuleQty = 0;
    bool status = true;

    long sryingeFillingPosition = map(this->volume, SYRINGE_MIN_VOLUME, SYRINGE_MAX_VOLUME, SYRINGE_MIN_POS, SYRINGE_MAX_POS);
    long capsulePositionX[5] = {CAPSULE_1_X_POS, CAPSULE_2_X_POS, CAPSULE_3_X_POS, CAPSULE_4_X_POS, CAPSULE_5_X_POS}; 
    
    bool fillSyringe();
    bool dispenseSyringe();
    bool emptySyringe();
    
    void homeX();
    void homeZ();
    void homeZp();

    bool runToVialX();
    bool runToVialZ();

    bool runToCapsuleX(int &i);
    bool runToCapsuleZ(int &i);

    bool remainCapsule(int &i);

    void dispensing();
    void dummyDispensing();

    float acceleration;
    float maxSpeed;

    long pos;
    long distance;
    long distanceReverse;

    unsigned long lastCheck = 0;

protected:

    // Calibration methods
    bool runToHomeX();
    bool runToHomeZ();
    bool runToHomeZp();

    void runAndMoveToPosX(long &pos);
    void runAndMoveToPosZ(long &pos);
    void runAndMoveToPosZp(long &pos);

    void runAndMovePosX(long &distance);
    void runAndMovePosZ(long &distance);
    void runAndMovePosZp(long &distance);

    void setMaxSpeedAndAccelerationX(float &maxSpeed, float &acceleration);
    void setMaxSpeedAndAccelerationZ(float &maxSpeed, float &acceleration);
    void setMaxSpeedAndAccelerationZp(float &maxSpeed, float &acceleration);

    void readLimitSwitch();
    void serialCalibrationX();
    void serialCalibrationZ();
    void serialCalibrationZp();

public:
    Dispensing(const byte pinsX[], const byte pinsZ[], const byte pinsZp[], const byte msPins[], const byte solenoidPin);
    
    // proccess methods
    void init();
    void homing();
    void dummyHoming();
    bool check(Mqtt &mqtt);
    bool start();
    bool startDummy();
    bool getDispensingStatus();

    // test methods
    void ledTest();
    void solenoidTest();
    void allLedAndSolenoidTest();

    // calibration methods
    void serialCalibration();

    // Destructor
    ~Dispensing();
};

#endif