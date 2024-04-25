#include "Dispensing.hpp"

Dispensing::Dispensing(const byte pinsX[], const byte pinsZ[], const byte pinsZp[])
{
    this->stepPinX = pinsX[0];
    this->dirPinX = pinsX[1];
    this->enablePinX = pinsX[2];
    this->ms1PinX = pinsX[3];
    this->ms2PinX = pinsX[4];
    this->limitSwitchPinX = pinsX[5];

    this->stepPinZ = pinsZ[0];
    this->dirPinZ = pinsZ[1];
    this->enablePinZ = pinsZ[2];
    this->ms1PinZ = pinsZ[3];
    this->ms2PinZ = pinsZ[4];
    this->limitSwitchPinZ = pinsZ[5];

    this->stepPinZp = pinsZp[0];
    this->dirPinZp = pinsZp[1];
    this->enablePinZp = pinsZp[2];
    this->ms1PinZp = pinsZp[3];
    this->ms2PinZp = pinsZp[4];
    this->limitSwitchPinZp = pinsZp[5];
    
    stepperX = AccelStepper(AccelStepper::DRIVER, this->stepPinX, this->dirPinX);
    stepperZ = AccelStepper(AccelStepper::DRIVER, this->stepPinZ, this->dirPinZ);
    stepperZp = AccelStepper(AccelStepper::DRIVER, this->stepPinZp, this->dirPinZp);
}

void Dispensing::init()
{
    pinMode(this->ms1PinX, OUTPUT);
    pinMode(this->ms2PinX, OUTPUT);
    pinMode(this->limitSwitchPinX, INPUT);
    pinMode(this->ms1PinZ, OUTPUT);
    pinMode(this->ms2PinZ, OUTPUT);
    pinMode(this->limitSwitchPinZ, INPUT);
    pinMode(this->ms1PinZp, OUTPUT);
    pinMode(this->ms2PinZp, OUTPUT);
    pinMode(this->limitSwitchPinZp, INPUT);

    stepperX.setEnablePin(this->enablePinX);
    stepperX.setPinsInverted(false, false, true);
    stepperX.enableOutputs();
    digitalWrite(this->enablePinX, LOW);
    if (MIRCROSTEPS == 16)
    {
        digitalWrite(this->ms1PinX, HIGH);
        digitalWrite(this->ms2PinX, HIGH);
    }
    else if (MIRCROSTEPS == 8)
    {
        digitalWrite(this->ms1PinX, LOW);
        digitalWrite(this->ms2PinX, LOW);
    }
    else if (MIRCROSTEPS == 4)
    {
        digitalWrite(this->ms1PinX, LOW);
        digitalWrite(this->ms2PinX, HIGH);
    }
    else if (MIRCROSTEPS == 2)
    {
        digitalWrite(this->ms1PinX, HIGH);
        digitalWrite(this->ms2PinX, LOW);
    }
    else
    {
        digitalWrite(this->ms1PinX, HIGH);
        digitalWrite(this->ms2PinX, LOW);
    }

    stepperZ.setEnablePin(this->enablePinZ);
    stepperZ.setPinsInverted(false, false, true);
    stepperZ.enableOutputs();
    if (MIRCROSTEPS == 16)
    {
        digitalWrite(this->ms1PinZ, HIGH);
        digitalWrite(this->ms2PinZ, HIGH);
    }
    else if (MIRCROSTEPS == 8)
    {
        digitalWrite(this->ms1PinZ, LOW);
        digitalWrite(this->ms2PinZ, LOW);
    }
    else if (MIRCROSTEPS == 4)
    {
        digitalWrite(this->ms1PinZ, LOW);
        digitalWrite(this->ms2PinZ, HIGH);
    }
    else if (MIRCROSTEPS == 2)
    {
        digitalWrite(this->ms1PinZ, HIGH);
        digitalWrite(this->ms2PinZ, LOW);
    }
    else
    {
        digitalWrite(this->ms1PinZ, HIGH);
        digitalWrite(this->ms2PinZ, LOW);
    }
    
    stepperZp.setEnablePin(this->enablePinZp);
    stepperZp.setPinsInverted(false, false, true);
    stepperZp.enableOutputs();
    if (MIRCROSTEPS == 16)
    {
        digitalWrite(this->ms1PinZp, HIGH);
        digitalWrite(this->ms2PinZp, HIGH);
    }
    else if (MIRCROSTEPS == 8)
    {
        digitalWrite(this->ms1PinZp, LOW);
        digitalWrite(this->ms2PinZp, LOW);
    }
    else if (MIRCROSTEPS == 4)
    {
        digitalWrite(this->ms1PinZp, LOW);
        digitalWrite(this->ms2PinZp, HIGH);
    }
    else if (MIRCROSTEPS == 2)
    {
        digitalWrite(this->ms1PinZp, HIGH);
        digitalWrite(this->ms2PinZp, LOW);
    }
    else
    {
        digitalWrite(this->ms1PinZp, HIGH);
        digitalWrite(this->ms2PinZp, LOW);
    }

    stepperX.setMaxSpeed(MAX_SPEED);
    stepperX.setAcceleration(ACCELERATION);

    stepperZ.setMaxSpeed(MAX_SPEED);
    stepperZ.setAcceleration(ACCELERATION);

    stepperZp.setMaxSpeed(MAX_SPEED);
    stepperZp.setAcceleration(ACCELERATION);

    this->homing();

    stepperX.disableOutputs();
    stepperZ.disableOutputs();
    stepperZp.disableOutputs();
}

void Dispensing::homing()
{
    stepperX.enableOutputs();
    stepperX.setSpeed(-HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinX) != HIGH)
    {
        stepperX.runSpeed();
    }
    stepperX.setCurrentPosition(0);
    stepperX.moveTo(HOME_POS);
    stepperX.setSpeed(HOMING_SPEED);
    while (stepperX.currentPosition() != HOME_POS)
    {
        stepperX.run();
    }
    stepperX.setCurrentPosition(HOME_POS);
    stepperX.disableOutputs();
    this->homedX = true;

    stepperZ.enableOutputs();
    stepperZ.setSpeed(-HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinZ) != HIGH)
    {
        stepperZ.runSpeed();
    }
    stepperZ.setCurrentPosition(0);
    stepperZ.moveTo(HOME_POS);
    stepperZ.setSpeed(HOMING_SPEED);
    while (stepperZ.currentPosition() != HOME_POS)
    {
        stepperZ.run();
    }
    stepperZ.setCurrentPosition(HOME_POS);
    stepperZ.disableOutputs();
    this->homedZ = true;

    stepperZp.enableOutputs();
    stepperZp.setSpeed(-HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinZp) != HIGH)
    {
        stepperZp.runSpeed();
    }
    stepperZp.setCurrentPosition(0);
    stepperZp.moveTo(HOME_POS);
    stepperZp.setSpeed(HOMING_SPEED);
    while (stepperZp.currentPosition() != HOME_POS)
    {
        stepperZp.run();
    }
    stepperZp.setCurrentPosition(HOME_POS);
    stepperZp.disableOutputs();
    this->homedZp = true;
}