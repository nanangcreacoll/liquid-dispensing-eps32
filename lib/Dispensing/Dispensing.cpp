#include "Dispensing.hpp"

Dispensing::Dispensing(const byte pinsX[], const byte pinsZ[], const byte pinsZp[])
{
    this->stepPinX = pinsX[0];
    this->dirPinX = pinsX[1];
    this->enablePinX = pinsX[2];
    this->ms1PinX = pinsX[3];
    this->ms2PinX = pinsX[4];

    this->stepPinZ = pinsZ[0];
    this->dirPinZ = pinsZ[1];
    this->enablePinZ = pinsZ[2];
    this->ms1PinZ = pinsZ[3];
    this->ms2PinZ = pinsZ[4];

    this->stepPinZp = pinsZp[0];
    this->dirPinZp = pinsZp[1];
    this->enablePinZp = pinsZp[2];
    this->ms1PinZp = pinsZp[3];
    this->ms2PinZp = pinsZp[4];
    
    stepperX = AccelStepper(AccelStepper::DRIVER, this->stepPinX, this->dirPinX);
    stepperZ = AccelStepper(AccelStepper::DRIVER, this->stepPinZ, this->dirPinZ);
    stepperZp = AccelStepper(AccelStepper::DRIVER, this->stepPinZp, this->dirPinZp);
}

void Dispensing::init()
{
    pinMode(this->enablePinX, OUTPUT);
    pinMode(this->ms1PinX, OUTPUT);
    pinMode(this->ms2PinX, OUTPUT);
    pinMode(this->enablePinZ, OUTPUT);
    pinMode(this->ms1PinZ, OUTPUT);
    pinMode(this->ms2PinZ, OUTPUT);
    pinMode(this->enablePinZp, OUTPUT);
    pinMode(this->ms1PinZp, OUTPUT);
    pinMode(this->ms2PinZp, OUTPUT);

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

    digitalWrite(this->enablePinZ, LOW);
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
    
    digitalWrite(this->enablePinZp, LOW);
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
}