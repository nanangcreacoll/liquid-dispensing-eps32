#include "Dispensing.hpp"

Dispensing::Dispensing(const byte pinsX[], const byte pinsZ[], const byte pinsZp[], const byte msPins[], const byte solenoidPin)
{
    this->stepPinX = pinsX[0];
    this->dirPinX = pinsX[1];
    this->enablePinX = pinsX[2];
    this->limitSwitchPinX = pinsX[3];
    this->ledPinX = pinsX[4];

    this->stepPinZ = pinsZ[0];
    this->dirPinZ = pinsZ[1];
    this->enablePinZ = pinsZ[2];
    this->limitSwitchPinZ = pinsZ[3];
    this->ledPinZ = pinsZ[4];

    this->stepPinZp = pinsZp[0];
    this->dirPinZp = pinsZp[1];
    this->enablePinZp = pinsZp[2];
    this->limitSwitchPinZp = pinsZp[3];
    this->ledPinZp = pinsZp[4];

    this->ms1Pin = msPins[0];
    this->ms2Pin = msPins[1];

    this->solenoidPin = solenoidPin;
    
    stepperX = AccelStepper(AccelStepper::DRIVER, this->stepPinX, this->dirPinX);
    stepperZ = AccelStepper(AccelStepper::DRIVER, this->stepPinZ, this->dirPinZ);
    stepperZp = AccelStepper(AccelStepper::DRIVER, this->stepPinZp, this->dirPinZp);
}

void Dispensing::init()
{
    pinMode(this->limitSwitchPinX, INPUT);
    pinMode(this->limitSwitchPinZ, INPUT);
    pinMode(this->limitSwitchPinZp, INPUT);
    
    pinMode(this->ledPinX, OUTPUT);
    pinMode(this->ledPinZ, OUTPUT);
    pinMode(this->ledPinZp, OUTPUT);

    pinMode(this->ms1Pin, OUTPUT);
    pinMode(this->ms2Pin, OUTPUT);

    pinMode(this->solenoidPin, OUTPUT);

    if (MIRCROSTEPS == 16)
    {
        digitalWrite(this->ms1Pin, HIGH);
        digitalWrite(this->ms2Pin, HIGH);
    }
    else if (MIRCROSTEPS == 8)
    {
        digitalWrite(this->ms1Pin, LOW);
        digitalWrite(this->ms2Pin, LOW);
    }
    else if (MIRCROSTEPS == 4)
    {
        digitalWrite(this->ms1Pin, LOW);
        digitalWrite(this->ms2Pin, HIGH);
    }
    else if (MIRCROSTEPS == 2)
    {
        digitalWrite(this->ms1Pin, HIGH);
        digitalWrite(this->ms2Pin, LOW);
    }
    else
    {
        digitalWrite(this->ms1Pin, HIGH);
        digitalWrite(this->ms2Pin, LOW);
    }

    Serial.println("Initializing stepper motors for dispensing ...");
    Serial.println("Using Microsteps: " + String(MIRCROSTEPS));
    
    stepperX.setEnablePin(this->enablePinX);
    stepperX.setPinsInverted(true, false, true);

    stepperZ.setEnablePin(this->enablePinZ);
    stepperZ.setPinsInverted(false, false, true);
    
    stepperZp.setEnablePin(this->enablePinZp);
    stepperZp.setPinsInverted(false, false, true);

    stepperX.setMaxSpeed(MAX_SPEED);
    stepperX.setAcceleration(ACCELERATION);
    Serial.println("Stepper X max speed: " + String(MAX_SPEED) + " steps/s, acceleration: " + String(ACCELERATION) + " steps/s^2");

    stepperZ.setMaxSpeed(MAX_SPEED);
    stepperZ.setAcceleration(ACCELERATION);
    Serial.println("Stepper Z max speed: " + String(MAX_SPEED) + " steps/s, acceleration: " + String(ACCELERATION) + " steps/s^2");

    stepperZp.setMaxSpeed(MAX_SPEED);
    stepperZp.setAcceleration(ACCELERATION);
    Serial.println("Stepper Z' max speed: " + String(MAX_SPEED) + " steps/s, acceleration: " + String(ACCELERATION) + " steps/s^2");

    stepperX.disableOutputs();
    stepperZ.disableOutputs();
    stepperZp.disableOutputs();
}

void Dispensing::homing()
{
    Serial.println("Homing stepper motors ...");

    stepperX.enableOutputs();
    digitalWrite(this->ledPinX, HIGH);
    stepperX.setSpeed(-HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinX) != HIGH)
    {
        stepperX.runSpeed();
    }
    stepperX.setCurrentPosition(0);
    Serial.println("Stepper X on limit switch");
    stepperX.moveTo(HOME_POS);
    stepperX.setSpeed(HOMING_SPEED);
    while (stepperX.currentPosition() != HOME_POS)
    {
        stepperX.run();
    }
    stepperX.setCurrentPosition(HOME_POS);
    stepperX.disableOutputs();
    digitalWrite(this->ledPinX, LOW);
    this->homedX = true;
    Serial.println("Stepper X homed!");

    stepperZ.enableOutputs();
    digitalWrite(this->ledPinZ, HIGH);
    stepperZ.setSpeed(-HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinZ) != HIGH)
    {
        stepperZ.runSpeed();
    }
    stepperZ.setCurrentPosition(0);
    Serial.println("Stepper Z on limit switch");
    stepperZ.moveTo(HOME_POS);
    stepperZ.setSpeed(HOMING_SPEED);
    while (stepperZ.currentPosition() != HOME_POS)
    {
        stepperZ.run();
    }
    stepperZ.setCurrentPosition(HOME_POS);
    stepperZ.disableOutputs();
    digitalWrite(this->ledPinZ, LOW);
    this->homedZ = true;
    Serial.println("Stepper Z homed!");

    stepperZp.enableOutputs();
    digitalWrite(this->ledPinZp, HIGH);
    stepperZp.setSpeed(-HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinZp) != HIGH)
    {
        stepperZp.runSpeed();
    }
    stepperZp.setCurrentPosition(0);
    Serial.println("Stepper Z' on limit switch");
    stepperZp.moveTo(HOME_POS);
    stepperZp.setSpeed(HOMING_SPEED);
    while (stepperZp.currentPosition() != HOME_POS)
    {
        stepperZp.run();
    }
    stepperZp.setCurrentPosition(HOME_POS);
    stepperZp.disableOutputs();
    digitalWrite(this->ledPinZp, LOW);
    this->homedZp = true;
    Serial.println("Stepper Z' homed!");

    Serial.println("Stepper motors homed!");
}

void Dispensing::readLimitSwitch()
{
    if (digitalRead(this->limitSwitchPinX) == HIGH)
    {
        Serial.println("Stepper X on limit switch");
    }
    if (digitalRead(this->limitSwitchPinZ) == HIGH)
    {
        Serial.println("Stepper Z on limit switch");
    }
    if (digitalRead(this->limitSwitchPinZp) == HIGH)
    {
        Serial.println("Stepper Z' on limit switch");
    }
}

void Dispensing::ledTest()
{
    digitalWrite(this->ledPinX, HIGH);
    delay(1000);
    digitalWrite(this->ledPinX, LOW);
    digitalWrite(this->ledPinZ, HIGH);
    delay(1000);
    digitalWrite(this->ledPinZ, LOW);
    digitalWrite(this->ledPinZp, HIGH);
    delay(1000);
    digitalWrite(this->ledPinZp, LOW);
}

void Dispensing::solenoidTest()
{
    digitalWrite(this->solenoidPin, HIGH);
    Serial.println("Solenoid ON");
    delay(1000);
    digitalWrite(this->solenoidPin, LOW);
    Serial.println("Solenoid OFF");
}