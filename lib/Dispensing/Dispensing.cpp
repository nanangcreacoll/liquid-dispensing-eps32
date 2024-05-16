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
    Serial.println("LED X ON");
    delay(1000);
    digitalWrite(this->ledPinX, LOW);
    Serial.println("LED X OFF");
    delay(1000);

    digitalWrite(this->ledPinZ, HIGH);
    Serial.println("LED Z ON");
    delay(1000);
    digitalWrite(this->ledPinZ, LOW);
    Serial.println("LED Z OFF");
    delay(1000);

    digitalWrite(this->ledPinZp, HIGH);
    Serial.println("LED Z' ON");
    delay(1000);

    digitalWrite(this->ledPinZp, LOW);
    Serial.println("LED Z' OFF");
    delay(1000);
}

void Dispensing::solenoidTest()
{
    digitalWrite(this->solenoidPin, HIGH);
    Serial.println("Solenoid ON");
    delay(1000);
    digitalWrite(this->solenoidPin, LOW);
    Serial.println("Solenoid OFF");
    delay(1000);
}

void Dispensing::allLedAndSolenoidTest()
{
    digitalWrite(this->ledPinX, HIGH);
    digitalWrite(this->ledPinZ, HIGH);
    digitalWrite(this->ledPinZp, HIGH);
    digitalWrite(this->solenoidPin, HIGH);
    Serial.println("LEDs and Solenoid ON");
    delay(1000);
    digitalWrite(this->ledPinX, LOW);
    digitalWrite(this->ledPinZ, LOW);
    digitalWrite(this->ledPinZp, LOW);
    digitalWrite(this->solenoidPin, LOW);
    Serial.println("LEDs and Solenoid OFF");
    delay(1000);
}

void Dispensing::runAndFindPosX(long pos, unsigned long speed)
{
    stepperX.enableOutputs();
    digitalWrite(this->ledPinX, HIGH);
    stepperX.moveTo(pos);
    stepperX.setSpeed(speed);
    while (stepperX.currentPosition() != pos)
    {
        stepperX.run();
    }
    Serial.println("Stepper X position: " + String(stepperX.currentPosition()));
    stepperX.disableOutputs();
    this->homedX = false;
    digitalWrite(this->ledPinX, LOW);
}

void Dispensing::runAndFindPosZ(long pos, unsigned long speed)
{
    stepperZ.enableOutputs();
    digitalWrite(this->ledPinZ, HIGH);
    stepperZ.moveTo(pos);
    stepperZ.setSpeed(speed);
    while (stepperZ.currentPosition() != pos)
    {
        stepperZ.run();
    }
    Serial.println("Stepper Z position: " + String(stepperZ.currentPosition()));
    stepperZ.disableOutputs();
    this->homedZ = false;
    digitalWrite(this->ledPinZ, LOW);
}

void Dispensing::runAndFindPosZp(long pos, unsigned long speed)
{
    stepperZp.enableOutputs();
    digitalWrite(this->ledPinZp, HIGH);
    stepperZp.moveTo(pos);
    stepperZp.setSpeed(speed);
    while (stepperZp.currentPosition() != pos)
    {
        stepperZp.run();
    }
    Serial.println("Stepper Z' position: " + String(stepperZp.currentPosition()));
    stepperZp.disableOutputs();
    this->homedZp = false;
    digitalWrite(this->ledPinZp, LOW);
}

void Dispensing::dispensing(int &volume, int &capsuleQty)
{
    this->volume = volume;
    this->capsuleQty = capsuleQty;
    if (this->volume > SYRINGE_MIN_VOLUME && this->capsuleQty > CAPSULE_MIN_QTY && this->volume <= SYRINGE_MAX_VOLUME && this->capsuleQty <= CAPSULE_MAX_QTY)
    {
        Serial.println("Dispensing " + String(this->volume) + " uL for " + String(this->capsuleQty) + " capsules ...");   
    }
    else
    {
        Serial.println("Volume or Capsule Quantity is invalid!");
        return;
    }

    if (this->homedX && this->homedZ && this->homedZp)
    {
        for (int i = 1; i <= this->capsuleQty; i++)
        {
            Serial.println("Dispensing capsule " + String(i) + " ...");
            
            if (this->emptySyringe())
            {
                stepperX.enableOutputs();
                digitalWrite(this->ledPinX, HIGH);
                stepperX.moveTo(VIAL_X_POS);
                stepperX.setSpeed(MAX_SPEED);
                while (stepperX.currentPosition() != VIAL_X_POS)
                {
                    stepperX.run();
                }
                stepperX.disableOutputs();
                digitalWrite(this->ledPinX, LOW);
                
                stepperZ.enableOutputs();
                digitalWrite(this->ledPinZ, HIGH);
                stepperZ.moveTo(VIAL_Z_POS);
                stepperZ.setSpeed(MAX_SPEED);
                while (stepperZ.currentPosition() != VIAL_Z_POS)
                {
                    stepperZ.run();
                }
                stepperZ.disableOutputs();
                digitalWrite(this->ledPinZ, LOW);

                if (this->fillSyringe())
                {
                    stepperZ.enableOutputs();
                    digitalWrite(this->ledPinZ, HIGH);
                    stepperZ.moveTo(HOME_POS);
                    stepperZ.setSpeed(MAX_SPEED);
                    while (stepperZ.currentPosition() != HOME_POS)
                    {
                        stepperZ.run();
                    }
                    stepperZ.disableOutputs();
                    digitalWrite(this->ledPinZ, LOW);

                    stepperX.enableOutputs();
                    digitalWrite(this->ledPinX, HIGH);
                    stepperX.moveTo(HOME_POS);
                    stepperX.setSpeed(MAX_SPEED);
                    while (stepperX.currentPosition() != HOME_POS)
                    {
                        stepperX.run();
                    }
                    stepperX.disableOutputs();
                    digitalWrite(this->ledPinX, LOW);

                    if (i == 1)
                    {
                        stepperX.enableOutputs();
                        digitalWrite(this->ledPinX, HIGH);
                        stepperX.moveTo(CAPSULE_1_X_POS);
                        stepperX.setSpeed(MAX_SPEED);
                        while (stepperX.currentPosition() != CAPSULE_1_X_POS)
                        {
                            stepperX.run();
                        }
                        stepperX.disableOutputs();
                        digitalWrite(this->ledPinX, LOW);

                        stepperZ.enableOutputs();
                        digitalWrite(this->ledPinZ, HIGH);
                        stepperZ.moveTo(CAPSULE_1_Z_POS);
                        stepperZ.setSpeed(MAX_SPEED);
                        while (stepperZ.currentPosition() != CAPSULE_1_Z_POS)
                        {
                            stepperZ.run();
                        }
                        stepperZ.disableOutputs();
                        digitalWrite(this->ledPinZ, LOW);

                        if (this->dispenseSyringe())
                        {
                            stepperZ.enableOutputs();
                            digitalWrite(this->ledPinZ, HIGH);
                            stepperZ.moveTo(HOME_POS);
                            stepperZ.setSpeed(MAX_SPEED);
                            while (stepperZ.currentPosition() != HOME_POS)
                            {
                                stepperZ.run();
                            }
                            stepperZ.disableOutputs();
                            digitalWrite(this->ledPinZ, LOW);
                            
                            stepperX.enableOutputs();
                            digitalWrite(this->ledPinX, HIGH);
                            stepperX.moveTo(HOME_POS);
                            stepperX.setSpeed(MAX_SPEED);
                            while (stepperX.currentPosition() != HOME_POS)
                            {
                                stepperX.run();
                            }
                            stepperX.disableOutputs();
                            digitalWrite(this->ledPinX, LOW);
                        }
                        else
                        {
                            Serial.println("Failed to dispense syringe for capsule 1!");
                            return;
                        }
                    }
                    else if (i == 2)
                    {
                        stepperX.enableOutputs();
                        digitalWrite(this->ledPinX, HIGH);
                        stepperX.moveTo(CAPSULE_2_X_POS);
                        stepperX.setSpeed(MAX_SPEED);
                        while (stepperX.currentPosition() != CAPSULE_2_X_POS)
                        {
                            stepperX.run();
                        }
                        stepperX.disableOutputs();
                        digitalWrite(this->ledPinX, LOW);

                        stepperZ.enableOutputs();
                        digitalWrite(this->ledPinZ, HIGH);
                        stepperZ.moveTo(CAPSULE_2_Z_POS);
                        stepperZ.setSpeed(MAX_SPEED);
                        while (stepperZ.currentPosition() != CAPSULE_2_Z_POS)
                        {
                            stepperZ.run();
                        }
                        stepperZ.disableOutputs();
                        digitalWrite(this->ledPinZ, LOW);

                        if (this->dispenseSyringe())
                        {
                            stepperZ.enableOutputs();
                            digitalWrite(this->ledPinZ, HIGH);
                            stepperZ.moveTo(HOME_POS);
                            stepperZ.setSpeed(MAX_SPEED);
                            while (stepperZ.currentPosition() != HOME_POS)
                            {
                                stepperZ.run();
                            }
                            stepperZ.disableOutputs();
                            digitalWrite(this->ledPinZ, LOW);
                            
                            stepperX.enableOutputs();
                            digitalWrite(this->ledPinX, HIGH);
                            stepperX.moveTo(HOME_POS);
                            stepperX.setSpeed(MAX_SPEED);
                            while (stepperX.currentPosition() != HOME_POS)
                            {
                                stepperX.run();
                            }
                            stepperX.disableOutputs();
                            digitalWrite(this->ledPinX, LOW);
                        }
                        else
                        {
                            Serial.println("Failed to dispense syringe for capsule 2!");
                            return;
                        }
                    }
                    else if (i == 3)
                    {
                        stepperX.enableOutputs();
                        digitalWrite(this->ledPinX, HIGH);
                        stepperX.moveTo(CAPSULE_3_X_POS);
                        stepperX.setSpeed(MAX_SPEED);
                        while (stepperX.currentPosition() != CAPSULE_3_X_POS)
                        {
                            stepperX.run();
                        }
                        stepperX.disableOutputs();
                        digitalWrite(this->ledPinX, LOW);

                        stepperZ.enableOutputs();
                        digitalWrite(this->ledPinZ, HIGH);
                        stepperZ.moveTo(CAPSULE_3_Z_POS);
                        stepperZ.setSpeed(MAX_SPEED);
                        while (stepperZ.currentPosition() != CAPSULE_3_Z_POS)
                        {
                            stepperZ.run();
                        }
                        stepperZ.disableOutputs();
                        digitalWrite(this->ledPinZ, LOW);

                        if (this->dispenseSyringe())
                        {
                            stepperZ.enableOutputs();
                            digitalWrite(this->ledPinZ, HIGH);
                            stepperZ.moveTo(HOME_POS);
                            stepperZ.setSpeed(MAX_SPEED);
                            while (stepperZ.currentPosition() != HOME_POS)
                            {
                                stepperZ.run();
                            }
                            stepperZ.disableOutputs();
                            digitalWrite(this->ledPinZ, LOW);
                            
                            stepperX.enableOutputs();
                            digitalWrite(this->ledPinX, HIGH);
                            stepperX.moveTo(HOME_POS);
                            stepperX.setSpeed(MAX_SPEED);
                            while (stepperX.currentPosition() != HOME_POS)
                            {
                                stepperX.run();
                            }
                            stepperX.disableOutputs();
                            digitalWrite(this->ledPinX, LOW);
                        }
                        else
                        {
                            Serial.println("Failed to dispense syringe for capsule 3!");
                            return;
                        }
                    }
                    else if (i == 4)
                    {
                        stepperX.enableOutputs();
                        digitalWrite(this->ledPinX, HIGH);
                        stepperX.moveTo(CAPSULE_4_X_POS);
                        stepperX.setSpeed(MAX_SPEED);
                        while (stepperX.currentPosition() != CAPSULE_4_X_POS)
                        {
                            stepperX.run();
                        }
                        stepperX.disableOutputs();
                        digitalWrite(this->ledPinX, LOW);

                        stepperZ.enableOutputs();
                        digitalWrite(this->ledPinZ, HIGH);
                        stepperZ.moveTo(CAPSULE_4_Z_POS);
                        stepperZ.setSpeed(MAX_SPEED);
                        while (stepperZ.currentPosition() != CAPSULE_4_Z_POS)
                        {
                            stepperZ.run();
                        }
                        stepperZ.disableOutputs();
                        digitalWrite(this->ledPinZ, LOW);

                        if (this->dispenseSyringe())
                        {
                            stepperZ.enableOutputs();
                            digitalWrite(this->ledPinZ, HIGH);
                            stepperZ.moveTo(HOME_POS);
                            stepperZ.setSpeed(MAX_SPEED);
                            while (stepperZ.currentPosition() != HOME_POS)
                            {
                                stepperZ.run();
                            }
                            stepperZ.disableOutputs();
                            digitalWrite(this->ledPinZ, LOW);
                            
                            stepperX.enableOutputs();
                            digitalWrite(this->ledPinX, HIGH);
                            stepperX.moveTo(HOME_POS);
                            stepperX.setSpeed(MAX_SPEED);
                            while (stepperX.currentPosition() != HOME_POS)
                            {
                                stepperX.run();
                            }
                            stepperX.disableOutputs();
                            digitalWrite(this->ledPinX, LOW);
                        }
                        else
                        {
                            Serial.println("Failed to dispense syringe for capsule 4!");
                            return;
                        }
                    }
                    else if (i == 5)
                    {
                        stepperX.enableOutputs();
                        digitalWrite(this->ledPinX, HIGH);
                        stepperX.moveTo(CAPSULE_5_X_POS);
                        stepperX.setSpeed(MAX_SPEED);
                        while (stepperX.currentPosition() != CAPSULE_5_X_POS)
                        {
                            stepperX.run();
                        }
                        stepperX.disableOutputs();
                        digitalWrite(this->ledPinX, LOW);

                        stepperZ.enableOutputs();
                        digitalWrite(this->ledPinZ, HIGH);
                        stepperZ.moveTo(CAPSULE_5_Z_POS);
                        stepperZ.setSpeed(MAX_SPEED);
                        while (stepperZ.currentPosition() != CAPSULE_5_Z_POS)
                        {
                            stepperZ.run();
                        }
                        stepperZ.disableOutputs();
                        digitalWrite(this->ledPinZ, LOW);

                        if (this->dispenseSyringe())
                        {
                            stepperZ.enableOutputs();
                            digitalWrite(this->ledPinZ, HIGH);
                            stepperZ.moveTo(HOME_POS);
                            stepperZ.setSpeed(MAX_SPEED);
                            while (stepperZ.currentPosition() != HOME_POS)
                            {
                                stepperZ.run();
                            }
                            stepperZ.disableOutputs();
                            digitalWrite(this->ledPinZ, LOW);
                            
                            stepperX.enableOutputs();
                            digitalWrite(this->ledPinX, HIGH);
                            stepperX.moveTo(HOME_POS);
                            stepperX.setSpeed(MAX_SPEED);
                            while (stepperX.currentPosition() != HOME_POS)
                            {
                                stepperX.run();
                            }
                            stepperX.disableOutputs();
                            digitalWrite(this->ledPinX, LOW);
                        }
                        else
                        {
                            Serial.println("Failed to dispense syringe for capsule 5!");
                            return;
                        }
                    }
                }
                else
                {
                    Serial.println("Failed to fill syringe!");
                    return;
                }
            }
            else
            {
                Serial.println("Failed to empty syringe!");
                return;
            }
        }
    }
    else
    {
        Serial.println("Stepper motors not homed!");
        return;
    }
    
    Serial.println("Dispensing completed!");
}

bool Dispensing::fillSyringe()
{
    if (this->homedZp && stepperX.currentPosition() == VIAL_X_POS && stepperZ.currentPosition() == VIAL_Z_POS && stepperZp.currentPosition() == HOME_POS)
    {
        Serial.println("Filling syringe ...");
        stepperZp.enableOutputs();
        digitalWrite(this->ledPinZp, HIGH);
        stepperZp.moveTo(-this->sryingeFillingSteps);
        stepperZp.setSpeed(MAX_SPEED);
        while (stepperZp.currentPosition() != this->sryingeFillingSteps)
        {
            stepperZp.run();
        }
        Serial.println("Syringe volume: " + String(volume) + " uL");
        stepperZp.disableOutputs();
        digitalWrite(this->ledPinZp, LOW);
        return true;
    }
    else
    {
        Serial.println("Stepper motors Z' not homed or Stepper motors X and Z not at vial position!");
        return false;
    }
}

bool Dispensing::dispenseSyringe()
{
    if (this->fillSyringe())
    {
        Serial.println("Dispensing syringe ...");
        stepperZp.enableOutputs();
        digitalWrite(this->ledPinZp, HIGH);
        stepperZp.moveTo(SYRINGE_MAX_POS);
        stepperZp.setSpeed(MAX_SPEED);
        while (stepperZp.currentPosition() != SYRINGE_MAX_POS)
        {
            stepperZp.run();
        }
        stepperZp.disableOutputs();
        digitalWrite(this->ledPinZp, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to dispense syringe!");
        return false;
    }
}

bool Dispensing::emptySyringe()
{
    if (this->homedZp && stepperX.currentPosition() == VIAL_X_POS && stepperZ.currentPosition() == VIAL_Z_POS && stepperZp.currentPosition() == HOME_POS)
    {
        Serial.println("Emptying syringe ...");
        stepperZp.enableOutputs();
        digitalWrite(this->ledPinZp, HIGH);
        stepperZp.moveTo(SYRINGE_MAX_POS);
        stepperZp.setSpeed(MAX_SPEED);
        while (stepperZp.currentPosition() != this->sryingeFillingSteps)
        {
            stepperZp.run();
        }
        stepperZp.disableOutputs();
        digitalWrite(this->ledPinZp, LOW);
        return true;
    }
    else
    {
        Serial.println("Stepper motors Z' not homed or Stepper motors X and Z not at vial position!");
        return false;
    }
}