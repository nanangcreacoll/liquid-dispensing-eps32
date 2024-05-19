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
    stepperZ.setPinsInverted(true, false, true);
    
    stepperZp.setEnablePin(this->enablePinZp);
    stepperZp.setPinsInverted(true, false, true);

    stepperX.setMaxSpeed(X_MAX_SPEED);
    stepperX.setAcceleration(X_ACCELERATION);
    Serial.println("Stepper X max speed: " + String(stepperX.maxSpeed()) + " steps/s, acceleration: " + String(stepperX.acceleration()) + " steps/s^2");

    stepperZ.setMaxSpeed(Z_MAX_SPEED);
    stepperZ.setAcceleration(Z_ACCELERATION);
    Serial.println("Stepper Z max speed: " + String(stepperZ.maxSpeed()) + " steps/s, acceleration: " + String(stepperZ.acceleration()) + " steps/s^2");

    stepperZp.setMaxSpeed(ZP_MAX_SPEED);
    stepperZp.setAcceleration(ZP_ACCELERATION);
    Serial.println("Stepper Z' max speed: " + String(stepperZp.maxSpeed()) + " steps/s, acceleration: " + String(stepperZp.acceleration()) + " steps/s^2");

    stepperX.disableOutputs();
    stepperZ.disableOutputs();
    stepperZp.disableOutputs();
}

void Dispensing::homing()
{
    Serial.println("Homing stepper motors ...");
    digitalWrite(this->solenoidPin, HIGH);

    this->homeX();

    this->homeZ();

    this->homeZp();
    
    digitalWrite(this->solenoidPin, LOW);
    Serial.println("Stepper motors homed!");
}

void Dispensing::homeX()
{
    stepperX.enableOutputs();
    digitalWrite(this->ledPinX, HIGH);
    stepperX.setSpeed(-X_HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinX) != HIGH)
    {
        stepperX.runSpeed();
    }
    delay(500);
    stepperX.setCurrentPosition(0);
    Serial.println("Stepper X on limit switch");

    stepperX.setSpeed(X_HOMING_SPEED);
    stepperX.moveTo(X_HOME_POS);
    while (stepperX.currentPosition() != X_HOME_POS)
    {
        stepperX.runSpeed();
    }
    stepperX.setCurrentPosition(X_HOME_POS);
    stepperX.disableOutputs();
    digitalWrite(this->ledPinX, LOW);
    Serial.println("Stepper X homed!");
}

void Dispensing::homeZ()
{
    stepperZ.enableOutputs();
    digitalWrite(this->ledPinZ, HIGH);
    stepperZ.setSpeed(-Z_HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinZ) != HIGH)
    {
        stepperZ.runSpeed();
    }
    delay(500);
    stepperZ.setCurrentPosition(0);
    Serial.println("Stepper Z on limit switch");
    
    stepperZ.setSpeed(Z_HOMING_SPEED);
    stepperZ.moveTo(Z_HOME_POS);
    while (stepperZ.currentPosition() != Z_HOME_POS)
    {
        stepperZ.runSpeed();
    }
    stepperZ.setCurrentPosition(Z_HOME_POS);
    stepperZ.disableOutputs();
    digitalWrite(this->ledPinZ, LOW);
    Serial.println("Stepper Z homed!");
}

void Dispensing::homeZp()
{
    stepperZp.enableOutputs();
    digitalWrite(this->ledPinZp, HIGH);
    stepperZp.setSpeed(-ZP_HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinZp) != HIGH)
    {
        stepperZp.runSpeed();
    }
    delay(500);
    stepperZp.setCurrentPosition(0);
    Serial.println("Stepper Z' on limit switch");

    stepperZp.setSpeed(ZP_HOMING_SPEED);
    stepperZp.moveTo(ZP_HOME_POS);
    while (stepperZp.currentPosition() != ZP_HOME_POS)
    {
        stepperZp.runSpeed();
    }
    stepperZp.setCurrentPosition(ZP_HOME_POS);
    stepperZp.disableOutputs();
    digitalWrite(this->ledPinZp, LOW);
    Serial.println("Stepper Z' homed!");
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

void Dispensing::runAndFindPosX(long &pos, unsigned long &speed)
{
    stepperX.enableOutputs();
    digitalWrite(this->ledPinX, HIGH);
    stepperX.setSpeed(speed);
    stepperX.move(pos);
    while (stepperX.currentPosition() != pos)
    {
        stepperX.runSpeed();
    }
    Serial.println("Stepper X position: " + String(stepperX.currentPosition()));
    stepperX.disableOutputs();
    digitalWrite(this->ledPinX, LOW);
}

void Dispensing::runAndFindPosZ(long &pos, unsigned long &speed)
{
    stepperZ.enableOutputs();
    digitalWrite(this->ledPinZ, HIGH);
    stepperZ.setSpeed(speed);
    stepperZ.move(pos);
    while (stepperZ.currentPosition() != pos)
    {
        stepperZ.runSpeed();
    }
    Serial.println("Stepper Z position: " + String(stepperZ.currentPosition()));
    stepperZ.disableOutputs();
    digitalWrite(this->ledPinZ, LOW);
}

void Dispensing::runAndFindPosZp(long &pos, unsigned long &speed)
{
    stepperZp.enableOutputs();
    digitalWrite(this->ledPinZp, HIGH);
    stepperZp.setSpeed(speed);
    stepperZp.move(pos);
    while (stepperZp.currentPosition() != pos)
    {
        stepperZp.runSpeed();
    }
    Serial.println("Stepper Z' position: " + String(stepperZp.currentPosition()));
    stepperZp.disableOutputs();
    digitalWrite(this->ledPinZp, LOW);
}

bool Dispensing::runToHomeX()
{
    if (stepperZ.currentPosition() == Z_HOME_POS)
    {
        stepperX.enableOutputs();
        digitalWrite(this->ledPinX, HIGH);
        stepperX.setSpeed(X_MAX_SPEED);
        stepperX.moveTo(X_HOME_POS);
        while (stepperX.currentPosition() != X_HOME_POS)
        {
            stepperX.runSpeed();
        }
        stepperX.disableOutputs();
        digitalWrite(this->ledPinX, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to run to home X!");
        return false;
    
    }
}

bool Dispensing::runToHomeZ()
{
    stepperZ.enableOutputs();
    digitalWrite(this->ledPinZ, HIGH);
    stepperZ.setSpeed(Z_MAX_SPEED);
    stepperZ.moveTo(Z_HOME_POS);
    while (stepperZ.currentPosition() != Z_HOME_POS)
    {
        stepperZ.runSpeed();
    }
    stepperZ.disableOutputs();
    digitalWrite(this->ledPinZ, LOW);
    return true;
}

bool Dispensing::runToHomeZp()
{
    stepperZp.enableOutputs();
    digitalWrite(this->ledPinZp, HIGH);
    stepperZp.setSpeed(ZP_MAX_SPEED);
    stepperZp.moveTo(ZP_HOME_POS);
    while (stepperZp.currentPosition() != ZP_HOME_POS)
    {
        stepperZp.runSpeed();
    }
    stepperZp.disableOutputs();
    digitalWrite(this->ledPinZp, LOW);
    return true;
}

void Dispensing::serialCalibrationX()
{
    Serial.println("Stepper X calibration ...");
    Serial.println("Enter the position to move: ");
    while (Serial.available() == 0)
    {
    }
    long pos = Serial.parseInt();
    Serial.println("Enter the speed to move: ");
    while (Serial.available() == 0)
    {
    }
    unsigned long speed = Serial.parseInt();
    this->runAndFindPosX(pos, speed);
}

void Dispensing::serialCalibrationZ()
{
    Serial.println("Stepper Z calibration ...");
    Serial.println("Enter the position to move: ");
    while (Serial.available() == 0)
    {
    }
    long pos = Serial.parseInt();
    Serial.println("Enter the speed to move: ");
    while (Serial.available() == 0)
    {
    }
    unsigned long speed = Serial.parseInt();
    this->runAndFindPosZ(pos, speed);
}

void Dispensing::serialCalibrationZp()
{
    Serial.println("Stepper Z' calibration ...");
    Serial.println("Enter the position to move: ");
    while (Serial.available() == 0)
    {
    }
    long pos = Serial.parseInt();
    Serial.println("Enter the speed to move: ");
    while (Serial.available() == 0)
    {
    }
    unsigned long speed = Serial.parseInt();
    this->runAndFindPosZp(pos, speed);
}

bool Dispensing::runToVialX()
{
    if (stepperX.currentPosition() == X_HOME_POS && stepperZ.currentPosition() == Z_HOME_POS)
    {
        stepperX.enableOutputs();
        digitalWrite(this->ledPinX, HIGH);
        stepperX.setSpeed(X_MAX_SPEED);
        stepperX.moveTo(VIAL_X_POS);
        while (stepperX.currentPosition() != VIAL_X_POS)
        {
            stepperX.runSpeed();
        }
        stepperX.disableOutputs();
        digitalWrite(this->ledPinX, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to run to vial X!");
        return false;
    }
}

bool Dispensing::runToVialZ()
{
    if (stepperZ.currentPosition() == Z_HOME_POS && stepperX.currentPosition() == VIAL_X_POS)
    {
        stepperZ.enableOutputs();
        digitalWrite(this->ledPinZ, HIGH);
        stepperZ.setSpeed(Z_MAX_SPEED);
        stepperZ.moveTo(VIAL_Z_POS);
        while (stepperZ.currentPosition() != VIAL_Z_POS)
        {
            stepperZ.runSpeed();
        }
        stepperZ.disableOutputs();
        digitalWrite(this->ledPinZ, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to run to vial Z!");
        return false;
    }
}

bool Dispensing::fillSyringe()
{
    if (stepperX.currentPosition() == VIAL_X_POS && stepperZ.currentPosition() == VIAL_Z_POS)
    {
        Serial.println("Filling syringe ...");
        stepperZp.enableOutputs();
        digitalWrite(this->ledPinZp, HIGH);
        stepperZp.setSpeed(ZP_MAX_SPEED);
        stepperZp.moveTo(this->sryingeFillingPosition);
        while (stepperZp.currentPosition() != this->sryingeFillingPosition)
        {
            stepperZp.runSpeed();
        }
        Serial.println("Syringe volume: " + String(volume) + " uL");
        stepperZp.disableOutputs();
        digitalWrite(this->ledPinZp, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to fill syringe!");
        return false;
    }
}

bool Dispensing::dispenseSyringe()
{
    if (this->fillSyringe() && stepperZp.currentPosition() == this->sryingeFillingPosition)
    {
        Serial.println("Dispensing syringe ...");
        stepperZp.enableOutputs();
        digitalWrite(this->ledPinZp, HIGH);
        stepperZp.setSpeed(ZP_MAX_SPEED);
        stepperZp.moveTo(SYRINGE_MIN_POS);
        while (stepperZp.currentPosition() != SYRINGE_MIN_POS)
        {
            stepperZp.runSpeed();
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
    if (stepperX.currentPosition() == VIAL_X_POS && stepperZp.currentPosition() == ZP_HOME_POS)
    {
        Serial.println("Emptying syringe ...");
        stepperZp.enableOutputs();
        digitalWrite(this->ledPinZp, HIGH);
        stepperZp.setSpeed(ZP_MAX_SPEED);
        stepperZp.moveTo(SYRINGE_MAX_POS);
        while (stepperZp.currentPosition() != SYRINGE_MAX_POS)
        {
            stepperZp.runSpeed();
        }
        stepperZp.disableOutputs();
        digitalWrite(this->ledPinZp, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to empty syringe!");
        return false;
    }
}

bool Dispensing::check(Mqtt &mqtt)
{
    if (mqtt.getSubMessage())
    {
        JsonDocument doc;
        deserializeJson(doc, mqtt.getSubMessage());
        if (doc.containsKey("volume") && doc.containsKey("capsuleQty"))
        {
            String volumeMsg = doc["volume"];
            String capsuleQtyMsg = doc["capsuleQty"];

            this->volume = volumeMsg.toInt();
            this->capsuleQty = capsuleQtyMsg.toInt();
            Serial.println("Volume: " + String(this->volume) + " uL, Capsule Quantity: " + String(this->capsuleQty));
            mqtt.clearSubMessage();
            this->status = false;
            return true;
        }
        else
        {
            Serial.println("Invalid message format!");
            mqtt.clearSubMessage();
            return false;
        }
    }
    else
    {
        return false;
    }
}

bool Dispensing::runToCapsuleX(int &i)
{
    if (stepperX.currentPosition() == VIAL_X_POS && stepperZ.currentPosition() == Z_HOME_POS)    
    {
        stepperX.enableOutputs();
        digitalWrite(this->ledPinX, HIGH);
        stepperX.setSpeed(X_MAX_SPEED);
        stepperX.moveTo(this->capsulePositionX[i]);
        while (stepperX.currentPosition() != this->capsulePositionX[i])
        {
            stepperX.runSpeed();
        }
        stepperX.disableOutputs();
        digitalWrite(this->ledPinX, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to run to capsule X!");
        return false;
    }
}

bool Dispensing::runToCapsuleZ(int &i)
{
    if (stepperX.currentPosition() == this->capsulePositionX[i] && stepperZ.currentPosition() == Z_HOME_POS)
    {
        stepperZ.enableOutputs();
        digitalWrite(this->ledPinZ, HIGH);
        stepperZ.setSpeed(Z_MAX_SPEED);
        stepperZ.moveTo(CAPSULE_Z_POS);
        while (stepperZ.currentPosition() != CAPSULE_Z_POS)
        {
            stepperZ.runSpeed();
        }
        stepperZ.disableOutputs();
        digitalWrite(this->ledPinZ, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to run to capsule Z!");
        return false;
    }
}

bool Dispensing::remainCapsule(int &i)
{
    return (i + 1) - this->capsuleQty != 0;
}

bool Dispensing::getDispensingStatus()
{
    return this->status;
}

bool Dispensing::start()
{
    if (this->volume > SYRINGE_MIN_VOLUME && this->volume <= SYRINGE_MAX_VOLUME && this->capsuleQty > CAPSULE_MIN_QTY && this->capsuleQty <= CAPSULE_MAX_QTY)
    {
        // this->dispensing();
        this->dummyDispensing();
        this->status = true;
        return true;
    }
    else
    {
        Serial.println("Invalid volume or capsule quantity!");
        return false;
    }
}

void Dispensing::dispensing()
{
    Serial.println("Dispensing capsule ...");
    digitalWrite(this->solenoidPin, HIGH);
    if (this->emptySyringe())
    {
        for (int i = 0; i < this->capsuleQty; i++)
        {
            if (this->runToVialX())
            {
                if (this->runToVialZ())
                {
                    if (this->fillSyringe())
                    {
                        if (this->runToHomeZ())
                        {
                            if (this->runToCapsuleX(i))
                            {
                                if (this->runToCapsuleZ(i))
                                {
                                    if (this->dispenseSyringe())
                                    {
                                        Serial.println("Capsule " + String(i+1) + " dispensed!");
                                        if (this->runToHomeZ())
                                        {
                                            if (this->remainCapsule(i))
                                            {
                                                continue;
                                            }
                                            else
                                            {
                                                if (this->runToHomeX())
                                                {
                                                    break;
                                                }
                                                else
                                                {
                                                    Serial.println("Failed to run to home X!");
                                                }
                                            }
                                        }
                                        else
                                        {
                                            Serial.println("Failed to run to home Z!");
                                        }
                                    }
                                    else
                                    {
                                        Serial.println("Failed to dispense capsule " + String(i+1) + "!");
                                    }
                                }
                                else
                                {
                                    Serial.println("Failed to run to capsule " + String(i+1) + " Z!");
                                }
                            }
                            else
                            {
                                Serial.println("Failed to run to capsule " + String(i+1) + " X!");
                            }
                        }
                        else
                        {
                            Serial.println("Failed to run to home Z!");
                        }
                    }
                    else
                    {
                        Serial.println("Failed to fill syringe!");
                    }
                }
                else
                {
                    Serial.println("Failed to run to vial Z!");
                }
            }
            else
            {
                Serial.println("Failed to run to vial X!");
            }
        }
    }
    else
    {
        Serial.println("Failed to empty syringe!");
    }
    digitalWrite(this->solenoidPin, LOW);
    Serial.println("Capsule dispensing completed!");
}

void Dispensing::dummyDispensing()
{
    Serial.println("Dummy dispensing ...");
    digitalWrite(this->solenoidPin, HIGH);
    for (int i = 0; i < this->capsuleQty; i++)
    {
        Serial.println("Capsule " + String(i+1) + " dispensed!");
        delay(1000);
    }
    digitalWrite(this->solenoidPin, LOW);
    Serial.println("Dummy dispensing completed!");
}

void Dispensing::dummyHoming()
{
    Serial.println("Dummy homing ...");
    digitalWrite(this->solenoidPin, HIGH);
    delay(1000);
    digitalWrite(this->solenoidPin, LOW);
    Serial.println("Dummy homing completed!");
}