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

    digitalWrite(this->ms1Pin, MS1_STATE);
    digitalWrite(this->ms2Pin, MS2_STATE);

    Serial.println("Initializing stepper motors for dispensing ...");
    Serial.println("Using Microsteps: " + String(MICROSTEPS));

    stepperX.setEnablePin(this->enablePinX);
    stepperX.setPinsInverted(DIRECTION_INVERT, false, ENABLE_INVERT);

    stepperZ.setEnablePin(this->enablePinZ);
    stepperZ.setPinsInverted(DIRECTION_INVERT, false, ENABLE_INVERT);

    stepperZp.setEnablePin(this->enablePinZp);
    stepperZp.setPinsInverted(DIRECTION_INVERT, false, ENABLE_INVERT);

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
    
    this->homeZ();
    this->homeX();
    this->homeZp();

    digitalWrite(this->solenoidPin, LOW);
    Serial.println("Stepper motors homed!");
    this->status = true;
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
    Serial.println("Stepper X on limit switch");
    stepperX.setCurrentPosition(0);
    delay(1000);

    stepperX.moveTo(X_HOME_PULL_BACK);
    while (stepperX.distanceToGo() != 0 && stepperX.currentPosition() != X_HOME_PULL_BACK)
    {
        stepperX.run();
    }
    delay(1000);

    stepperX.setSpeed(-X_SECOND_HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinX) != HIGH)
    {
        stepperX.runSpeed();
    }
    Serial.println("Stepper X on limit switch");
    stepperX.setCurrentPosition(0);
    delay(1000);

    stepperX.moveTo(X_OFF_SET);
    while (stepperX.distanceToGo() != 0 && stepperX.currentPosition() != X_OFF_SET)
    {
        stepperX.run();
    }
    stepperX.setCurrentPosition(0);
    delay(500);

    stepperX.moveTo(X_HOME_POS);
    while (stepperX.distanceToGo() != 0 && stepperX.currentPosition() != X_HOME_POS)
    {
        stepperX.run();
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
    Serial.println("Stepper Z on limit switch");
    stepperZ.setCurrentPosition(0);
    delay(1000);

    stepperZ.moveTo(Z_HOME_PULL_BACK);
    while (stepperZ.distanceToGo() != 0 && stepperZ.currentPosition() != Z_HOME_POS)
    {
        stepperZ.run();
    }
    delay(1000);
    
    stepperZ.setSpeed(-Z_SECOND_HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinZ) != HIGH)
    {
        stepperZ.runSpeed();
    }
    Serial.println("Stepper Z on limit switch");
    stepperZ.setCurrentPosition(0);
    delay(1000);
    
    stepperZ.moveTo(Z_OFF_SET);
    while (stepperZ.distanceToGo() != 0 && stepperZ.currentPosition() != Z_OFF_SET)
    {
        stepperZ.run();
    }
    stepperZ.setCurrentPosition(0);
    delay(500);

    stepperZ.moveTo(Z_HOME_POS);
    while (stepperZ.distanceToGo() != 0 && stepperZ.currentPosition() != Z_HOME_POS)
    {
        stepperZ.run();
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
    delay(1000);
    Serial.println("Stepper Z' on limit switch");
    stepperZp.setCurrentPosition(0);

    stepperZp.moveTo(ZP_HOME_PULL_BACK);
    while (stepperZp.distanceToGo() != 0 && stepperZp.currentPosition() != ZP_HOME_PULL_BACK)
    {
        stepperZp.run();
    }
    delay(1000);

    stepperZp.setSpeed(-ZP_SECOND_HOMING_SPEED);
    while (digitalRead(this->limitSwitchPinZp) != HIGH)
    {
        stepperZp.runSpeed();
    }
    Serial.println("Stepper Z' on limit switch");
    stepperZp.setCurrentPosition(0);
    delay(1000);

    stepperZp.moveTo(ZP_OFF_SET);
    while (stepperZp.distanceToGo() != 0 && stepperZp.currentPosition() != ZP_OFF_SET)
    {
        stepperZp.run();
    }
    stepperZp.setCurrentPosition(0);
    delay(500);

    stepperZp.moveTo(ZP_HOME_POS);
    while (stepperZp.distanceToGo() != 0 && stepperZp.currentPosition() != ZP_HOME_POS)
    {
        stepperZp.run();
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
    delay(5000);
    digitalWrite(this->solenoidPin, LOW);
    Serial.println("Solenoid OFF");
    delay(5000);
}

void Dispensing::allLedAndSolenoidTest()
{
    digitalWrite(this->ledPinX, HIGH);
    digitalWrite(this->ledPinZ, HIGH);
    digitalWrite(this->ledPinZp, HIGH);
    digitalWrite(this->solenoidPin, HIGH);
    Serial.println("LEDs and Solenoid ON");
    delay(5000);
    digitalWrite(this->ledPinX, LOW);
    digitalWrite(this->ledPinZ, LOW);
    digitalWrite(this->ledPinZp, LOW);
    digitalWrite(this->solenoidPin, LOW);
    Serial.println("LEDs and Solenoid OFF");
    delay(5000);
}

void Dispensing::runAndMoveToPosX(long &pos)
{
    stepperX.enableOutputs();
    digitalWrite(this->ledPinX, HIGH);
    stepperX.moveTo(pos);
    while (stepperX.distanceToGo() != 0 && stepperX.currentPosition() != pos)
    {
        stepperX.run();
    }
    Serial.println("Stepper X position: " + String(stepperX.currentPosition()));
    stepperX.disableOutputs();
    digitalWrite(this->ledPinX, LOW);
}

void Dispensing::runAndMoveToPosZ(long &pos)
{
    stepperZ.enableOutputs();
    digitalWrite(this->ledPinZ, HIGH);
    stepperZ.moveTo(pos);
    while (stepperZ.distanceToGo() != 0 && stepperZ.currentPosition() != pos)
    {
        stepperZ.run();
    }
    Serial.println("Stepper Z position: " + String(stepperZ.currentPosition()));
    stepperZ.disableOutputs();
    digitalWrite(this->ledPinZ, LOW);
}

void Dispensing::runAndMoveToPosZp(long &pos)
{
    stepperZp.enableOutputs();
    digitalWrite(this->ledPinZp, HIGH);
    stepperZp.moveTo(pos);
    while (stepperZp.distanceToGo() != 0 && stepperZp.currentPosition() != pos)
    {
        stepperZp.run();
    }
    Serial.println("Stepper Z' position: " + String(stepperZp.currentPosition()));
    stepperZp.disableOutputs();
    digitalWrite(this->ledPinZp, LOW);
}

void Dispensing::runAndMovePosX(long &distance)
{
    stepperX.enableOutputs();
    digitalWrite(this->ledPinX, HIGH);
    stepperX.move(distance);
    while (stepperX.distanceToGo() != 0)
    {
        stepperX.run();
    }
    Serial.println("Stepper X position: " + String(stepperX.currentPosition()));
    stepperX.disableOutputs();
    digitalWrite(this->ledPinX, LOW);
}

void Dispensing::runAndMovePosZ(long &distance)
{
    stepperZ.enableOutputs();
    digitalWrite(this->ledPinZ, HIGH);
    stepperZ.move(distance);
    while (stepperZ.distanceToGo() != 0)
    {
        stepperZ.run();
    }
    Serial.println("Stepper Z position: " + String(stepperZ.currentPosition()));
    stepperZ.disableOutputs();
    digitalWrite(this->ledPinZ, LOW);
}

void Dispensing::runAndMovePosZp(long &distance)
{
    stepperZp.enableOutputs();
    digitalWrite(this->ledPinZp, HIGH);
    stepperZp.move(distance);
    while (stepperZp.distanceToGo() != 0)
    {
        stepperZp.run();
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
        stepperX.moveTo(X_HOME_POS);
        while (stepperX.distanceToGo() != 0 && stepperX.currentPosition() != X_HOME_POS)
        {
            stepperX.run();
        }
        stepperX.disableOutputs();
        digitalWrite(this->ledPinX, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to run to home X with Z position not at home!");
        return false;
    }
}

bool Dispensing::runToHomeZ()
{
    stepperZ.enableOutputs();
    digitalWrite(this->ledPinZ, HIGH);
    stepperZ.moveTo(Z_HOME_POS);
    while (stepperZ.distanceToGo() != 0 && stepperZ.currentPosition() != Z_HOME_POS)
    {
        stepperZ.run();
    }
    stepperZ.disableOutputs();
    digitalWrite(this->ledPinZ, LOW);
    return true;
}

bool Dispensing::runToHomeZp()
{
    stepperZp.enableOutputs();
    digitalWrite(this->ledPinZp, HIGH);
    stepperZp.moveTo(ZP_HOME_POS);
    while (stepperZp.distanceToGo() != 0 && stepperZp.currentPosition() != ZP_HOME_POS)
    {
        stepperZp.run();
    }
    stepperZp.disableOutputs();
    digitalWrite(this->ledPinZp, LOW);
    return true;
}

void Dispensing::setMaxSpeedAndAccelerationX(float &maxSpeed, float &acceleration)
{
    stepperX.setMaxSpeed(maxSpeed);
    stepperX.setAcceleration(acceleration);
}

void Dispensing::setMaxSpeedAndAccelerationZ(float &maxSpeed, float &acceleration)
{
    stepperZ.setMaxSpeed(maxSpeed);
    stepperZ.setAcceleration(acceleration);
}

void Dispensing::setMaxSpeedAndAccelerationZp(float &maxSpeed, float &acceleration)
{
    stepperZp.setMaxSpeed(maxSpeed);
    stepperZp.setAcceleration(acceleration);
}

void Dispensing::serialCalibrationX()
{
    Serial.println("\nStepper X calibration.");
    Serial.println("\t1. Set max speed and acceleration");
    Serial.println("\t2. Move to home position");
    Serial.println("\t3. Move to position");
    Serial.println("\t4. Move distance");
    Serial.println("\t5. Move distance reverse");
    Serial.println("Enter the option: ");
    while (Serial.available() == 0)
    {
    }
    int option = Serial.readStringUntil('\r\n').toInt();
    switch (option)
    {
    case 1:
        Serial.println("Enter max speed: ");
        while (Serial.available() == 0)
        {
        }
        this->maxSpeed = Serial.parseFloat();
        Serial.println("Enter acceleration: ");
        while (Serial.available() == 0)
        {
        }
        this->acceleration = Serial.parseFloat();
        this->setMaxSpeedAndAccelerationX(maxSpeed, acceleration);
        Serial.println("Stepper X max speed: " + String(stepperX.maxSpeed()) + " steps/s, acceleration: " + String(stepperX.acceleration()) + " steps/s^2");
        break;
    case 2:
        this->runToHomeX();
        break;
    case 3:
        Serial.println("Enter the position to move: ");
        while (Serial.available() == 0)
        {
        }
        this->pos = Serial.parseInt();
        this->runAndMoveToPosX(pos);
        break;
    case 4:
        Serial.println("Enter the distance to move: ");
        while (Serial.available() == 0)
        {
        }
        this->distance = Serial.parseInt();
        this->runAndMovePosX(distance);
        break;
    case 5:
        Serial.println("Enter the distance to move reverse: ");
        while (Serial.available() == 0)
        {
        }
        this->distanceReverse = -1 * Serial.parseInt();
        this->runAndMovePosX(distanceReverse);
        break;
    default:
        Serial.println("Invalid option!");
        break;
    }
}

void Dispensing::serialCalibrationZ()
{
    Serial.println("\nStepper Z calibration.");
    Serial.println("\t1. Set max speed and acceleration");
    Serial.println("\t2. Move to home position");
    Serial.println("\t3. Move to position");
    Serial.println("\t4. Move distance");
    Serial.println("\t5. Move distance reverse");
    Serial.println("Enter the option: ");
    while (Serial.available() == 0)
    {
    }
    int option = Serial.readStringUntil('\r\n').toInt();
    switch (option)
    {
    case 1:
        Serial.println("Enter max speed: ");
        while (Serial.available() == 0)
        {
        }
        this->maxSpeed = Serial.parseFloat();
        Serial.println("Enter acceleration: ");
        while (Serial.available() == 0)
        {
        }
        this->acceleration = Serial.parseFloat();
        this->setMaxSpeedAndAccelerationZ(maxSpeed, acceleration);
        Serial.println("Stepper Z max speed: " + String(stepperZ.maxSpeed()) + " steps/s, acceleration: " + String(stepperZ.acceleration()) + " steps/s^2");
        break;
    case 2:
        this->runToHomeZ();
        break;
    case 3:
        Serial.println("Enter the position to move: ");
        while (Serial.available() == 0)
        {
        }
        this->pos = Serial.parseInt();
        this->runAndMoveToPosZ(pos);
        break;
    case 4:
        Serial.println("Enter the distance to move: ");
        while (Serial.available() == 0)
        {
        }
        this->distance = Serial.parseInt();
        this->runAndMovePosZ(distance);
        break;
    case 5:
        Serial.println("Enter the distance to move reverse: ");
        while (Serial.available() == 0)
        {
        }
        this->distanceReverse = -1 * Serial.parseInt();
        this->runAndMovePosZ(distanceReverse);
        break;
    default:
        Serial.println("Invalid option!");
        break;
    }
}

void Dispensing::serialCalibrationZp()
{
    Serial.println("\nStepper Z' calibration.");
    Serial.println("\t1. Set max speed and acceleration");
    Serial.println("\t2. Move to home position");
    Serial.println("\t3. Move to position");
    Serial.println("\t4. Move distance");
    Serial.println("\t5. Move distance reverse");
    Serial.println("Enter the option: ");
    while (Serial.available() == 0)
    {
    }
    int option = Serial.readStringUntil('\r\n').toInt();
    switch (option)
    {
    case 1:
        Serial.println("Enter max speed: ");
        while (Serial.available() == 0)
        {
        }
        this->maxSpeed = Serial.parseFloat();
        Serial.println("Enter acceleration: ");
        while (Serial.available() == 0)
        {
        }
        this->acceleration = Serial.parseFloat();
        this->setMaxSpeedAndAccelerationZp(maxSpeed, acceleration);
        Serial.println("Stepper Z' max speed: " + String(stepperZp.maxSpeed()) + " steps/s, acceleration: " + String(stepperZp.acceleration()) + " steps/s^2");
        break;
    case 2:
        this->runToHomeZp();
        break;
    case 3:
        Serial.println("Enter the position to move: ");
        while (Serial.available() == 0)
        {
        }
        this->pos = Serial.parseInt();
        this->runAndMoveToPosZp(pos);
        break;
    case 4:
        Serial.println("Enter the distance to move: ");
        while (Serial.available() == 0)
        {
        }
        this->distance = Serial.parseInt();
        this->runAndMovePosZp(distance);
        break;
    case 5:
        Serial.println("Enter the distance to move reverse: ");
        while (Serial.available() == 0)
        {
        }
        this->distanceReverse = -1 * Serial.parseInt();
        this->runAndMovePosZp(distanceReverse);
        break;
    default:
        Serial.println("Invalid option!");
        break;
    }
}

void Dispensing::serialCalibration()
{
    Serial.println("\nSerial calibration.");
    Serial.println("\t1. Stepper X");
    Serial.println("\t2. Stepper Z");
    Serial.println("\t3. Stepper Z'");
    Serial.println("\t4. Led test");
    Serial.println("\t5. Solenoid test");
    Serial.println("\t6. All led and solenoid test");
    Serial.println("\t7. Read limit switch");
    Serial.println("\t8. Homing all steppers");
    Serial.println("\t9. Reset");
    Serial.println("Enter the option to calibrate: ");
    while (Serial.available() == 0)
    {
    }
    int option = Serial.readStringUntil('\r\n').toInt();
    switch (option)
    {
    case 1:
        this->serialCalibrationX();
        break;
    case 2:
        this->serialCalibrationZ();
        break;
    case 3:
        this->serialCalibrationZp();
        break;
    case 4:
        while (true)
        {
            char quit = Serial.read();
            this->ledTest();
            if (quit == 'q')
            {
                Serial.println("Quit led test!");
                break;
            }
        }
        break;
    case 5:
        while (true)
        {
            char quit = Serial.read();
            this->solenoidTest();
            if (quit == 'q')
            {
                Serial.println("Quit solenoid test!");
                break;
            }
        }
        break;
    case 6:
        while (true)
        {
            char quit = Serial.read();
            this->allLedAndSolenoidTest();
            if (quit == 'q')
            {
                Serial.println("Quit all led and solenoid test!");
                break;
            }
        }
        break;
    case 7:
        while (true)
        {
            char quit = Serial.read();
            this->readLimitSwitch();
            if (quit == 'q')
            {
                Serial.println("Quit read limit switch!");
                break;
            }
        }
        break;
    case 8:
        this->homing();
        break;
    case 9:
        Serial.println("Resetting in 3 seconds ...");
        delay(3000);
        ESP.restart();
        break;
    default:
        Serial.println("Invalid option!");
        break;
    }
}

bool Dispensing::runToVialX()
{
    if (stepperZ.currentPosition() == Z_HOME_POS)
    {
        stepperX.enableOutputs();
        digitalWrite(this->ledPinX, HIGH);
        stepperX.moveTo(VIAL_X_POS);
        while (stepperX.distanceToGo() != 0 && stepperX.currentPosition() != VIAL_X_POS)
        {
            stepperX.run();
        }
        stepperX.disableOutputs();
        digitalWrite(this->ledPinX, LOW);
        return true;
    }
    else if (stepperZ.currentPosition() == VIAL_Z_POS && stepperX.currentPosition() == VIAL_X_POS)
    {
        Serial.println("Already at vial X!");
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
        stepperZ.moveTo(VIAL_Z_POS);
        while (stepperZ.distanceToGo() != 0 && stepperZ.currentPosition() != VIAL_Z_POS)
        {
            stepperZ.run();
        }
        stepperZ.disableOutputs();
        digitalWrite(this->ledPinZ, LOW);
        return true;
    }
    else if (stepperZ.currentPosition() == VIAL_Z_POS && stepperX.currentPosition() == VIAL_X_POS)
    {
        Serial.println("Already at vial Z!");
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
        stepperZp.move(-this->syringeMove);
        while (stepperZp.distanceToGo() != 0)
        {
            stepperZp.run();
        }
        Serial.println("Syringe volume: " + String(this->volume) + " uL");
        stepperZp.disableOutputs();
        digitalWrite(this->ledPinZp, LOW);
        delay(500);
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
    Serial.println("Dispensing syringe ...");
    stepperZp.enableOutputs();
    digitalWrite(this->ledPinZp, HIGH);
    stepperZp.moveTo(SYRINGE_MIN_POS);
    while (stepperZp.distanceToGo() != 0 && stepperZp.currentPosition() != SYRINGE_MIN_POS)
    {
        stepperZp.run();
    }
    stepperZp.disableOutputs();
    digitalWrite(this->ledPinZp, LOW);
    delay(500);

    #if defined(DISPENSE_TYPE) && DISPENSE_TYPE == 1
        Serial.println("Vibrate ...");
        stepperX.setAcceleration(X_MAX_SPEED);
        stepperX.enableOutputs();
        digitalWrite(this->ledPinX, HIGH);
        stepperX.move(-200);
        while (stepperX.distanceToGo() != 0)
        {
            stepperX.run();
        }
        stepperX.move(200);
        while (stepperX.distanceToGo() != 0)
        {
            stepperX.run();
        }
        stepperX.disableOutputs();
        stepperX.setAcceleration(X_ACCELERATION);
        digitalWrite(this->ledPinX, LOW);
    #endif

    return true;
}

bool Dispensing::emptySyringe()
{
    if (stepperX.currentPosition() == VIAL_X_POS && stepperZp.currentPosition() == ZP_HOME_POS)
    {
        Serial.println("Emptying syringe ...");
        stepperZp.enableOutputs();
        digitalWrite(this->ledPinZp, HIGH);
        stepperZp.moveTo(SYRINGE_MIN_POS);
        while (stepperZp.distanceToGo() != 0 && stepperZp.currentPosition() != SYRINGE_MIN_POS)
        {
            stepperZp.run();
        }
        stepperZp.disableOutputs();
        digitalWrite(this->ledPinZp, LOW);
        return true;
    }
    else if (stepperZp.currentPosition() == SYRINGE_MIN_POS)
    {
        Serial.println("Already at syringe minimum position!");
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
            this->syringeMove = map(this->volume, SYRINGE_MIN_VOLUME, SYRINGE_MAX_VOLUME, SYRINGE_MIN_STEPS, SYRINGE_MAX_STEPS + FILLING_OFFSET);
            
            Serial.println("Volume: " + String(this->volume) + " uL, Capsule Quantity: " + String(this->capsuleQty));
            mqtt.clearSubMessage();
            this->status = false;
            return true;
        }
    }
    mqtt.clearSubMessage();
    return false;
}

bool Dispensing::runToCapsuleX(int &i)
{
    if (stepperX.currentPosition() == VIAL_X_POS && stepperZ.currentPosition() == Z_HOME_POS)
    {
        stepperX.enableOutputs();
        digitalWrite(this->ledPinX, HIGH);
        stepperX.moveTo(this->capsulePositionX[i]);
        while (stepperX.distanceToGo() != 0 && stepperX.currentPosition() != this->capsulePositionX[i])
        {
            stepperX.run();
        }
        stepperX.disableOutputs();
        digitalWrite(this->ledPinX, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to run to capsule " + String(i + 1) + " X!");
        return false;
    }
}

bool Dispensing::runToCapsuleZ(int &i)
{
    if (stepperX.currentPosition() == this->capsulePositionX[i] && stepperZ.currentPosition() == Z_HOME_POS)
    {
        stepperZ.enableOutputs();
        digitalWrite(this->ledPinZ, HIGH);
        stepperZ.moveTo(CAPSULE_Z_POS);
        while (stepperZ.distanceToGo() != 0 && stepperZ.currentPosition() != CAPSULE_Z_POS)
        {
            stepperZ.run();
        }
        stepperZ.disableOutputs();
        digitalWrite(this->ledPinZ, LOW);
        return true;
    }
    else
    {
        Serial.println("Failed to run to capsule " + String(i + 1) + " Z!");
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
        this->dispensing();
        this->status = true;
        return true;
    }
    else
    {
        Serial.println("Invalid volume or capsule quantity!");
        return false;
    }
}

bool Dispensing::startDummy()
{
    if (this->volume > SYRINGE_MIN_VOLUME && this->volume <= SYRINGE_MAX_VOLUME && this->capsuleQty > CAPSULE_MIN_QTY && this->capsuleQty <= CAPSULE_MAX_QTY)
    {
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
    if (this->runToVialX() && this->emptySyringe() && this->runToVialZ())
    {
        for (int i = 0; i < this->capsuleQty; i++)
        {
            if (this->runToVialX() && this->runToVialZ() && this->fillSyringe() && this->runToHomeZ() && this->runToCapsuleX(i) && this->runToCapsuleZ(i) && this->dispenseSyringe())
            {
                Serial.println("Capsule " + String(i + 1) + " dispensed!");
                
                // if (this->runToHomeZ())
                // {
                //     if (this->remainCapsule(i))
                //     {
                //         continue;
                //     }
                //     else
                //     {
                //         if (this->runToHomeX() && this->runToHomeZp())
                //         {
                //             this->homing();
                //             break;
                //         }
                //         else
                //         {
                //             Serial.println("Failed to run to home X or Z'!");
                //             this->homing();
                //         }
                //     }
                // }
                // else
                // {
                //     Serial.println("Failed to run to home Z!");
                //     this->homing();
                // }

                if (this->remainCapsule(i))
                {
                    if (this->runToHomeZ())
                    {
                        continue;
                    }
                    else
                    {
                        Serial.println("Failed to run to home Z!");
                        this->homing();
                        break;
                    }
                }
                else
                {
                    this->homing();
                    break;
                }
            }
            else
            {
                Serial.println("Failed to dispense capsule " + String(i + 1) + "!");
                this->homing();
            }
        }
    }
    else
    {
        Serial.println("Failed to empty syringe!");
        this->homing();
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
        Serial.println("Capsule " + String(i + 1) + " dispensed!");
        delay(1000);
    }
    this->dummyHoming();
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
    this->status = true;
}

Dispensing::~Dispensing() {}