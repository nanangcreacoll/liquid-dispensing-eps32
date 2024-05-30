#include <Arduino.h>
#include <WIFI.hpp>
#include <Mqtt.hpp>
#include <Dispensing.hpp>
#include <ArduinoJson.h>

#include "env.h"

WIFI wifi(WIFI_SSID, WIFI_PASSWORD);
Mqtt mqtt(MQTT_SERVER, MQTT_PORT, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);

byte pinsX[] = {STEPPER_X_STEP, STEPPER_X_DIR, STEPPER_X_ENABLE, STEPPER_X_LIMIT_SWITCH, STEPPER_X_LED};
byte pinsZ[] = {STEPPER_Z_STEP, STEPPER_Z_DIR, STEPPER_Z_ENABLE, STEPPER_Z_LIMIT_SWITCH, STEPPER_Z_LED};
byte pinsZp[] = {STEPPER_ZP_STEP, STEPPER_ZP_DIR, STEPPER_ZP_ENABLE, STEPPER_ZP_LIMIT_SWITCH, STEPPER_ZP_LED};
byte msPins[] = {MS1_PIN, MS2_PIN};

Dispensing dispensing(pinsX, pinsZ, pinsZp, msPins, SOLENOID_PIN);

unsigned long lastCheck = 0;
bool checked = false;

void dispenseStart()
{
  if (dispensing.check(mqtt) && checked)
  {
    JsonDocument doc;
    doc["status"] = dispensing.getDispensingStatus();

    if (mqtt.publish(DISPENSING_STATUS_TOPIC, doc.as<String>().c_str()))
    {
      Serial.println("Published to " + mqtt.getPubTopic() + "!");
      Serial.println(mqtt.getPubMessage());

      if (dispensing.start())
      {
        Serial.println("Dispensing started!");
        doc["status"] = dispensing.getDispensingStatus();
        if (mqtt.publish(DISPENSING_STATUS_TOPIC, doc.as<String>().c_str()))
        {
          Serial.println("Published to " + mqtt.getPubTopic() + "!");
          Serial.println(mqtt.getPubMessage());
        }
        else
        {
          Serial.println("Failed to publish to " + mqtt.getPubTopic() + "!");
        }
      }
      else
      {
        Serial.println("Failed to start dispensing!");
      }      
    }
    else
    {
      Serial.println("Failed to publish to " + mqtt.getPubTopic() + "!");
    }
    checked = false;
  }
  else
  {
    if (millis() - lastCheck >= CHECK_PERIOD)
    {
      Serial.println("Waiting for dispensing check!");
      lastCheck = millis();
    }
    checked = true;
  }
}

void dispenseStartDummy()
{
  if (dispensing.check(mqtt) && checked)
  {
    JsonDocument doc;
    doc["status"] = dispensing.getDispensingStatus();

    if (mqtt.publish(DISPENSING_STATUS_TOPIC, doc.as<String>().c_str()))
    {
      Serial.println("Published to " + mqtt.getPubTopic() + "!");
      Serial.println(mqtt.getPubMessage());

      if (dispensing.startDummy())
      {
        Serial.println("Dispensing started!");
        doc["status"] = dispensing.getDispensingStatus();
        if (mqtt.publish(DISPENSING_STATUS_TOPIC, doc.as<String>().c_str()))
        {
          Serial.println("Published to " + mqtt.getPubTopic() + "!");
          Serial.println(mqtt.getPubMessage());
        }
        else
        {
          Serial.println("Failed to publish to " + mqtt.getPubTopic() + "!");
        }
      }
      else
      {
        Serial.println("Failed to start dispensing!");
      }      
    }
    else
    {
      Serial.println("Failed to publish to " + mqtt.getPubTopic() + "!");
    }
    checked = false;
  }
  else
  {
    if (millis() - lastCheck >= CHECK_PERIOD)
    {
      Serial.println("Waiting for dispensing check!");
      lastCheck = millis();
    }
    checked = true;
  }
}

void setup()
{
  Serial.begin(115200);
  wifi.init();
  mqtt.init();
  if (mqtt.subscribe(DISPENSING_DATA_TOPIC))
  {
    Serial.println("Subscribed to topic " + mqtt.getSubTopic() + "!");
  } 
  else
  {
    Serial.println("Failed to subscribe to topic " + mqtt.getSubTopic() + "!");
  }
  dispensing.init();

  #if defined(DISPENSING) && DISPENSING == 0
    dispensing.dummyHoming();
  #elif defined(DISPENSING) && DISPENSING == 1
    dispensing.homing();
  #elif defined(DISPENSING) && DISPENSING == 2
    Serial.println("Serial calibration started!");
  #else
    Serial.println("Dispensing mode not defined!");
  #endif
}

void loop()
{
  wifi.check();
  mqtt.check();

  #if defined(DISPENSING) && DISPENSING == 0
    dispenseStartDummy();
  #elif defined(DISPENSING) && DISPENSING == 1
    dispenseStart();
  #elif defined(DISPENSING) && DISPENSING == 2
    dispensing.serialCalibration();
  #else
    Serial.println("Dispensing mode not defined!");
  #endif
}