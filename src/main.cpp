#include <Arduino.h>
#include <WIFI.hpp>
#include <Mqtt.hpp>
#include <Dispensing.hpp>

#include "env.h"

WIFI wifi(WIFI_SSID, WIFI_PASSWORD);
Mqtt mqtt(MQTT_SERVER, MQTT_PORT, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);

byte pinsX[] = {STEPPER_X_STEP, STEPPER_X_DIR, STEPPER_X_ENABLE, STEPPER_X_LIMIT_SWITCH};
byte pinsZ[] = {STEPPER_Z_STEP, STEPPER_Z_DIR, STEPPER_Z_ENABLE, STEPPER_Z_LIMIT_SWITCH};
byte pinsZp[] = {STEPPER_ZP_STEP, STEPPER_ZP_DIR, STEPPER_ZP_ENABLE, STEPPER_ZP_LIMIT_SWITCH};
byte msPins[] = {MS1_PIN, MS2_PIN};

Dispensing dispensing(pinsX, pinsZ, pinsZp, msPins);

void setup()
{
  Serial.begin(115200);
  wifi.init();
  mqtt.init();
  mqtt.subscribe(DISPENSING_DATA_TOPIC);
  dispensing.init();
}

void loop()
{
  wifi.check();
  mqtt.check();
}