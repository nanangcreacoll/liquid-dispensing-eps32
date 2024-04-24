#include <Arduino.h>
#include <WIFI.hpp>
#include <Mqtt.hpp>
#include <ArduinoJson.h>

#include "env.h"

WIFI wifi(WIFI_SSID, WIFI_PASSWORD);
Mqtt mqtt(MQTT_SERVER, MQTT_PORT, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);

void setup()
{
  wifi.init();
  mqtt.init();
  mqtt.subscribe(DISPENSING_DATA_TOPIC);
}

void loop()
{
  wifi.check();
  mqtt.check();
}