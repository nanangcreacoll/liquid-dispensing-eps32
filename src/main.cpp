#include <Arduino.h>
#include <WIFI.hpp>
#include <Mqtt.hpp>
#include <ArduinoJson.h>

#include "env.h"

WIFI wifi(WIFI_SSID, WIFI_PASSWORD);
Mqtt mqtt(MQTT_SERVER, MQTT_PORT, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);

String subTopic = "dispensing/data";
String pubTopic = "dispensing/status";

void setup()
{
  wifi.init();
  mqtt.init();
  mqtt.subscribe(subTopic.c_str());
}

void loop()
{
  wifi.check();
  mqtt.check();
  mqtt.loop();
  JsonDocument doc;
  doc["status"] = true;
  String message;
  serializeJson(doc, message);
  Serial.println(message);
  mqtt.publish(pubTopic.c_str(), message.c_str());
  delay(3000);
}