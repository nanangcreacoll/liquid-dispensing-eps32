#include <Arduino.h>
#include <WIFI.hpp>
#include <Mqtt.hpp>

#include "env.h"

WIFI wifi(WIFI_SSID);

void setup()
{
  wifi.init();
}

void loop()
{
  wifi.check();
}