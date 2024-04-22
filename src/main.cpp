#include <Arduino.h>
#include <WIFI.hpp>

#include "env.h"

WIFI wifi(WIFI_SSID, WIFI_PASSWORD);

void setup()
{
  wifi.init();
}

void loop()
{
  wifi.check();
}