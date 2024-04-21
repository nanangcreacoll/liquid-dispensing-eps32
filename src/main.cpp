#include <Arduino.h>
#include <WIFI.hpp>

#define WIFI_SSID "FIRSTA2"
#define WIFI_PASSWORD "klaten12345"

WIFI wifi(WIFI_SSID, WIFI_PASSWORD);

void setup()
{
  wifi.init();
}

void loop()
{
  wifi.check();
}