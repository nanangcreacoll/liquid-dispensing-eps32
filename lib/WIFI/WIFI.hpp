#ifndef __WIFI
#define __WIFI

#include <Arduino.h>
#include <WiFi.h>

#define WIFI_TIMEOUT_MS 5000
#define WIFI_CONNECTING_PERIOD 500

class WIFI 
{
private:
    String ssid;
    String password;
    bool connection = false;

public:
    WIFI(const char* ssid, const char* password = NULL);
    void init();
    void check();
};

#endif