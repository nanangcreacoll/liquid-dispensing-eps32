#include "WIFI.hpp"

WIFI::WIFI(const char *ssid, const char *password)
{
    this->ssid = ssid;
    this->password = password;
}

void WIFI::init()
{
    Serial.begin(115200);
    WiFi.begin(this->ssid, this->password);

    unsigned long startAttemptTime = millis();
    unsigned long timeNow = 0;
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS)
    {
        if (millis() - timeNow >= WIFI_CONNECTING_PERIOD)
        {
            Serial.println("Connecting to WiFi ...");
            timeNow = millis();
        }
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Failed connecting to WiFi!");
        this->connection = true;
    }
    else
    {
        Serial.print("Connected to WiFi, IP: ");
        Serial.println(WiFi.localIP());
        this->connection = false;
    }

    Serial.flush();
}

void WIFI::check()
{
    Serial.begin(115200);
    if (WiFi.status() != WL_CONNECTED && !this->connection)
    {
        unsigned long timeNow = 0;
        while (WiFi.status() != WL_CONNECTED)
        {
            if (millis() - timeNow >= WIFI_CONNECTING_PERIOD)
            {
                Serial.println("WiFi not connected!");
                timeNow = millis();
            }
        }
        this->connection = true;
    }
    else if (WiFi.status() == WL_CONNECTED && this->connection)
    {
        Serial.print("Connected to WiFi IP: ");
        Serial.println(WiFi.localIP());
        this->connection = false;
    }
    Serial.flush();
}