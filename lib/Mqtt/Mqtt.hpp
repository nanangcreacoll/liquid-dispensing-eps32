#ifndef __MQTT
#define __MQTT

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define MQTT_TIMEOUT_MS 5000 // 5 seconds
#define MQTT_CONNECTING_PERIOD 500 // 0.5 seconds

class Mqtt
{
private:
    WiFiClient wifiClient;
    PubSubClient pubSubClient;

    String server;
    unsigned int port;
    String username;
    String password;
    String clientId;
    String pubTopic;
    String pubMessage;
    String subTopic;
    String subMessage;

    bool connection = false;

public:
    Mqtt(const char* server, unsigned int port, const char* clientId, const char* username = NULL, const char* password = NULL);
    void init();
    void callback(char* topic, byte* message, unsigned int length);
    bool subscribe(const char* topic);
    bool publish(const char* topic, const char* message);
    void connect();
    void check();
    void clearSubMessage();
    String getSubMessage();
    String getSubTopic();
    String getPubMessage();
    String getPubTopic();
};

#endif