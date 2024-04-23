#ifndef __MQTT
#define __MQTT

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define MQTT_TIMEOUT_MS 5000
#define MQTT_CONNECTING_PERIOD 500

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

public:
    Mqtt(const char* server, unsigned int port, const char* clientId, const char* username = NULL, const char* password = NULL);
    void init();
    void callback(char* topic, byte* message, unsigned int length);
    void setCallback();
    void subscribe(const char* topic);
    void publish(const char* topic, const char* message);
    void loop();
    void connect();
    void check();
    String getSubMessage();
    String getSubTopic();
    String getPubMessage();
    String getPubTopic();
};

#endif