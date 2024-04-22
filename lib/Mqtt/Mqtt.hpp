#ifndef __MQTT
#define __MQTT

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient client;
PubSubClient mqtt(client);

class Mqtt
{
private:
    String server;
    unsigned int port;
    String username;
    String password;
    String subsMessage;

public:
    Mqtt(const char* server, unsigned int port, const char* username = NULL, const char* password = NULL);
    void init();
    void callback();
    void subscribe(const char* topic);
    void publish(const char* topic, const char* message);
    String getSubMessage();
};

#endif