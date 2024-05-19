#include "Mqtt.hpp"

Mqtt::Mqtt(const char *server, unsigned int port, const char *clientId, const char *username, const char *password)
{
    this->server = server;
    this->port = port;
    this->username = username;
    this->password = password;
    this->clientId = clientId;
    pubSubClient.setClient(wifiClient);
    pubSubClient.setServer(this->server.c_str(), this->port);
    pubSubClient.setCallback([this](char *topic, byte *message, unsigned int length) {
        this->callback(topic, message, length);
    });
}

void Mqtt::init()
{
    this->connect();
}

void Mqtt::callback(char *topic, byte *message, unsigned int length)
{
    this->subTopic = topic;
    Serial.print("Received message from topic: ");
    Serial.println(topic);
    for (int i = 0; i < length; i++)
    {
        this->subMessage += (char)message[i];
    }
    Serial.println(this->subMessage);
}

bool Mqtt::subscribe(const char *topic)
{
    this->subTopic = topic;
    return pubSubClient.subscribe(topic);
}

bool Mqtt::publish(const char *topic, const char *message)
{
    this->pubTopic = topic;
    this->pubMessage = message;
    return pubSubClient.publish(topic, message);
}

void Mqtt::connect()
{
    unsigned long timeNow = 0;
    while (!pubSubClient.connected())
    {
        if (millis() - timeNow >= MQTT_CONNECTING_PERIOD)
        {
            Serial.println("Connecting to MQTT server ...");
            if (pubSubClient.connect(this->clientId.c_str(), this->username.c_str(), this->password.c_str()))
            {
                Serial.println("Connected to MQTT server!");
                if (this->subTopic != NULL)
                {
                    this->subscribe(this->subTopic.c_str());
                }
            }
            else
            {
                Serial.print("Failed with state: ");
                Serial.println(pubSubClient.state());
            }
            timeNow = millis();
        }
    }
}

void Mqtt::check()
{
    if (!pubSubClient.connected())
    {
        this->connect();
    }
    pubSubClient.loop();
}

bool Mqtt::clearSubMessage()
{
    this->subMessage = "";
    return true;
}

String Mqtt::getPubMessage()
{
    return this->pubMessage;
}

String Mqtt::getPubTopic()
{
    return this->pubTopic;
}

String Mqtt::getSubMessage()
{
    return this->subMessage;
}

String Mqtt::getSubTopic()
{
    return this->subTopic;
}