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
}

void Mqtt::init()
{
    this->connect();
    this->setCallback();
}

void Mqtt::callback(char *topic, byte *message, unsigned int length)
{
    Serial.begin(115200);
    this->subTopic = topic;
    Serial.print("Received message from topic: ");
    Serial.println(topic);
    for (int i = 0; i < length; i++)
    {
        this->subMessage += (char)message[i];
    }
    Serial.println(this->subMessage);
    Serial.flush();
}

void Mqtt::setCallback()
{
    pubSubClient.setCallback([this](char *topic, byte *message, unsigned int length) {
        this->callback(topic, message, length);
    });
}

void Mqtt::subscribe(const char *topic)
{
    this->subTopic = topic;
    pubSubClient.subscribe(topic);
}

void Mqtt::publish(const char *topic, const char *message)
{
    this->pubTopic = topic;
    this->pubMessage = message;
    pubSubClient.publish(topic, message);
}

void Mqtt::loop()
{
    pubSubClient.loop();
}

void Mqtt::connect()
{
    Serial.begin(115200);
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
    Serial.flush();
}

void Mqtt::check()
{
    if (!pubSubClient.connected())
    {
        this->connect();
    }
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