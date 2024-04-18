#include <Arduino.h>

#include "./Hello/Hello.hpp"

Hello nanang("Firmansyah");

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  Serial.println(nanang.helloName());
}