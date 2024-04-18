#ifndef __HELLO
#define __HELLO

#include <Arduino.h>

class Hello
{
private:
  String name;

public:
  Hello(const char* name);
  String helloName();
};

#endif