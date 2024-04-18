#include "Hello.hpp"

Hello::Hello(const char* name)
{
  this->name = name;
}

String Hello::helloName()
{
  String hello = "Hello " + this->name;
  return hello;
}