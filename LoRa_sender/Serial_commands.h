#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <Arduino.h>

typedef bool (*SerialParameterChangeCallback)(const String &parameter, long value);

void serialSetup();
bool handleSerialCommands(String *unhandledCommand = nullptr,
                          SerialParameterChangeCallback onChange = nullptr);

#endif
