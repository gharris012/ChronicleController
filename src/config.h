#ifndef config_h
#define config_h

#include "application.h"
#include <math.h>
#include "Button.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_MCP23017.h"
#include "OneWire.h"

// A/C Thermostat Heater pin
#define ACTPIN A2
#define ACHPIN 12 // on MCP
#define OWNPIN D6
#define LCDLINELENGTH 10
#define LCDLINEHEIGHT 16
#define LCDBLANKLINE "          "
#define BUTTON_COUNT  6

// we don't actually have an OLED reset pin, but the constructor needs it
#define OLED_RESET D4

void debug(String message);
void debug(String message, int value);
void debug(String message, float value);
void debug(String message, float value, float value2);
void debug(String message, int value, int value2);
void debug(String message, char *value);
void debug(String message, char *value, float value2);

float readTempC(byte pin); // analog thermistor
float readTempC(uint8_t addr[8]);
float convertTempCtoF(float tempC);

#endif
