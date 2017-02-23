#include "application.h"
#include "Button.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_MCP23017.h"

void debug(String message);
void debug(String message, int value);
void debug(String message, float value);
void debug(String message, float value, float value2);
void debug(String message, int value, int value2);
void debug(String message, char *value);
