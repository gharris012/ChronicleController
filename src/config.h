#ifndef config_h
#define config_h

#include "application.h"
#include <math.h>
#include "Button.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_MCP23017.h"
#include "OneWire.h"
#include "pid.h"
#include "Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT.h"

#define APP_VERSION "05"

#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883

// A/C Thermostat Heater pin
#define ACTPIN A2
#define ACHPIN 8 // on MCP
#define OWNPIN D6
#define LCDLINELENGTH 10
#define LCDLINEHEIGHT 16
#define LCDBLANKLINE "          "
#define BUTTON_COUNT  6
#define MENU_ON 1
#define MENU_OFF 2
#define MENU_AUTO 3

// we don't actually have an OLED reset pin, but the constructor needs it
#define OLED_RESET D4

typedef struct DSTempSensor
{
    char name[10];
    uint8_t addr[8];
    uint8_t blynkPin;
    Adafruit_MQTT_Publish *aioFeed;

    float tempF;
    bool present;
} DSTempSensor;

typedef struct Thermistor
{
    char name[10];
    byte pin;
    uint8_t blynkPin;
    Adafruit_MQTT_Publish *aioFeed;

    float tempF;
} Thermistor;

typedef struct Fermenter
{
    char name[10];
    DSTempSensor *dstemp;
    uint8_t targetTemp;
    byte mode;
};

float readTempC(byte pin); // analog thermistor
float readTempC(DSTempSensor *dstemp);
float convertTempCtoF(float tempC);
void displayLine(byte line, char *message, bool clear);

#endif
