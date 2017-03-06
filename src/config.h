#ifndef config_h
#define config_h

#define APP_VERSION "1"

#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883

// A/C Thermostat Heater pin
#define OWNPIN D6
#define LCDLINELENGTH 10
#define LCDLINEHEIGHT 16
#define LCDBLANKLINE "          "
#define AUTO_MODE_ON 1
#define AUTO_MODE_OFF 2
#define AUTO_MODE_PID 3
#define AUTO_MODE_AUTO 4
#define MENU_ON AUTO_MODE_ON
#define MENU_OFF AUTO_MODE_OFF
#define MENU_PID AUTO_MODE_PID       // PID control
#define MENU_AUTO AUTO_MODE_AUTO     // Dumb auto : setpoint, threshold min/max
#define CONTROL_HIGH_DIFFERENTIAL 10 // error > DIFFERENTIAL -> high differential
#define BUTTON_COUNT 6

#include "application.h"
#include <math.h>
#include "lib/Button.h"
#include "lib/Adafruit_GFX.h"
#include "lib/Adafruit_SSD1306.h"
#include "lib/Adafruit_MCP23017.h"
#include "lib/OneWire.h"
#include "lib/pid.h"
#include "lib/Adafruit_MQTT_SPARK.h"
#include "lib/Adafruit_MQTT.h"
#include "lib/httpclient2.h"

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
    float tempF;

    uint8_t blynkPin;
    Adafruit_MQTT_Publish *aioFeed;
} Thermistor;

typedef struct Actuator
{
    char name[10];
    bool isMcp;
    bool isWebPowerSwitch;
    byte pin;
    Adafruit_MCP23017* mcp;

    bool state;
    unsigned long timer_last;
} Actuator;

typedef struct TemperatureControl
{
    char name[10];

    DSTempSensor *dstempsensor;
    Thermistor *thermistor;
    byte mode; // 1 - ON ; 2 - OFF ; 3 - AUTO
    Actuator actuator;

    double tempF;

    PID pid;
    double target;
    double input;        // yes, this is redundant with tempF, if we run out of memory it can be optimized
    double output;
    double error;

    int min;
    int max;
    int window;
    unsigned long window_start;
    unsigned long window_end;

    double Kp;
    double Ki;
    double Kd;
} TemperatureControl;

typedef struct Chiller
{
    char name[10];

    TemperatureControl *heater;
    Actuator fan;
    DSTempSensor *dstempsensor;
    byte mode;
    bool state;

    int target;

    // normal operation
    int normal_target_offset; // how far below the lowest fermenter target to go
    int normal_threshold_high;
    int normal_threshold_low;

    // high differential operation
    int high_target_offset; // how far below the lowest fermenter target to go
    int high_threshold_high;
    int high_threshold_low;

    int min_temperature;

    // there is a very long tail for all this since the heater warms up slowly
    //  and the a/c won't kick on/off immediately when the heater goes on/off
    // we will monitor the heater temperature
    //  when it is above control_set_temperature, we will assume the a/c is on
    //  when it is below, we will assume the a/c is off
    int min_on_time;    // minimum time for the chiller to be on
    int min_off_time;   // minimum time for the chiller to be off

    int control_set_temperature;   // what the a/c control unit is set to .. we need to set
                                   // the heater above this to make the unit kick on,
                                   // and wait until after it gets below this to say
                                   // the a/c is off .. very inexact

    unsigned long timer_last;
} Chiller;

typedef struct Fermenter
{
    char name[10];
    TemperatureControl *control;
} Fermenter;

float readTempC(DSTempSensor *dstemp);
float readTempC(Thermistor *thermistor);
float convertTempCtoF(float tempC);
void displayLine(byte line, char *message, bool clear);
void resetOWN();
void scanOWN();
void timer_check_buttons();
void read_temperatures();
void read_ds_temperatures();
void setup_pids();
void update_pids();
void update_pid(TemperatureControl *control);
void update_chiller();
void run_controls();
void run_control(TemperatureControl *control);
void actuate(Actuator *actuator, bool on);
void chiller_fan_off();
void chiller_check_heater();
void all_off();
void update_display();
void update_aio();
void update_blynk();
#endif
