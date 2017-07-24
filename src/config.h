#ifndef config_h
#define config_h

#define APP_VERSION "v5"

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

    float last_tempF;
    int last_valid_read;
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
    boolean log_results;       // send pid results to Particle.publish
    unsigned long log_next_time;    // if we're logging, this is when it is safe to log again

    double tempF;

    PID pid;
    double target;
    double input;        // yes, this is redundant with tempF, if we run out of memory it can be optimized
    double output;
    double error;

    double auto_threshold_high; // if we are above this amount, turn on control full-bore
    double auto_threshold_low;  // if we are below this amount, turn off control
    double hysterisis;

    int window_min;
    int window_max;
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
    int control_temperature_offset_high; // how far over the set temperature to set the heater
                                    // lower is better since we want to recover as quickly
                                    // as possible
    int control_temperature_offset_low; // how far under the control temperature before
                                    // the chiller turns off

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
void display_line(byte line, char *message, bool clear, bool flush);
void resetOWN();
void scanOWN();
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
void check_memory();

void mode_for_display(bool state, char *buffer, byte buffer_size);
void mode_for_display(bool state, float tempF, char *buffer, byte buffer_size);
void mode_for_display(byte mode, float tempF, char *buffer, byte buffer_size);
void mode_as_string(byte mode, char *buffer, byte buffer_size);
void tempF_for_display(float tempF, char buffer[], byte buffer_size);

void to_json(TemperatureControl *control, char *buffer, byte buffer_size);
void to_json(Actuator *actuator, char *buffer, byte buffer_size);
void to_json(Fermenter *fermenter, char *buffer, byte buffer_size);
void to_json(Chiller *chiller, char *buffer, byte buffer_size);

void particle_config_act(const char *event, const char *data);

void ppublish(String message);
void ppublish(String message, int value);
void ppublish(String message, int value, unsigned long int value2);
void ppublish(String message, float value);
void ppublish(String message, const char *value);
void ppublish(String message, const char *value, int value2);
void ppublish(String message, const char *value, const char *value2);

#endif
