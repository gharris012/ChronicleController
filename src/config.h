#ifndef config_h
#define config_h

#define APP_VERSION "dh1"

// A/C Thermostat Heater pin
#define OWNPIN D6
#define LCDLINELENGTH 10
#define LCDLINEHEIGHT 16
#define LCDBLANKLINE "          "

#define AUTO_MODE_ON 1
#define AUTO_MODE_OFF 2
#define AUTO_MODE_PID 4
#define AUTO_MODE_AUTO 8
#define AUTO_MODE_CHILL 16
#define AUTO_MODE_HEAT 32

#define MENU_ON 1
#define MENU_OFF 2
#define MENU_PID 3
#define MENU_AUTO 4
#define MENU_AHEAT 5
#define MENU_ACHILL 6
#define MENU_PHEAT 7
#define MENU_PCHILL 8

#define MENU_ON_MODE AUTO_MODE_ON
#define MENU_OFF_MODE AUTO_MODE_OFF
#define MENU_PID_MODE AUTO_MODE_PID        // PID control
#define MENU_AUTO_MODE AUTO_MODE_AUTO      // Dumb auto : setpoint, threshold min/max
#define MENU_PCHILL_MODE AUTO_MODE_PID | AUTO_MODE_CHILL   // PID - Chiller only
#define MENU_PHEAT_MODE AUTO_MODE_PID | AUTO_MODE_HEAT     // PID - Heater only
#define MENU_ACHILL_MODE AUTO_MODE_AUTO | AUTO_MODE_CHILL   // Auto - Chiller only
#define MENU_AHEAT_MODE AUTO_MODE_AUTO | AUTO_MODE_HEAT     // Auto - Heater only

#define CONTROL_HIGH_DIFFERENTIAL 10  // error > DIFFERENTIAL -> high differential
#define ACTION_NONE 1
#define ACTION_CHILL 2
#define ACTION_HEAT 4
#define BUTTON_COUNT 6

#include "Particle.h"
#include <stdlib.h>
#include <math.h>
#include "Button.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_MCP23017.h"
#include "OneWire.h"
#include "pid.h"
#include "HttpClient.h"

struct DSTempSensor
{
    char name[10];
    uint8_t addr[8];
    uint8_t blynkPin;

    float tempF;

    float last_tempF;
    int last_valid_read;
    bool present;
};

struct Thermistor
{
    char name[10];
    byte pin;
    float tempF;

    uint8_t blynkPin;
};

struct Actuator
{
    char name[10];
    bool isMcp;
    bool isWebPowerSwitch;
    byte pin;
    Adafruit_MCP23017* mcp;

    bool target_state; // for non-closed-loop devices, ie: WPS
    uint8_t blynkPin;

    bool state;
    unsigned long timer_last;
};

struct PIDControl
{
    PID pid;
    double Kp;
    double Ki;
    double Kd;

	double output;
	double output_original;
	int adjustedFlag;

    bool publish_results;

    int min;
    int max;
    int window;
    unsigned long window_start;
    unsigned long window_end;
};

struct TemperatureControl
{
    char name[10];

    DSTempSensor *dstempsensor;
    Thermistor *thermistor;
    byte mode; // 1 - ON ; 2 - OFF ; 3 - AUTO
    Actuator heater;
    Actuator chiller;

    uint8_t action; // 1 none, 2 - chill, 4 - heat
    uint8_t last_action; // 1 none, 2 - chill, 4 - heat

    double tempF;
    double target;
    double error;
    double hysterisis;

    PIDControl heat_pid;
    PIDControl chill_pid;
};

struct Chiller
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
    unsigned int min_on_time;    // minimum time for the chiller to be on
    unsigned int min_off_time;   // minimum time for the chiller to be off

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
};

struct Fermenter
{
    char name[10];
    TemperatureControl *control;
};

float readTempC(DSTempSensor *dstemp);
float readTempC(Thermistor *thermistor);
float convertTempCtoF(float tempC);
void display_line(byte line, char *message, bool clear, bool flush);
void resetOWN();
void scanOWN();
void read_temperatures();
void read_ds_temperatures();
bool compute_pid(PIDControl *pid);
void setup_controls();
void update_controls();
void update_control(TemperatureControl *control);
void update_chiller();
void run_controls();
void run_control(TemperatureControl *control);
void verify_actuator(Actuator *actuator);
void actuate(Actuator *actuator, bool on);
void actuate(Actuator *actuator, bool on, bool force);
void chiller_fan_off();
void chiller_check_heater();
void all_off();
void update_display();
void update_blynk();
void check_memory();

void mode_for_display(bool state, char *buffer, byte buffer_size);
void mode_for_display(bool state, float tempF, char *buffer, byte buffer_size);
void mode_for_display(byte mode, float tempF, char *buffer, byte buffer_size);
void mode_as_string(byte mode, char *buffer, byte buffer_size);
void tempF_for_display(float tempF, char buffer[], byte buffer_size);

void ppublish(String message, ...);
#endif
