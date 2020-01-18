
#include "config.h"
#include "keys.h"
#include <blynk.h>
//#define BLYNK_PRINT Serial

Adafruit_MCP23017 mcp;
byte achState = LOW;
Adafruit_SSD1306 display(-1);
OneWire own(OWNPIN);
HttpClient http;

IPAddress WebPowerSwitch_IPAddress(192, 168, 1, 97);
int WebPowerSwitch_Port = 80;

http_header_t WebPowerSwitch_Headers[] = {
    //  { "Content-Type", "application/json" },
    //  { "Accept" , "application/json" },
    {"Accept", "*/*"},
    {"Authorization", WEBPOWERSWITCH_AUTH},
    {NULL, NULL} // NOTE: Always terminate headers will NULL
};
http_request_t WebPowerSwitch_Request;
http_response_t WebPowerSwitch_Response;
const char WebPowerSwitch_BaseUrl[] = "/outlet?";

SerialLogHandler logHandler(LOG_LEVEL_WARN, {{"app", LOG_LEVEL_INFO},
                                             {"app.control.chiller", LOG_LEVEL_TRACE},
                                             {"app.control.actuator", LOG_LEVEL_TRACE},
                                             {"app.control.pid", LOG_LEVEL_TRACE}});

Logger LogChiller("app.control.chiller");
Logger LogActuator("app.control.actuator");
Logger LogPID("app.control.pid");

// == [ setup ] ==

bool ds_temp_sensor_is_converting = FALSE;
const byte DS_TEMP_SENSOR_CONVERT_DURATION = 250;
const unsigned long int DS_TEMP_GRACE_PERIOD = 60000; // if we haven't gotten a valid temp in this amount of time, mark it as disconnected
const float INVALID_TEMPERATURE = -123;
const byte DS_SENSOR_COUNT = 4;
const byte DS_FERMENTER_1 = 0;
const byte DS_FERMENTER_2 = 1;
const byte DS_AMBIENT = 2;
const byte DS_CHILLER = 3;
DSTempSensor ds_temp_sensor[DS_SENSOR_COUNT] = {
    {"Ferm1", // Fermenter 1
     {0x28, 0xFF, 0x93, 0x76, 0x71, 0x16, 0x4, 0x73},
     1,
     INVALID_TEMPERATURE,
     INVALID_TEMPERATURE,
     0,
     FALSE},
    {"Ferm2", // Fermenter 2
     {0x28, 0xFF, 0x19, 0xE7, 0x70, 0x16, 0x5, 0x9E},
     2,
     INVALID_TEMPERATURE,
     INVALID_TEMPERATURE,
     0,
     FALSE},
    {"Amb", // Ambient
     {0x28, 0xFF, 0xEF, 0xE1, 0x70, 0x16, 0x5, 0xAD},
     9,
     INVALID_TEMPERATURE,
     INVALID_TEMPERATURE,
     0,
     FALSE},
    {"Chill", // Chiller
     {0x28, 0xFF, 0x2A, 0xEA, 0x70, 0x16, 0x5, 0xC9},
     10,
     INVALID_TEMPERATURE,
     INVALID_TEMPERATURE,
     0,
     FALSE}};

const byte THERMISTOR_COUNT = 1;
const byte THERM_HEATER = 0;
const byte THERM_HEATER_PIN = A2;
const byte BLYNK_HEATER_VPIN = 4;
Thermistor thermistors[THERMISTOR_COUNT] = {
    {"A/C TStat", THERM_HEATER_PIN, INVALID_TEMPERATURE, BLYNK_HEATER_VPIN}};

const byte WPS_F1_CHILL_SOCKET = 1;
const byte WPS_F1_HEAT_SOCKET = 3;
TemperatureControl control_F1 = {
    "F1-Ctrl",
    &ds_temp_sensor[DS_FERMENTER_1], // ds temp sensor
    NULL,                            // thermistor
    AUTO_MODE_OFF,                   // mode
    5,                               // blynkMenuPin mode
    14,                              // blynk chill output pin
    20,                              // blynk heat output pin
    24,                              // blynk composite output pin
    0,                               // blynk last composite reported

    {"F1-Heat", FALSE, TRUE, WPS_F1_HEAT_SOCKET, NULL, FALSE, 22, FALSE, 0},   // actuator - wps
    {"F1-Chill", FALSE, TRUE, WPS_F1_CHILL_SOCKET, NULL, FALSE, 18, FALSE, 0}, // actuator - wps
    ACTION_NONE, // action
    ACTION_NONE, // last_action

    INVALID_TEMPERATURE, // tempf
    65,                  // target
    0,                   // error
    2,                   // hysterisis

    {PID(), 5000, 20, 100, 0, 0, 0, FALSE, 5000, 60000, 60000, 0, 0}, // heat-pid
    {PID(), 5000, 20, 100, 0, 0, 0, FALSE, 5000, 60000, 60000, 0, 0}  // chill-pid
};

const byte WPS_F2_CHILL_SOCKET = 2;
const byte WPS_F2_HEAT_SOCKET = 4;
TemperatureControl control_F2 = {
    "F2-Ctrl",
    &ds_temp_sensor[DS_FERMENTER_2],                                           // ds temp sensor
    NULL,                                                                      // thermistor
    AUTO_MODE_OFF,                                                             // mode
    6,                                                                         // blynkMenuPin mode
    15,                              // blynk chill output pin
    21,                              // blynk heat output pin
    25,                              // blynk composite output pin
    0,                               // blynk last composite reported
    {"F2-Heat", FALSE, TRUE, WPS_F2_HEAT_SOCKET, NULL, FALSE, 23, FALSE, 0},   // actuator - wps
    {"F2-Chill", FALSE, TRUE, WPS_F2_CHILL_SOCKET, NULL, FALSE, 19, FALSE, 0}, // actuator - wps
    ACTION_NONE, // action
    ACTION_NONE, // last_action

    INVALID_TEMPERATURE, // tempf
    65,                  // target
    0,                   // error
    2,                   // hysterisis

    {PID(), 5000, 20, 100, 0, 0, 0, FALSE, 5000, 60000, 60000, 0, 0}, // heat-pid
    {PID(), 5000, 20, 100, 0, 0, 0, FALSE, 5000, 60000, 60000, 0, 0}  // chill-pid
};

TemperatureControl control_Heater = {
    "H-Ctrl",

    NULL,                       // ds temp sensor
    &thermistors[THERM_HEATER], // thermistor
    AUTO_MODE_OFF,              // mode
    // blynk pins
    0,
    0,
    0,
    0,
    0,
    {"H-Act", TRUE, FALSE, 8, NULL, FALSE, 0, FALSE, 0}, // actuator - mcp
    {},

    ACTION_NONE, // action
    ACTION_NONE, // last_action

    INVALID_TEMPERATURE, // tempF
    65,
    0,
    0,

    {PID(), 100, 0.15f, 1000, 0, 0, 0, FALSE, 1, 1000, 1000, 0, 0},
    {}};

const byte FERMENTER_COUNT = 2;
const byte F_FERMENTER_1 = 0;
const byte F_FERMENTER_2 = 1;
Fermenter fermenters[FERMENTER_COUNT] = {
    {"F1", &control_F1},
    {"F2", &control_F2}};

// after the chiller is turned off, keep checking heater until it gets below
// {{control_set_temperature}}, then mark chiller as off, and start fan-off
const byte WPS_CHILLER_FAN_SOCKET = 5;
const byte CHILLER_UPDATE_DELAY = 60; // in seconds ~ ish
const byte CHILLER_DEFAULT_TARGET = 35;
const byte CHILLER_HIGH_DIFF_THRESHOLD = 10; // if we are more than 5 degrees off either client, go into high differential mode
const int CHILLER_FAN_POST_TIME = 60000;
const int chiller_check_heater_delay = 1000;
Chiller chiller = {
    "C-Ctrl",
    &control_Heater,
    {"C-Fan", FALSE, TRUE, WPS_CHILLER_FAN_SOCKET, NULL, FALSE, 12, FALSE, 0}, // actuator - wps
    &ds_temp_sensor[DS_CHILLER],
    AUTO_MODE_OFF,
    FALSE,                  // mode, state
    CHILLER_DEFAULT_TARGET, // target
    10,
    5,
    5, // normal: target offset, high threshold, low threshold
    20,
    10,
    10,        // high differential: target offset, high threshold, low threshold
    25,        // min temperature
    5 * 60000, // min on time - 5 minutes
    5 * 60000, // min off time
    80,
    5,
    2, // control_set_temperature, control_temperature_offset_high, control_temperature_offset_low
    0  // timer_last
};

Button buttons[BUTTON_COUNT] = {
    {"Left", 13, &mcp},
    {"Right", 14, &mcp},
    {"Up", 15, &mcp},
    {"Down", 4, &mcp},
    {"Sel", 3, &mcp},
    {"Off", 2, &mcp}};

const int read_temperatures_delay = 1000;
const int update_controls_delay = 1000;
const int check_memory_delay = 5000;
const int update_display_delay = 1000;
const int update_blynk_delay = 5000;

BlynkTimer timer;

void setup()
{
    Serial.begin(115200);
    delay(2000); // allow time to connect serial monitor

    Log.info("System started");
    Log.info("Device ID: %s", (const char *)System.deviceID());
    Log.info("System version: %s", (const char *)System.version());
    Log.info("App version: %s", (const char *)APP_VERSION);
    ppublish("Starting up App Version: %s", (const char *)APP_VERSION);

    Log.info("setting up Heater pin");
    mcp.begin();
    mcp.pinMode(control_Heater.heater.pin, OUTPUT);
    mcp.digitalWrite(control_Heater.heater.pin, LOW);
    pinMode(control_Heater.thermistor->pin, INPUT);

    WebPowerSwitch_Request.ip = WebPowerSwitch_IPAddress;
    WebPowerSwitch_Request.port = WebPowerSwitch_Port;

    Log.info("Turning everything off");
    all_off();
    run_controls();

    Log.info("Setting up OWN");
    scanOWN();

    setup_controls();

    Log.info("Setting up buttons");
    setup_buttons(buttons, BUTTON_COUNT);

    Log.info("setting up blynk");
    Blynk.begin(BLYNK_KEY);

    // initialize with the I2C addr 0x3C (for the diy display)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    // Show the Adafruit splashscreen
    display.display();
    delay(500);

    Log.info("Setting up display");
    // Clear the buffer.
    display.clearDisplay();

    char buffer[LCDLINELENGTH];
    // text display tests
    display.setTextSize(2);
    display.setTextColor(WHITE, 0);
    display.setCursor(0, 0);
    snprintf(buffer, sizeof(buffer), "Ready %s", (const char *)APP_VERSION);
    display.print(buffer);
    display.display();

    timer.setInterval(read_temperatures_delay, read_temperatures);
    timer.setInterval(update_controls_delay, update_controls);
    timer.setInterval(update_display_delay, update_display);
    timer.setInterval(update_blynk_delay, update_blynk);
}

bool rescanOWN = FALSE;
void loop()
{
    Blynk.run();
    timer.run();
    check_buttons(buttons, BUTTON_COUNT);
    run_controls();

    if (rescanOWN)
    {
        scanOWN();
        rescanOWN = FALSE;
    }
    // retry any failed WPS updates
    verify_actuator(&chiller.fan);
    verify_actuator(&control_F1.heater);
    verify_actuator(&control_F1.chiller);
    verify_actuator(&control_F2.heater);
    verify_actuator(&control_F2.chiller);
}

void check_memory()
{
    Log.info("Free memory: %ld", System.freeMemory());
}

/*
    **********
    A 76.0 02 -- prod
    H 80 80 1 -- dev
    1 81.4 OF
    2 71.2 OF
    C 71.1 OF
    **********
*/
void update_display()
{
    const byte mbuf_size = 5;
    const byte fbuf_size = 5;
    char buffer[LCDLINELENGTH];
    char mbuf[mbuf_size];
    char fbuf[fbuf_size];

    //snprintf(buf, sizeof(buf), "A %2.1f %s", dsTemp[2].tempF, APP_VERSION);

    // if the heater is off, show ambient and version info
    if (chiller.heater->mode == AUTO_MODE_OFF)
    {
        memset(buffer, 0, sizeof(buffer));
        memset(fbuf, 0, sizeof(fbuf));
        tempF_for_display(ds_temp_sensor[DS_AMBIENT].tempF, fbuf, fbuf_size);
        snprintf(buffer, sizeof(buffer), "A %s %s", fbuf, APP_VERSION);
        display_line(0, buffer, FALSE, FALSE);
    }
    else
    {
        memset(buffer, 0, sizeof(buffer));
        memset(mbuf, 0, sizeof(mbuf));
        memset(fbuf, 0, sizeof(fbuf));
        mode_for_display(chiller.heater->mode, chiller.heater->target, mbuf, mbuf_size);
        tempF_for_display(chiller.heater->tempF, fbuf, fbuf_size);
        snprintf(buffer, sizeof(buffer), "H %s %s", fbuf, mbuf);
        display_line(0, buffer, FALSE, FALSE);
    }

    memset(buffer, 0, sizeof(buffer));
    memset(mbuf, 0, sizeof(mbuf));
    memset(fbuf, 0, sizeof(fbuf));
    mode_for_display(fermenters[F_FERMENTER_1].control->mode, fermenters[F_FERMENTER_1].control->target, mbuf, mbuf_size);
    tempF_for_display(fermenters[F_FERMENTER_1].control->tempF, fbuf, fbuf_size);
    snprintf(buffer, sizeof(buffer), "1 %s %3s", fbuf, mbuf);
    display_line(1, buffer, FALSE, FALSE);

    memset(buffer, 0, sizeof(buffer));
    memset(mbuf, 0, sizeof(mbuf));
    memset(fbuf, 0, sizeof(fbuf));
    mode_for_display(fermenters[F_FERMENTER_2].control->mode, fermenters[F_FERMENTER_2].control->target, mbuf, mbuf_size);
    tempF_for_display(fermenters[F_FERMENTER_2].control->tempF, fbuf, fbuf_size);
    snprintf(buffer, sizeof(buffer), "2 %s %3s", fbuf, mbuf);
    display_line(2, buffer, FALSE, FALSE);

    memset(buffer, 0, sizeof(buffer));
    memset(mbuf, 0, sizeof(mbuf));
    memset(fbuf, 0, sizeof(fbuf));
    mode_for_display(chiller.mode, chiller.target, mbuf, mbuf_size);
    tempF_for_display(chiller.dstempsensor->tempF, fbuf, fbuf_size);
    snprintf(buffer, sizeof(buffer), "C %s %3s", fbuf, mbuf);
    display_line(3, buffer, FALSE, TRUE);
}

int blynk_pid_f1_last_report = 0;
int blynk_pid_f2_last_report = 0;
void update_blynk()
{
    int i;
    for (i = 0; i < THERMISTOR_COUNT; i++)
    {
        if (thermistors[i].blynkPin >= 0 && thermistors[i].tempF != INVALID_TEMPERATURE)
        {
            Blynk.virtualWrite(thermistors[i].blynkPin, thermistors[i].tempF);
        }
    }
    for (i = 0; i < DS_SENSOR_COUNT; i++)
    {
        if (ds_temp_sensor[i].blynkPin >= 0)
        {
            if (!ds_temp_sensor[i].present)
            {
                ppublish("Sensor not found: %s at %d", ds_temp_sensor[i].name, i);
                rescanOWN = TRUE;
            }

            if (ds_temp_sensor[i].present && ds_temp_sensor[i].tempF != INVALID_TEMPERATURE)
            {
                Blynk.virtualWrite(ds_temp_sensor[i].blynkPin, ds_temp_sensor[i].tempF);
            }
        }
    }

    blynk_report_fermenter(&fermenters[F_FERMENTER_1]);
    blynk_report_fermenter(&fermenters[F_FERMENTER_2]);

    // set chiller mode - V13
    Blynk.virtualWrite(13, mode_as_menu(chiller.mode));
    // set chill target - V11
    Blynk.virtualWrite(11, chiller.target);
    // set fan status - 1/0 - V12
    Blynk.virtualWrite(chiller.fan.blynkPin, (chiller.fan.state ? 255 : 0));
}

void blynk_report_fermenter(Fermenter *fermenter)
{
    Blynk.virtualWrite(fermenter->control->blynkMenuPin, mode_as_menu(fermenter->control->mode));
    Blynk.virtualWrite(fermenter->control->chiller.blynkPin, (fermenter->control->chiller.state ? 255 : 0));
    Blynk.virtualWrite(fermenter->control->heater.blynkPin, (fermenter->control->heater.state ? 255 : 0));

    int8_t reportHeat = 0;
    int8_t reportChill = 0;
    if ((fermenter->control->mode & AUTO_MODE_ON) > 0)
    {
        if ((fermenter->control->mode & AUTO_MODE_CHILL) > 0)
        {
            reportChill = 100;
        }
        if ((fermenter->control->mode & AUTO_MODE_HEAT) > 0)
        {
            reportHeat = 100;
        }
    }
    else if (fermenter->control->mode == AUTO_MODE_OFF)
    {
        reportHeat = 0;
        reportChill = 0;
    }
    else if (fermenter->control->mode == AUTO_MODE_AUTO)
    {
        reportHeat = (fermenter->control->heater.state ? 100 : 0);
        reportChill = (fermenter->control->chiller.state ? 100 : 0);
    }
    else if ((fermenter->control->mode & AUTO_MODE_PID) > 0)
    {
        if ((fermenter->control->mode & AUTO_MODE_CHILL) > 0)
        {
            reportChill = ((fermenter->control->chill_pid.output / fermenter->control->chill_pid.window) * 100);
        }
        if ((fermenter->control->mode & AUTO_MODE_HEAT) > 0)
        {
            reportHeat = ((fermenter->control->heat_pid.output / fermenter->control->heat_pid.window) * 100);
        }
    }
    int8_t reportComposite = 0;
    if (reportHeat > reportChill)
    {
        reportComposite = reportHeat;
    }
    else
    {
        reportComposite = -1 * reportChill;
    }

    /// CompositeOutput range is -100 (full chill) to 100 (full heat), 0 is off
    Blynk.virtualWrite(fermenter->control->blynkCompositeOutputPin, reportComposite);
    Blynk.virtualWrite(fermenter->control->blynkHeatOutputPin, reportHeat);
    Blynk.virtualWrite(fermenter->control->blynkChillOutputPin, reportChill);
    if (reportComposite != fermenter->control->blynkLastComposite)
    {
        ppublish("Report: %s: %d (c:%d,h:%d)", fermenter->name, reportComposite, reportChill, reportHeat);
        fermenter->control->blynkLastComposite = reportComposite;
    }
}

void tempF_for_display(float tempF, char buffer[], byte buffer_size)
{
    if (tempF == INVALID_TEMPERATURE)
    {
        snprintf(buffer, buffer_size, "nc");
    }
    else
    {
        snprintf(buffer, buffer_size, "%2.0f", tempF);
    }
}

void mode_for_display(bool state, char *buffer, byte buffer_size)
{
    if (state)
    {
        mode_as_string(AUTO_MODE_ON, buffer, buffer_size);
    }
    else
    {
        mode_as_string(AUTO_MODE_OFF, buffer, buffer_size);
    }
}

void mode_for_display(bool state, float tempF, char *buffer, byte buffer_size)
{
    if (state)
    {
        snprintf(buffer, buffer_size, "%2.0f", tempF);
    }
    else
    {
        mode_as_string(AUTO_MODE_OFF, buffer, buffer_size);
    }
}

void mode_for_display(byte mode, float tempF, char *buffer, byte buffer_size)
{
    if (mode == AUTO_MODE_AUTO || mode == AUTO_MODE_PID)
    {
        snprintf(buffer, buffer_size, "%2.0f", tempF);
    }
    else
    {
        mode_as_string(mode, buffer, buffer_size);
    }
}

void action_as_string(byte mode, char *buffer, byte buffer_size)
{
    switch (mode)
    {
        case ACTION_NONE:
            snprintf(buffer, buffer_size, "none");
            break;
        case ACTION_HEAT:
            snprintf(buffer, buffer_size, "heat");
            break;
        case ACTION_CHILL:
            snprintf(buffer, buffer_size, "chill");
            break;
        default:
            snprintf(buffer, buffer_size, "unk");
            break;
    }
}

void mode_as_string(byte mode, char *buffer, byte buffer_size)
{
    switch (mode)
    {
    case CONTROL_MODE_OFF:
        snprintf(buffer, buffer_size, "OFF");
        break;
    case CONTROL_MODE_ON_CHILL:
        snprintf(buffer, buffer_size, "OC");
        break;
    case CONTROL_MODE_ON_HEAT:
        snprintf(buffer, buffer_size, "OH");
        break;
    case CONTROL_MODE_AUTO:
        snprintf(buffer, buffer_size, "AA");
        break;
    case CONTROL_MODE_PID:
        snprintf(buffer, buffer_size, "PA");
        break;
    case CONTROL_MODE_AUTO_CHILL:
        snprintf(buffer, buffer_size, "AC");
        break;
    case CONTROL_MODE_AUTO_HEAT:
        snprintf(buffer, buffer_size, "AH");
        break;
    case CONTROL_MODE_PID_CHILL:
        snprintf(buffer, buffer_size, "PC");
        break;
    case CONTROL_MODE_PID_HEAT:
        snprintf(buffer, buffer_size, "PH");
        break;

    default:
        snprintf(buffer, buffer_size, "UN");
    }
}

void resetOWN()
{
    byte i = 0;
    for (i = 0; i < DS_SENSOR_COUNT; i++)
    {
        ds_temp_sensor[i].tempF = 0;
        ds_temp_sensor[i].present = FALSE;
    }
}

void scanOWN()
{
    uint8_t addr[8];
    byte i = 0;

    for (i = 0; i < DS_SENSOR_COUNT; i++)
    {
        ds_temp_sensor[i].present = FALSE;
    }

    Log.info("Searching OWN");
    ppublish("Searching OWN");
    if (own.reset() == 1)
    {
        Log.info("Network present");
        ppublish("OWN Network present");
    }
    else
    {
        ppublish("OWN Network problem!");
        Log.info("Network problem :(");
    }

    own.reset_search();
    delay(250);
    // search own for sensors
    while (own.search(addr))
    {
        Log.info("Found: %02X-%02X", addr[6], addr[7]);
        for (i = 0; i < DS_SENSOR_COUNT; i++)
        {
            if (memcmp(addr, ds_temp_sensor[i].addr, 8) == 0)
            {
                Log.trace(" %s at index %d", ds_temp_sensor[i].name, i);
                ppublish("Found sensor %s at %d", ds_temp_sensor[i].name, i);
                ds_temp_sensor[i].present = TRUE;
            }
        }
    }
    for (i = 0; i < DS_SENSOR_COUNT; i++)
    {
        if (!ds_temp_sensor[i].present)
        {
            Log.warn("Sensor %s at index %d not found!", ds_temp_sensor[i].name, i);
            ppublish("Sensor not found! %s at %d", ds_temp_sensor[i].name, i);
        }
    }

    // set resolution for ds temp sensors
    own.reset();
    own.skip();
    own.write(0x4E); // Write scratchpad
    own.write(0);    // TL
    own.write(0);    // TH
    own.write(0x3F); // 10-bit resolution
    own.write(0x48); // Copy Scratchpad
    own.write(0x44); // start conversion
    own.reset();

    own.reset_search();
}

void display_line(byte line, char *message, bool clear, bool flush)
{
    char lcdLineBuf[LCDLINELENGTH];

    if (clear)
    {
        display.setCursor(0, LCDLINEHEIGHT * line);
        memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
        snprintf(lcdLineBuf, LCDLINELENGTH, LCDBLANKLINE);
        display.print(lcdLineBuf);
        display.display();
    }

    display.setCursor(0, LCDLINEHEIGHT * line);
    memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
    snprintf(lcdLineBuf, LCDLINELENGTH, "%s%s", message, LCDBLANKLINE);
    display.print(lcdLineBuf);
    if (clear || flush)
    {
        display.display();
    }
}

void setup_controls()
{
    control_F1.chill_pid.pid.Setup(&control_F1.tempF, &control_F1.chill_pid.output, &control_F1.target,
                                  control_F1.chill_pid.Kp, control_F1.chill_pid.Ki, control_F1.chill_pid.Kd,
                                  PID::P_ON_E, PID::REVERSE);
    control_F1.chill_pid.pid.SetOutputLimits(0, control_F1.chill_pid.max);
    control_F1.chill_pid.pid.SetMode(PID::AUTOMATIC);
    control_F1.chill_pid.pid.SetSampleTime(control_F1.chill_pid.window);

    control_F1.heat_pid.pid.Setup(&control_F1.tempF, &control_F1.heat_pid.output, &control_F1.target,
                                 control_F1.heat_pid.Kp, control_F1.heat_pid.Ki, control_F1.heat_pid.Kd,
                                 PID::P_ON_E, PID::DIRECT);
    control_F1.heat_pid.pid.SetOutputLimits(0, control_F1.heat_pid.max);
    control_F1.heat_pid.pid.SetMode(PID::AUTOMATIC);
    control_F1.heat_pid.pid.SetSampleTime(control_F1.heat_pid.window);

    control_F2.chill_pid.pid.Setup(&control_F2.tempF, &control_F2.chill_pid.output, &control_F2.target,
                                  control_F2.chill_pid.Kp, control_F2.chill_pid.Ki, control_F2.chill_pid.Kd,
                                  PID::P_ON_E, PID::REVERSE);
    control_F2.chill_pid.pid.SetOutputLimits(0, control_F2.chill_pid.max);
    control_F2.chill_pid.pid.SetMode(PID::AUTOMATIC);
    control_F2.chill_pid.pid.SetSampleTime(control_F2.chill_pid.window);

    control_F2.heat_pid.pid.Setup(&control_F2.tempF, &control_F2.heat_pid.output, &control_F2.target,
                                 control_F2.heat_pid.Kp, control_F2.heat_pid.Ki, control_F2.heat_pid.Kd,
                                 PID::P_ON_E, PID::DIRECT);
    control_F2.heat_pid.pid.SetOutputLimits(0, control_F2.heat_pid.max);
    control_F2.heat_pid.pid.SetMode(PID::AUTOMATIC);
    control_F2.heat_pid.pid.SetSampleTime(control_F2.heat_pid.window);

    control_Heater.heat_pid.pid.Setup(&control_Heater.tempF, &control_Heater.heat_pid.output, &control_Heater.target,
                                     control_Heater.heat_pid.Kp, control_Heater.heat_pid.Ki, control_Heater.heat_pid.Kd,
                                     PID::P_ON_E, PID::DIRECT);
    control_Heater.heat_pid.pid.SetOutputLimits(0, control_Heater.heat_pid.max);
    control_Heater.heat_pid.pid.SetMode(PID::AUTOMATIC);
    control_Heater.heat_pid.pid.SetSampleTime(control_Heater.heat_pid.window);
}

void update_controls()
{
    static int check_count = 0;

    check_count++;
    Log.trace("Updating Controls");
    update_control(&control_F1);
    update_control(&control_F2);
    update_control(&control_Heater);

    // chiller doesn't need to be updated very often
    if (check_count >= CHILLER_UPDATE_DELAY)
    {
        update_chiller();
        check_count = 0;
    }
}

bool compute_pid(PIDControl *pid)
{
    double output_adjusted = 0;
    int adjustedFlag = 0;

    // returns true when a new computation has been done
    // ie: new window
    if (pid->pid.Compute())
    {
        output_adjusted = pid->output;
        if (output_adjusted < 0)
        {
            output_adjusted = 0;
            adjustedFlag |= 1;
        }
        pid->window_start = millis();
        if (pid->min > output_adjusted)
        {
            // set window to min if output is more than half of min, otherwise 0
            if ((pid->min / 2) < output_adjusted)
            {
                output_adjusted = pid->min;
                adjustedFlag |= 2;
            }
            else
            {
                output_adjusted = 0;
                adjustedFlag |= 4;
            }
        }
        // enforce window_min for off-time as well
        // -- leave window_min at the end
        else if (output_adjusted > (pid->max - pid->min))
        {
            // round up
            if (output_adjusted > (pid->max - (pid->min / 2)))
            {
                output_adjusted = pid->max;
                adjustedFlag |= 8;
            }
            else
            {
                output_adjusted = pid->max - pid->min;
                adjustedFlag |= 16;
            }
        }

        // extend the max window a bit to make sure we stay on full-time
        if (output_adjusted == pid->max)
        {
            pid->window_end = millis() + output_adjusted + 5000;
        }
        else
        {
            pid->window_end = millis() + output_adjusted;
        }

        // reset output to adjusted so PID can use it in the next computation
        pid->output_original = pid->output;
        pid->output = output_adjusted;
        pid->adjustedFlag = adjustedFlag;

        return true;
    }
    else
    {
        return false;
    }
}

// calculate and update vars - every second
void update_control(TemperatureControl *control)
{
    Log.trace("Updating Control for %s", control->name);
    if (control->dstempsensor != NULL)
    {
        control->tempF = control->dstempsensor->tempF;
    }
    else if (control->thermistor != NULL)
    {
        control->tempF = control->thermistor->tempF;
    }
    else
    {
        ppublish("No temperature source for %s!", control->name);
        Log.warn(" no temperature source!");
        control->tempF = INVALID_TEMPERATURE;
    }

    if (control->tempF == INVALID_TEMPERATURE)
    {
        ppublish("Invalid temperature reading for %s!", control->name);
        Log.warn("Invalid temperature reading for %s!", control->name);
        control->action = ACTION_NONE;
        control->error = 0;
    }
    else
    {
        control->error = control->target - control->tempF;

        // negative -> needs cold

        // positive -> needs heat
        //  or we're in a cooling loop and within hysterisis
        if (control->error < 0
            || (abs(control->error) <= control->hysterisis
                && ( control->mode & AUTO_MODE_PID ) > 0 && control->last_action == ACTION_CHILL))
        {
            if ((control->mode & AUTO_MODE_CHILL) > 0)
            {
                if ((control->mode & AUTO_MODE_PID) > 0)
                {
                    if (compute_pid(&control->chill_pid))
                    {
                        if (control->chill_pid.publish_results)
                        {
                            char buffer[50];
                            snprintf(buffer, 50, "%s PID: %3.2f %3.2f %3.2f %d", control->name, control->error, control->chill_pid.output_original, control->chill_pid.output, control->chill_pid.adjustedFlag);
                            ppublish(buffer);
                            LogPID.trace(buffer);
                        }
                    }
                    // as long as we stay within hysterisis, keep action_chill
                    control->action = ACTION_CHILL;
                }
                else if ((control->mode & AUTO_MODE_AUTO) > 0)
                {
                    if (abs(control->error) > control->hysterisis)
                    {
                        control->action = ACTION_CHILL;
                    }
                    else
                    {
                        control->action = ACTION_NONE;
                    }
                }
            }
            else
            {
                // above temp, chill disabled, nothing to do
                control->action = ACTION_NONE;
            }
        }
        else if (control->error > 0
                || (abs(control->error) <= control->hysterisis
                    && (control->mode & AUTO_MODE_PID) > 0 && control->last_action == ACTION_HEAT))
        {
            if ((control->mode & AUTO_MODE_HEAT) > 0)
            {
                if ((control->mode & AUTO_MODE_PID) > 0)
                {
                    if (compute_pid(&control->heat_pid))
                    {
                        if (control->heat_pid.publish_results)
                        {
                            char buffer[50];
                            snprintf(buffer, 50, "%s PID: %3.2f %3.2f %3.2f %d", control->name, control->error, control->heat_pid.output_original, control->heat_pid.output, control->heat_pid.adjustedFlag);
                            ppublish(buffer);
                            LogPID.trace(buffer);
                        }
                    }
                    // as long as we stay within hysterisis, keep action_heat
                    control->action = ACTION_HEAT;
                }
                else if ((control->mode & AUTO_MODE_AUTO) > 0)
                {
                    if (abs(control->error) > control->hysterisis)
                    {
                        control->action = ACTION_HEAT;
                    }
                    else
                    {
                        control->action = ACTION_NONE;
                    }
                }

            }
            else
            {
                // below temp, heat disabled, nothing to do
                control->action = ACTION_NONE;
            }
        }
        else
        {
            // at temp, things disabled? nothing to do
            LogPID.info("%s control: error: %3.2f ; action: %d ; last_action: %d nothing to do", control->name, control->error, control->action, control->last_action);
        }
    }

    char buffer[50];
    char actbuf[5];
    char lastactbuf[5];
    char mbuf[5];
    action_as_string(control->action, actbuf, 5);
    action_as_string(control->last_action, lastactbuf, 5);
    mode_as_string(control->mode, mbuf, 5);
    snprintf(buffer, 50, "%s(%s): e:%3.2f; a:%s; l:%s",
        control->name,
        mbuf,
        control->error,
        actbuf,
        lastactbuf);
    ppublish(buffer);
    LogPID.trace(buffer);
}

// this puppy is special
void update_chiller()
{
    LogChiller.trace("updating chiller");

    bool state = FALSE;

    if ((chiller.mode & AUTO_MODE_AUTO) > 0)
    {
        // set up chiller target
        //  gather fermenter target temperatures
        //  chiller target = min of both - offset, or min_temperature, whichever is higher

        float f_diff = 0;
        float f_target = 100;

        // 'loop' over fermenters and find lowest target, and biggest differential
        if ((fermenters[F_FERMENTER_1].control->mode & AUTO_MODE_CHILL) > 0)
        {
            if (fermenters[F_FERMENTER_1].control->target < f_target)
            {
                f_target = fermenters[F_FERMENTER_1].control->target;
            }
            if (fermenters[F_FERMENTER_1].control->tempF != INVALID_TEMPERATURE &&
                fermenters[F_FERMENTER_1].control->tempF - fermenters[F_FERMENTER_1].control->target > f_diff)
            {
                f_diff = fermenters[F_FERMENTER_1].control->tempF - fermenters[F_FERMENTER_1].control->target;
            }
        }
        if ((fermenters[F_FERMENTER_2].control->mode & AUTO_MODE_CHILL) > 0)
        {
            if (fermenters[F_FERMENTER_2].control->target < f_target)
            {
                f_target = fermenters[F_FERMENTER_2].control->target;
            }
            if (fermenters[F_FERMENTER_2].control->tempF != INVALID_TEMPERATURE &&
                fermenters[F_FERMENTER_2].control->tempF - fermenters[F_FERMENTER_2].control->target > f_diff)
            {
                f_diff = fermenters[F_FERMENTER_2].control->tempF - fermenters[F_FERMENTER_2].control->target;
            }
        }

        int offset;
        int threshold_high;
        int threshold_low;

        if (f_diff > CHILLER_HIGH_DIFF_THRESHOLD)
        {
            LogChiller.trace(" high differential");
            //ppublish("Chiller High Differential: %2.0f", f_diff);
            offset = chiller.high_target_offset;
            threshold_high = chiller.high_threshold_high;
            threshold_low = chiller.high_threshold_low;
        }
        else
        {
            LogChiller.trace(" normal differential");
            //ppublish("Chiller Normal Differential: %2.0f", f_diff);
            offset = chiller.normal_target_offset;
            threshold_high = chiller.normal_threshold_high;
            threshold_low = chiller.normal_threshold_low;
        }

        // apply offset, then constrain between min and default
        chiller.target = constrain(f_target - offset, chiller.min_temperature, CHILLER_DEFAULT_TARGET);

        LogChiller.trace(" current: %2f ; target: %2d", chiller.dstempsensor->tempF, chiller.target);
        LogChiller.trace(" threshold high: %d ; low: %d", threshold_high, threshold_low);
        LogChiller.trace(" on temp: %2d ; off temp: %2d", chiller.target + threshold_high, chiller.target - threshold_low);
        ppublish("Chiller Target: %d < %d < %d", chiller.target - threshold_low, (int)chiller.target, chiller.target + threshold_high);

        // if we're off, kick on when we get over target+threshold
        // if we're on, stay on until we are below target-threshold
        if ((chiller.state == FALSE && chiller.dstempsensor->tempF > (chiller.target + threshold_high))
            || (chiller.state == TRUE && chiller.dstempsensor->tempF > (chiller.target - threshold_low)))
        {
            state = TRUE;
        }
        else
        {
            state = FALSE;
        }
    }
    else if ((chiller.mode & AUTO_MODE_ON) > 0)
    {
        state = TRUE;
    }
    else if (chiller.mode == AUTO_MODE_OFF)
    {
        state = FALSE;
    }
    else
    {
        LogChiller.warn(" Unknown mode requested: %d", chiller.mode);
    }
    LogChiller.trace(" prefilter state: %d", state);
    ppublish("Chiller pre-filter state: %d", state);

    // Filter Chiller logic: have we been on or off long enough?
    //  some cushion to allow initialization
    if (chiller.state != state && chiller.timer_last > 5000)
    {
        unsigned long int next_available_time = 0;
        // on and on_time > min_on_time
        if (chiller.state == TRUE && (( millis() - chiller.timer_last ) >= chiller.min_on_time))
        {
            next_available_time = chiller.timer_last + chiller.min_on_time;
            state = TRUE;
        }
        // off and off_time > min_off_time
        if (chiller.state == FALSE && (( millis() - chiller.timer_last ) >= chiller.min_off_time))
        {
            next_available_time = chiller.timer_last + chiller.min_off_time;
            state = FALSE;
        }

        if (chiller.state == state)
        {
            LogChiller.warn(" overriding due to min on/off time: %d ; next time: %ld", chiller.state, next_available_time);
            ppublish(" overriding due to min on/off time: %d ; next time: %ld", chiller.state, next_available_time);
        }
    }

    // override : current temp < min temp -> shut it down!
    //  ... or at least start the shut down process
    if (chiller.dstempsensor->tempF != INVALID_TEMPERATURE && chiller.dstempsensor->tempF <= chiller.min_temperature &&
        (state == TRUE || chiller.state == TRUE))
    {
        LogChiller.warn(" temp too low, shutting down");
        ppublish("chiller: temp too low, shutting down");
        state = FALSE;
    }

    // if we decide the A/C should be on, turn on the heater PID
    LogChiller.trace(" chiller state: %d ; timer_last: %ld", chiller.state, chiller.timer_last);
    if (chiller.state != state || (chiller.state == FALSE && chiller.timer_last == 0))
    {
        LogChiller.trace(" updating chiller state");
        chiller.state = state;
        chiller.timer_last = millis();
        if (state)
        {
            LogChiller.info(" turning Chiller ON");
            ppublish("Turning Chiller ON");
            actuate(&chiller.fan, TRUE);

            chiller.heater->mode = AUTO_MODE_PID & AUTO_MODE_HEAT;
            chiller.heater->target = chiller.control_set_temperature + chiller.control_temperature_offset_high;
        }

        else
        {
            LogChiller.info(" turning Chiller OFF");
            ppublish("Turning Chiller OFF");
            chiller.heater->mode = AUTO_MODE_OFF;
            timer.setTimeout(chiller_check_heater_delay, chiller_check_heater);
        }
    }
    else
    {
        LogChiller.trace(" nothing to do!");
    }
}

void chiller_check_heater()
{
    LogChiller.trace(" check chiller heater: %2.2f < %2d", chiller.heater->tempF, chiller.control_set_temperature);
    if (chiller.state == FALSE)
    {
        if (chiller.heater->tempF < (chiller.control_set_temperature - chiller.control_temperature_offset_low))
        {
            LogChiller.info(" heater is back down below control_set_temperature, marking Chiller off");
            ppublish("heater is below control, marking Chiller off");
            chiller.state = FALSE;
            chiller.timer_last = millis();
            timer.setTimeout(CHILLER_FAN_POST_TIME, chiller_fan_off);
            Log.trace(" scheduling fan off");
        }
        else
        {
            timer.setTimeout(chiller_check_heater_delay, chiller_check_heater);
        }
    }
}

void chiller_fan_off()
{
    actuate(&chiller.fan, FALSE);
}

// check pid/controls and turn on/off as needed based on window
void run_controls()
{
    run_control(&control_F1);
    run_control(&control_F2);
    run_control(&control_Heater);
}

// determine, right now, if a control should be on/off
void run_control(TemperatureControl *control)
{
    control->last_action = 0;
    if ((control->mode & AUTO_MODE_PID) > 0)
    {
        if ((control->action & ACTION_CHILL) > 0 )
        {
            if (millis() >= control->chill_pid.window_start && millis() <= control->chill_pid.window_end)
            {
                actuate(&control->chiller, TRUE);
            }
            else
            {
                actuate(&control->chiller, FALSE);
            }
        }
        if ((control->action & ACTION_HEAT) > 0)
        {
            if (millis() >= control->heat_pid.window_start && millis() <= control->heat_pid.window_end)
            {
                actuate(&control->heater, TRUE);
            }
            else
            {
                actuate(&control->heater, FALSE);
            }
        }

        control->last_action |= control->action;
        control->action = ACTION_NONE;
    }
    else if ((control->mode & AUTO_MODE_AUTO) > 0)
    {
        if ((control->action & ACTION_CHILL) > 0)
        {
            actuate(&control->chiller, TRUE);
            if ((control->last_action & ACTION_HEAT )> 0)
            {
                actuate(&control->heater, FALSE);
            }
        }
        else if ((control->action & ACTION_HEAT) >0)
        {
            actuate(&control->heater, TRUE);
            if ((control->last_action & ACTION_CHILL) > 0)
            {
                actuate(&control->chiller, FALSE);
            }
        }
        control->last_action |= control->action;
        control->action = ACTION_NONE;
    }
    else if ((control->mode & AUTO_MODE_ON) > 0)
    {
        control->last_action = 0;
        if ((control->mode & AUTO_MODE_CHILL) > 0)
        {
            actuate(&control->chiller, TRUE);
            control->last_action |= ACTION_CHILL;
        }
        if ((control->mode & AUTO_MODE_HEAT) > 0)
        {
            actuate(&control->heater, TRUE);
            control->last_action |= ACTION_HEAT;
        }
        control->action = ACTION_NONE;
    }
    else if ((control->mode & AUTO_MODE_OFF) > 0)
    {
        actuate(&control->chiller, FALSE);
        actuate(&control->heater, FALSE);
        control->last_action = ACTION_NONE;
        control->action = ACTION_NONE;
    }
    else
    {
        Log.warn(" Invalid control mode: %d", control->mode);
    }
}

void verify_actuator(Actuator *actuator)
{
    if (actuator->target_state != actuator->state)
    {
        actuate(actuator, actuator->target_state);
    }
}

void actuate(Actuator *actuator, bool on)
{
    actuate(actuator, on, FALSE);
}

// do the actual on/off
void actuate(Actuator *actuator, bool on, bool force)
{
    actuator->target_state = on;
    if (on)
    {
        if (actuator->state == FALSE || force)
        {
            LogActuator.info("  actuator: turning %s ON", actuator->name);
            if (actuator->isMcp)
            {
                actuator->mcp->digitalWrite(actuator->pin, HIGH);
                actuator->state = TRUE;
                actuator->timer_last = millis();
            }
            else if (actuator->isWebPowerSwitch)
            {
                ppublish("actuator: turning %s ON", actuator->name);

                String path = WebPowerSwitch_BaseUrl;
                path += actuator->pin;
                path += "=ON";
                WebPowerSwitch_Request.path = path.c_str();
                LogActuator.trace("  actuator: wps path: %s", path.c_str());
                http.get(WebPowerSwitch_Request, WebPowerSwitch_Response, WebPowerSwitch_Headers);
                if (WebPowerSwitch_Response.status != 200)
                {
                    ppublish("actuator: %s WPS failed: %d", actuator->name, WebPowerSwitch_Response.status);
                    LogActuator.warn(" Response Status: %d", WebPowerSwitch_Response.status);
                    LogActuator.warn(" Response Body: %s", WebPowerSwitch_Response.body.c_str());
                }
                else
                {
                    LogActuator.trace(" Response Status: %d", WebPowerSwitch_Response.status);
                    actuator->state = TRUE;
                    actuator->timer_last = millis();
                }
            }
            else
            {
                digitalWrite(actuator->pin, HIGH);
                actuator->state = TRUE;
                actuator->timer_last = millis();
            }
        }
    }
    else
    {
        // when we're just starting up, force things off
        if (actuator->state == TRUE || force)
        {
            LogActuator.trace("  actuator: turning %s OFF", actuator->name);
            if (actuator->isMcp)
            {
                actuator->mcp->digitalWrite(actuator->pin, LOW);
                actuator->state = FALSE;
                actuator->timer_last = millis();
            }
            else if (actuator->isWebPowerSwitch)
            {
                ppublish("actuator: turning %s OFF", actuator->name);

                String path = WebPowerSwitch_BaseUrl;
                path += actuator->pin;
                path += "=OFF";
                WebPowerSwitch_Request.path = path.c_str();
                LogActuator.trace("  actuator: wps path: %s", path.c_str());
                http.get(WebPowerSwitch_Request, WebPowerSwitch_Response, WebPowerSwitch_Headers);
                if (WebPowerSwitch_Response.status != 200)
                {
                    ppublish("actuator: %s WPS failed: %d", actuator->name, WebPowerSwitch_Response.status);
                    LogActuator.warn(" Response Status: %d", WebPowerSwitch_Response.status);
                    LogActuator.warn(" Response Body: %s", WebPowerSwitch_Response.body.c_str());
                }
                else
                {
                    LogActuator.trace(" Response Status: %d", WebPowerSwitch_Response.status);
                    actuator->state = FALSE;
                    actuator->timer_last = millis();
                }
            }
            else
            {
                digitalWrite(actuator->pin, LOW);
                actuator->state = FALSE;
                actuator->timer_last = millis();
            }
        }
    }
}

// schedule everything to be turned off
void all_off()
{
    actuate(&chiller.fan, FALSE);

    //turn off the pumps immediately
    control_F1.mode = AUTO_MODE_OFF;
    actuate(&control_F1.chiller, FALSE, TRUE);

    control_F2.mode = AUTO_MODE_OFF;
    actuate(&control_F2.chiller, FALSE, TRUE);

    chiller.mode = AUTO_MODE_OFF;
    // update chiller immediately
    update_chiller();
}

void read_temperatures()
{
    Log.trace("Refreshing temperatures");
    own.reset();
    own.skip();
    own.write(0x44);
    own.reset();
    // schedule a reading from ds sensors
    Log.trace("scheduling ds read");
    timer.setTimeout(DS_TEMP_SENSOR_CONVERT_DURATION, read_ds_temperatures);

    byte i = 0;

    Log.trace("Reading thermistor temperatures");
    for (i = 0; i < THERMISTOR_COUNT; i++)
    {
        thermistors[i].tempF = convertTempCtoF(readTempC(&thermistors[i]));
    }
    Log.trace("Done reading thermistor temperatures");
}

void read_ds_temperatures()
{
    Log.trace("Reading ds temperatures");

    byte i = 0;
    byte present_count = 0;
    float therm = INVALID_TEMPERATURE;
    bool done_converting = FALSE;

    own.reset();
    for (i = 0; i < DS_SENSOR_COUNT; i++)
    {
        therm = INVALID_TEMPERATURE;
        if (ds_temp_sensor[i].present)
        {
            present_count++;
            own.reset();
            own.select(ds_temp_sensor[i].addr);
            if (own.read())
            {
                // if at least one comes back, indicate that we are done converting
                done_converting = TRUE;
                therm = readTempC(&ds_temp_sensor[i]);
                ds_temp_sensor[i].last_tempF = therm;
                if (therm != INVALID_TEMPERATURE)
                {
                    ds_temp_sensor[i].tempF = convertTempCtoF(therm);
                    ds_temp_sensor[i].last_valid_read = millis();
                }
            }
            else
            {
                ds_temp_sensor[i].last_tempF = INVALID_TEMPERATURE;
            }
            // if we didn't get a good read, and it's been more than
            //  DS_TEMP_GRACE_PERIOD millis, note it
            // but leave it.. it might come back!
            if (ds_temp_sensor[i].last_tempF == INVALID_TEMPERATURE && ds_temp_sensor[i].tempF != INVALID_TEMPERATURE && (ds_temp_sensor[i].last_valid_read + DS_TEMP_GRACE_PERIOD) < millis())
            {
                ds_temp_sensor[i].tempF = INVALID_TEMPERATURE;
                Log.warn(" %s in invalid for too long", ds_temp_sensor[i].name);
                ppublish(" %s in invalid for too long", ds_temp_sensor[i].name);
            }
        }
    }
    // if there are none found, don't keep hammering the network
    if (present_count == 0)
    {
        Log.warn("No DS Temperatures Present!");
        ppublish("No DS Temperatures Present!");
    }
    else if (!done_converting)
    {
        timer.setTimeout(DS_TEMP_SENSOR_CONVERT_DURATION, read_ds_temperatures);
    }
}

#define TEMPSAMPLES 10
#define SERIESRESISTOR 10000
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
#define TEMPCOEFFICIENT 3950

float readTempC(Thermistor *thermistor)
{
    byte i;
    float average = 0;

    // take N samples in a row, with a slight delay
    for (i = 0; i < TEMPSAMPLES; i++)
    {
        average += analogRead(thermistor->pin);
        delay(10);
    }
    average /= TEMPSAMPLES;

#ifdef THERM_DEBUG
    Log.trace("Average analog reading %f", average);
#endif

    // convert the value to resistance
    average = 4095 / average - 1;
    average = SERIESRESISTOR / average;
#ifdef THERM_DEBUG
    Log.trace("Thermistor resistance %f", average);
#endif

    float steinhart;
    steinhart = average / THERMISTORNOMINAL;          // (R/Ro)
    steinhart = log(steinhart);                       // ln(R/Ro)
    steinhart /= TEMPCOEFFICIENT;                     // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                      // Invert
    steinhart -= 273.15;                              // convert to C

#ifdef THERM_DEBUG
    Log.trace("Temperature %f *C", steinhart, steinhartf);
#endif

    return steinhart;
}

float readTempC(DSTempSensor *dstemp)
{
    byte i;
    uint8_t data[12];
    float celsius;
    int16_t raw;

    own.reset();
    own.select(dstemp->addr);
    own.write(0xBE);

    for (i = 0; i < 9; i++)
    {
        data[i] = own.read();
    }

    uint8_t crc = OneWire::crc8(data, 8);
    if (crc != data[8])
    {
        Log.warn(" invalid crc: %d != %d", crc, data[8]);
        return INVALID_TEMPERATURE;
    }

    raw = (data[1] << 8) | data[0];

    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
    {
        raw = raw & ~7; // 9 bit resolution, 93.75 ms
    }
    else if (cfg == 0x20)
    {
        raw = raw & ~3; // 10 bit res, 187.5 ms
    }
    else if (cfg == 0x40)
    {
        raw = raw & ~1; // 11 bit res, 375 ms
    }

    //Log.trace(" %s => data[0]: %d ; data[1]: %d => raw: %d", dstemp->name, data[0], data[1], raw);

    celsius = (float)raw / 16.0;

    //Log.trace(" %s => tempC: %f", dstemp->name, celsius);

    return celsius;
}

float convertTempCtoF(float tempC)
{
    return tempC * 1.8 + 32.0;
}

void button_onPress(Button *button)
{
    //Log.trace("%s was clicked.", button->name);
}

void button_onRelease(Button *button)
{
    //Log.trace("%s was clicked.", button->name);
}

void button_onLongClick(Button *button)
{
    //Log.trace("%s was clicked.", button->name);
}

//TODO: fix button handling
void button_onClick(Button *button)
{
    char buffer[10];
    memset(buffer, 0, sizeof(buffer));
    Log.info("%s was clicked.", button->name);
    if (strcmp("Right", button->name) == 0)
    {
        if (chiller.heater->mode == AUTO_MODE_PID)
        {
            chiller.heater->mode = AUTO_MODE_ON;
        }
        else if (chiller.heater->mode == AUTO_MODE_OFF)
        {
            chiller.heater->mode = AUTO_MODE_PID;
        }
        else
        {
            chiller.heater->mode = AUTO_MODE_OFF;
        }
        mode_as_string(chiller.heater->mode, buffer, sizeof(buffer));
        Log.info("Setting heater to %s", buffer);
    }
    else if (strcmp("Up", button->name) == 0)
    {
        scanOWN();
    }
    else if (strcmp("Down", button->name) == 0)
    {
        if (chiller.mode == AUTO_MODE_AUTO)
        {
            chiller.mode = AUTO_MODE_ON;
        }
        else if (chiller.mode == AUTO_MODE_OFF)
        {
            chiller.mode = AUTO_MODE_AUTO;
        }
        else
        {
            chiller.mode = AUTO_MODE_OFF;
        }
        mode_as_string(chiller.mode, buffer, sizeof(buffer));
        Log.info(" setting %s to %s", chiller.name, buffer);
        update_chiller();
    }
    else if (strcmp("Left", button->name) == 0)
    {
        if (fermenters[F_FERMENTER_1].control->mode == AUTO_MODE_PID)
        {
            fermenters[F_FERMENTER_1].control->mode = AUTO_MODE_ON;
        }
        else if (fermenters[F_FERMENTER_1].control->mode == AUTO_MODE_OFF)
        {
            fermenters[F_FERMENTER_1].control->mode = AUTO_MODE_PID;
        }
        else
        {
            fermenters[F_FERMENTER_1].control->mode = AUTO_MODE_OFF;
        }
        mode_as_string(fermenters[F_FERMENTER_1].control->mode, buffer, sizeof(buffer));
        Log.info("Setting %s to %s", fermenters[F_FERMENTER_1].name, buffer);
    }
    else if (strcmp("Sel", button->name) == 0)
    {
        if (fermenters[F_FERMENTER_2].control->mode == AUTO_MODE_PID)
        {
            fermenters[F_FERMENTER_2].control->mode = AUTO_MODE_ON;
        }
        else if (fermenters[F_FERMENTER_2].control->mode == AUTO_MODE_OFF)
        {
            fermenters[F_FERMENTER_2].control->mode = AUTO_MODE_PID;
        }
        else
        {
            fermenters[F_FERMENTER_2].control->mode = AUTO_MODE_OFF;
        }
        mode_as_string(fermenters[F_FERMENTER_2].control->mode, buffer, sizeof(buffer));
        Log.info("Setting %s to %s", fermenters[F_FERMENTER_2].name, buffer);
    }
    else if (strcmp("Off", button->name) == 0)
    {
        Log.info("Turning everything off.");
        all_off();
    }
    else
    {
        Log.warn("%s was clicked and not handled", button->name);
    }
}

bool isBlynkConnected = FALSE;
bool blynkFirstRun = TRUE;
BLYNK_CONNECTED()
{
    // on initial connection, sync all buttons
    //  this makes Blynk in charge!
    if (!isBlynkConnected)
    {
        isBlynkConnected = TRUE;
        Blynk.syncAll();
    }
}
BLYNK_WRITE(V0)
{
    if (blynkFirstRun)
    {
        Log.info("blynk -> ignoring off (first run)");
        ppublish("blynk -> ignoring off (first run)");
        blynkFirstRun = FALSE;
    }
    else
    {
        all_off();
        Log.info("blynk -> turning everything off.");
        ppublish("blynk -> turning everything off.");
    }
}

byte mode_as_menu(byte mode)
{
    switch(mode)
    {
        case CONTROL_MODE_OFF: return MENU_OFF;
        case CONTROL_MODE_ON_CHILL: return MENU_ON_CHILL;
        case CONTROL_MODE_ON_HEAT: return MENU_ON_HEAT;
        case CONTROL_MODE_AUTO: return MENU_AUTO;
        case CONTROL_MODE_PID: return MENU_PID;
        case CONTROL_MODE_PID_CHILL: return MENU_PID_CHILL;
        case CONTROL_MODE_PID_HEAT: return MENU_PID_HEAT;
        case CONTROL_MODE_AUTO_CHILL: return MENU_AUTO_CHILL;
        case CONTROL_MODE_AUTO_HEAT: return MENU_AUTO_HEAT;
        default: return MENU_OFF;
    }
}

byte menu_as_mode(byte menu)
{
    switch (menu)
    {
        case MENU_OFF: return CONTROL_MODE_OFF;
        case MENU_ON_CHILL: return CONTROL_MODE_ON_CHILL;
        case MENU_ON_HEAT: return CONTROL_MODE_ON_HEAT;
        case MENU_PID: return CONTROL_MODE_PID;
        case MENU_AUTO: return CONTROL_MODE_AUTO;
        case MENU_PID_CHILL: return CONTROL_MODE_PID_CHILL;
        case MENU_PID_HEAT: return CONTROL_MODE_PID_HEAT;
        case MENU_AUTO_CHILL: return CONTROL_MODE_AUTO_CHILL;
        case MENU_AUTO_HEAT: return CONTROL_MODE_AUTO_HEAT;
        default: return CONTROL_MODE_OFF;
    }
}

BLYNK_WRITE(V5)
{
    int y = param.asInt();
    char buf[5];
    fermenters[F_FERMENTER_1].control->mode = menu_as_mode(y);
    mode_as_string(fermenters[F_FERMENTER_1].control->mode, buf, 5);
    Log.info("blynk -> Setting %s Mode to %s (%d->%d)", fermenters[F_FERMENTER_1].name, buf, y, fermenters[F_FERMENTER_1].control->mode);
    ppublish("blynk -> Setting %s Mode to %s (%d->%d)", fermenters[F_FERMENTER_1].name, buf, y, fermenters[F_FERMENTER_1].control->mode);
}
BLYNK_WRITE(V6)
{
    int y = param.asInt();
    char buf[5];
    fermenters[F_FERMENTER_2].control->mode = menu_as_mode(y);
    mode_as_string(fermenters[F_FERMENTER_2].control->mode, buf, 5);
    Log.info("blynk -> Setting %s Mode to %s (%d->%d)", fermenters[F_FERMENTER_2].name, buf, y, fermenters[F_FERMENTER_2].control->mode);
    ppublish("blynk -> Setting %s Mode to %s (%d->%d)", fermenters[F_FERMENTER_2].name, buf, y, fermenters[F_FERMENTER_2].control->mode);
}
BLYNK_WRITE(V7)
{
    int y = param.asInt();
    Log.info("blynk -> Setting %s Target to %d", fermenters[F_FERMENTER_1].name, y);
    ppublish("blynk -> Setting %s Target to %d", fermenters[F_FERMENTER_1].name, y);
    fermenters[F_FERMENTER_1].control->target = y;
}
BLYNK_WRITE(V8)
{
    int y = param.asInt();
    Log.info("blynk -> Setting %s Target to %d", fermenters[F_FERMENTER_2].name, y);
    ppublish("blynk -> Setting %s Target to %d", fermenters[F_FERMENTER_2].name, y);
    fermenters[F_FERMENTER_2].control->target = y;
}
BLYNK_WRITE(V13)
{
    int y = param.asInt();
    char buf[5];
    chiller.mode = menu_as_mode(y);
    mode_as_string(chiller.mode, buf, 5);
    Log.info("blynk -> Setting %s Mode to %s", chiller.name, buf);
    ppublish("blynk -> Setting %s Mode to %s", chiller.name, buf);
    update_chiller();
}
BLYNK_WRITE(V30)
{
    rescanOWN = true;
    Log.info("blynk -> scheduling OWN rescan");
    ppublish("blynk -> scheduling OWN rescan");
}


void ppublish(String message, ...)
{
    char buffer[50];
    va_list args;
    va_start(args, message);
    vsnprintf(buffer, 50, message.c_str(), args);
    Particle.publish("chronicle", buffer, PRIVATE);

    va_end(args);
}
