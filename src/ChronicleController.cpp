#include "config.h"
#include "keys.h"
#include "lib/blynk.h"
//#define BLYNK_PRINT Serial

//#define AIO_ENABLE

Adafruit_MCP23017 mcp;
byte achState = LOW;
Adafruit_SSD1306 display;
OneWire own(OWNPIN);
HttpClient http;

http_header_t WebPowerSwitch_Headers[] = {
    //  { "Content-Type", "application/json" },
    //  { "Accept" , "application/json" },
    { "Accept" , "*/*"},
    { "Authorization", WEBPOWERSWITCH_AUTH },
    { NULL, NULL } // NOTE: Always terminate headers will NULL
};
http_request_t WebPowerSwitch_Request;
http_response_t WebPowerSwitch_Response;
const char WebPowerSwitch_BaseUrl[] = "/outlet?";

#ifdef AIO_ENABLE
    TCPClient TheClient;
    Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);
    Adafruit_MQTT_Publish aio_pid_setpoint = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pid-setpoint");
    Adafruit_MQTT_Publish aio_pid_current = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pid-current");
    Adafruit_MQTT_Publish aio_pid_output = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pid-output");
    Adafruit_MQTT_Publish aio_pid_error = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pid-error");
    Adafruit_MQTT_Publish aio_pid_state = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pid-state");
#endif

SerialLogHandler logHandler(LOG_LEVEL_ALL);

const float INVALID_TEMPERATURE = -123;
const byte DS_SENSOR_COUNT = 5;
const byte DS_FERMENTER_1 = 0;
const byte DS_FERMENTER_2 = 1;
const byte DS_AMBIENT = 2;
const byte DS_CHILLER = 3;
DSTempSensor ds_temp_sensor[DS_SENSOR_COUNT] = {
    {
        "04-73", // Fermenter 1
        {0x28, 0xFF, 0x93, 0x76, 0x71, 0x16, 0x4, 0x73},
        1, NULL,
        0, FALSE
    },
    {
        "05-9E", // Fermenter 2
        {0x28, 0xFF, 0x19, 0xE7, 0x70, 0x16, 0x5, 0x9E},
        2, NULL,
        0, FALSE
    },
    {
        "05-AD", // Ambient
        {0x28, 0xFF, 0xEF, 0xE1, 0x70, 0x16, 0x5, 0xAD},
        9, NULL,
        0, FALSE
    },
    {
        "05-C9", // Chiller
        {0x28, 0xFF, 0x2A, 0xEA, 0x70, 0x16, 0x5, 0xC9},
        10, NULL,
        0, FALSE
    }
};

const byte THERMISTOR_COUNT = 1;
const byte THERM_HEATER = 0;
const byte THERM_HEATER_PIN = A2;
const byte BLYNK_HEATER_VPIN = 4;
Thermistor thermistors[THERMISTOR_COUNT] = {
    { "A/C TStat", THERM_HEATER_PIN, INVALID_TEMPERATURE, BLYNK_HEATER_VPIN, NULL }
};

const byte WPS_F1_PUMP_SOCKET = 1;
TemperatureControl control_F1 = {
    "F1-Ctrl",
    &ds_temp_sensor[DS_FERMENTER_1],    // ds temp sensor
    NULL,                               // thermistor
    AUTO_MODE_OFF,                      // mode
    { "F1-Act", FALSE, TRUE, WPS_F1_PUMP_SOCKET, NULL, FALSE, 0 }, // actuator - wps
    INVALID_TEMPERATURE,                // tempF
    PID(1),                              // PID object - will initialize later
    65, 0, 0, 0,                        // setpoint, input, output, error
    10000, 60000, 60000, 0, 0,         // min, max, window, window_start, window_end
    50, 0.3, 500                        // Kp, Ki, Kd
};

const byte WPS_F2_PUMP_SOCKET = 2;
TemperatureControl control_F2 = {
    "F2-Ctrl",
    &ds_temp_sensor[DS_FERMENTER_2],    // ds temp sensor
    NULL,                               // thermistor
    AUTO_MODE_OFF,                      // mode
    { "F2-Act", FALSE, TRUE, WPS_F2_PUMP_SOCKET, NULL, FALSE, 0 }, // actuator - wps
    INVALID_TEMPERATURE,                // tempF
    PID(1),                              // PID object - will initialize later
    65, 0, 0, 0,                        // setpoint, input, output, error
    10000, 60000, 60000, 0, 0,         // min, max, window, window_start, window_end
    50, 0.3, 500                        // Kp, Ki, Kd
};

TemperatureControl control_Heater = {
    "H-Ctrl",
    NULL,                               // ds temp sensor
    &thermistors[THERM_HEATER],         // thermistor
    AUTO_MODE_OFF,                      // mode
    { "H-Act", TRUE, FALSE, 8, NULL, FALSE, 0 }, // actuator - mcp
    INVALID_TEMPERATURE,                // tempF
    PID(1),                              // PID object - will initialize later
    65, 0, 0, 0,                        // setpoint, input, output, error
    0, 1000, 1000, 0, 0,                // min, max, window, window_start, window_end
    100, 0.15, 1000                     // Kp, Ki, Kd
};

const byte FERMENTER_COUNT = 2;
const byte F_FERMENTER_1 = 0;
const byte F_FERMENTER_2 = 1;
Fermenter fermenters[FERMENTER_COUNT] = {
    { "F1", &control_F1 },
    { "F1", &control_F2 }
};

const byte WPS_CHILLER_FAN_SOCKET = 3;
const int CHILLER_FAN_POST_TIME = 60000;
Chiller chiller = {
    "C-Ctrl",
    &control_Heater,
    { "C-Fan", FALSE, TRUE, WPS_CHILLER_FAN_SOCKET, NULL, FALSE, 0 }, // actuator - wps
    &ds_temp_sensor[DS_CHILLER],
    AUTO_MODE_OFF, FALSE,           // mode, state
    20,                             // target
    10, 5, 5,                       // normal: target offset, high threshold, low threshold
    20, 20, 10,                     // high differential: target offset, high threshold, low threshold
    18,                             // min temperature
    5*60000,                        // min on time - 5 minutes
    5*60000,                        // min off time
    70,                             // control_set_temperature
    0                               // timer_last
};

Button buttons[BUTTON_COUNT] = {
    { "Left", 13, &mcp },
    { "Right", 14, &mcp },
    { "Up", 15, &mcp },
    { "Down", 4, &mcp },
    { "Sel", 3, &mcp },
    { "Off", 2, &mcp }
};

//Timer //Timer_Check_Buttons(48, timer_check_buttons);
Timer Timer_Read_Temperatures(952, read_temperatures);
// this launches on-demand after a temperature read is scheduled
Timer Timer_Read_DS_Temperatures(247, read_ds_temperatures, true);
Timer Timer_Update_Pids(9983, update_pids);
//Timer Timer_Run_Controls(10, run_controls);
Timer Timer_Check_Memory(9345, check_memory);

// this is also called whenever a chiller-affecting setting is changed
//Timer Timer_Update_Chiller(59876, //update_chiller);
//Timer Timer_Chiller_Fan_Off(CHILLER_FAN_POST_TIME, chiller_fan_off, true);
// after the chiller is turned off, keep checking heater until it gets below
// {{control_set_temperature}}, then mark chiller as off, and start fan-off
//Timer Timer_Chiller_Check_Heater(1000, chiller_check_heater, true);

char display_buffer[10];

void setup() {
    Serial.begin(115200);
    delay(2000); // allow time to connect serial monitor

    Log.info("System started");
    Log.info("Device ID: %s", (const char*)System.deviceID());
    Log.info("System version: %s", (const char*)System.version());
    Log.info("App version: %s", (const char*)APP_VERSION);

    Log.info("setting up Heater pin");
    mcp.begin();
    mcp.pinMode(control_Heater.actuator.pin, OUTPUT);
    mcp.digitalWrite(control_Heater.actuator.pin, LOW);
    pinMode(control_Heater.thermistor->pin, INPUT);

    WebPowerSwitch_Request.ip = {192,168,2,9};
    WebPowerSwitch_Request.port = 80;

    Log.info("Turning everything off");
    all_off();
    //Timer_Run_Controls.start();

    Log.info("Setting up OWN");
    scanOWN();
    // set resolution for all ds temp sensors
    own.reset();
    own.skip();
    own.write(0x4E);         // Write scratchpad
    own.write(0);            // TL
    own.write(0);            // TH
    own.write(0x3F);         // 10-bit resolution
    own.write(0x48);         // Copy Scratchpad
    own.write(0x44);         // start conversion
    own.reset();

    Log.info("Setting up temp timer");
    Timer_Read_Temperatures.start();

    Log.info("Setting up PIDs");
    setup_pids();

    //Log.info("Setting up buttons");
    //setup_buttons(buttons, BUTTON_COUNT);
    //Timer_Check_Buttons.start();

    //Log.info("setting up blynk");
    //Blynk.begin(BLYNK_KEY);

    #ifdef AIO_ENABLE
        Log.info("connecting to AIO");
        int8_t aio_state = mqtt.connect();
        if ( aio_state != 0 )
        {
            Log.warn("Unable to connect to aio: %s", mqtt.connectErrorString(aio_state));
        }
    #endif

    // initialize with the I2C addr 0x3C (for the diy display)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    // Show the Adafruit splashscreen
    display.display();
    delay(500);

    Log.info("Setting up display");
    // Clear the buffer.
    display.clearDisplay();

    // text display tests
    display.setTextSize(2);
    display.setTextColor(WHITE, 0);
    display.setCursor(0,0);
    snprintf(display_buffer, sizeof(display_buffer), "Ready %s", (const char*)APP_VERSION);
    display.print(display_buffer);
    display.display();

    Log.info("Free memory: %ld", System.freeMemory());
    Timer_Check_Memory.start();

/*
    **********
    A 76.0 02 -- prod
    H 80.1 80 -- dev
    1 81.4 OF
    2 71.2 OF
    C 71.1 OF
    **********
*/
}

void loop()
{
    //Blynk.run();

    /*
    update_display();
    update_aio();
    update_blynk();
    */

/*

    if ( now - lastTemperatureTime >= temperatureInterval )
    {
        lastTemperatureTime = now;

        Log.info("Refreshing temperatures");
        own.reset();
        own.skip();
        own.write(0x44);
        own.reset();
        dsTempIsConverting = TRUE;

        Log.info("Reading thermistor temperatures");
        for ( i = 0 ; i < THERMISTOR_COUNT ; i ++ )
        {
            therm = readTempC(&thermistors[i]);
            therm = convertTempCtoF(therm);
            thermistors[i].tempF = therm;
            if ( thermistors[i].blynkPin >= 0 && therm > 0 )
            {
                Blynk.virtualWrite(thermistors[i].blynkPin, therm);
            }

            //Log.info(" %s -> %2.2f", thermistors[i].name, therm);
        }

        if ( now > temperatureInterval )
        {
            memset(buf, 0, sizeof(buf));
            //snprintf(buf, sizeof(buf), "A %2.1f %s", dsTemp[2].tempF, APP_VERSION);

            memset(tbuf, 0, sizeof(tbuf));
            if ( pid_Heater_enabled )
            {
                snprintf(tbuf, sizeof(tbuf), "%2f", pid_Heater_Setpoint);
            }
            else if ( achState )
            {
                snprintf(tbuf, sizeof(tbuf), "ON ");
            }
            else
            {
                snprintf(tbuf, sizeof(tbuf), "OFF");
            }
            Log.info("loop -> Heater is %s (%s)", tbuf, achState ? "ON" : "OFF");
            snprintf(buf, sizeof(buf), "H %2.0f %s %s", thermistors[0].tempF, tbuf, APP_VERSION);
            displayLine(0, buf, FALSE);

            memset(tbuf, 0, sizeof(tbuf));
            if ( fermenters[0].mode == 1 )
            {
                snprintf(tbuf, sizeof(tbuf), "ON ");
            }
            else if ( fermenters[0].mode == 2 )
            {
                snprintf(tbuf, sizeof(tbuf), "OFF");
            }
            else if ( fermenters[0].mode == 3 )
            {
                snprintf(tbuf, sizeof(tbuf), "%2d", fermenters[0].targetTemp);
            }
            else
            {
                Log.warn("Unknown mode %d for %s", fermenters[0].mode, fermenters[0].name);
            }
            Log.info("loop -> %s is %s", fermenters[0].name, tbuf);
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "1 %2.1f %s", dsTemp[0].tempF, tbuf);
            displayLine(1, buf, FALSE);

            memset(tbuf, 0, sizeof(tbuf));
            if ( fermenters[1].mode == 1 )
            {
                snprintf(tbuf, sizeof(tbuf), "ON ");
            }
            else if ( fermenters[1].mode == 2 )
            {
                snprintf(tbuf, sizeof(tbuf), "OFF");
            }
            else if ( fermenters[1].mode == 3 )
            {
                snprintf(tbuf, sizeof(tbuf), "%2d", fermenters[1].targetTemp);
            }
            else
            {
                Log.info("Mode %d", fermenters[1].mode);
            }
            Log.info("setting 1 to %s", tbuf);
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "2 %2.1f %s", dsTemp[1].tempF, tbuf);
            displayLine(2, buf, FALSE);

            memset(tbuf, 0, sizeof(tbuf));
            if ( achState )
            {
                snprintf(tbuf, sizeof(tbuf), "ON ");
            }
            else
            {
                snprintf(tbuf, sizeof(tbuf), "OFF");
            }

            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "C %2.1f %s", dsTemp[3].tempF, tbuf);
            displayLine(3, buf, FALSE);
        }

        if ( pid_Heater_enabled )
        {
            Blynk.virtualWrite(3, MENU_AUTO);
        }
        else if ( achState )
        {
            Blynk.virtualWrite(3, MENU_ON);
        }
        else
        {
            Blynk.virtualWrite(3, MENU_OFF);
        }
    }
    now = millis();
    if ( ( now - lastTemperatureTime >= dsTempConvertTime ) && dsTempIsConverting )
    {
        dsTempIsConverting = FALSE;
        Log.info("Reading ds temperatures");

        own.reset();
        for ( i = 0 ; i < DSCOUNT ; i ++ )
        {
            if ( dsTemp[i].present )
            {
                memset(buf, 0, sizeof(buf));
                sprintf(buf, "%02X-%02X", dsTemp[i].addr[6], dsTemp[i].addr[7]);

                own.reset();
                own.select(dsTemp[i].addr);
                if ( own.read() )
                {
                    therm = readTempC(&dsTemp[i]);
                    if ( therm > -123 ) // invalid crc, skip
                    {
                        therm = convertTempCtoF(therm);
                        //Log.info(" %s -> %2.2f", buf, therm);

                        dsTemp[i].tempF = therm;
                        if ( dsTemp[i].blynkPin >= 0 )
                        {
                            Blynk.virtualWrite(dsTemp[i].blynkPin, therm);
                        }
                        #ifdef AIO_ENABLE
                            if ( dsTemp[i].aioFeed )
                            {
                                if ( mqtt.connected() )
                                {
                                    Log.info(" updating aio feed");
                                    dsTemp[i].aioFeed->publish(therm);
                                }
                                else
                                {
                                    Log.warn(" mqtt disconnected");
                                }
                            }
                        #endif
                    }
                }
                else
                {
                    Log.info(" %s -> not ready yet :(", buf);
                }
            }
        }
    }
    */
}

void check_memory()
{
    Log.info("Free memory: %ld", System.freeMemory());
}

void update_display()
{

}

void update_aio()
{

}

void update_blynk()
{

}

void timer_check_buttons()
{
    check_buttons(buttons, BUTTON_COUNT);
}

void resetOWN()
{
    byte i = 0;
    for ( i = 0 ; i < DS_SENSOR_COUNT ; i ++ )
    {
        ds_temp_sensor[i].tempF = 0;
        ds_temp_sensor[i].present = FALSE;
    }
}

void scanOWN()
{
    uint8_t addr[8];
    byte i = 0;

    for ( i = 0 ; i < DS_SENSOR_COUNT ; i ++ )
    {
        ds_temp_sensor[i].present = FALSE;
    }

    Log.info("Searching OWN");
    if ( own.reset() == 1 )
    {
        Log.info("Network present");
    }
    else
    {
        Log.info("Network problem :(");
    }

    own.reset_search();
    // search own for sensors
    while(own.search(addr))
    {
        Log.info("Found: %02X-%02X", addr[6], addr[7]);
        for ( i = 0 ; i < DS_SENSOR_COUNT ; i ++ )
        {
            if ( memcmp(addr, ds_temp_sensor[i].addr, 8) == 0 )
            {
                Log.info(" %s at index %d", ds_temp_sensor[i].name, i);
                ds_temp_sensor[i].present = TRUE;
            }
        }
    }
    own.reset_search();
}

void displayLine(byte line, char *message, bool clear)
{
    char lcdLineBuf[LCDLINELENGTH];

    if ( clear )
    {
        display.setCursor(0, LCDLINEHEIGHT * line);
        memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
        snprintf(lcdLineBuf, LCDLINELENGTH, LCDBLANKLINE);
        display.print(lcdLineBuf);
        display.display();
    }

    display.setCursor(0, LCDLINEHEIGHT * line);
    memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
    snprintf(lcdLineBuf, LCDLINELENGTH, message);
    display.print(lcdLineBuf);
    display.display();
}

void setup_pids()
{
    control_F1.pid.init(&control_F1.input, &control_F1.output, &control_F1.target,
                        control_F1.Kp, control_F1.Ki, control_F1.Kd, PID::REVERSE);
    control_F1.pid.SetOutputLimits(0, control_F1.max); // we take care of the minimum ourselves
    control_F1.pid.SetMode(PID::AUTOMATIC);
    control_F1.pid.SetSampleTime(control_F1.window);

    control_F2.pid.init(&control_F2.input, &control_F2.output, &control_F2.target,
                        control_F2.Kp, control_F2.Ki, control_F2.Kd, PID::REVERSE);
    control_F2.pid.SetOutputLimits(0, control_F2.max);
    control_F2.pid.SetMode(PID::AUTOMATIC);
    control_F2.pid.SetSampleTime(control_F2.window);

    control_Heater.pid.init(&control_Heater.input, &control_Heater.output, &control_Heater.target,
                        control_Heater.Kp, control_Heater.Ki, control_Heater.Kd, PID::DIRECT);
    control_Heater.pid.SetOutputLimits(0, control_Heater.max); // we take care of the minimum ourselves
    control_Heater.pid.SetMode(PID::AUTOMATIC);
    control_Heater.pid.SetSampleTime(control_Heater.window);
}

void update_pids()
{
    Log.info("Updating PIDS");
    update_pid(&control_F1);
    update_pid(&control_F2);
    update_pid(&control_Heater);
    //update_chiller();

    //Timer_Run_Controls.start();
}

// calculate and update vars - every second
void update_pid(TemperatureControl *control)
{
    Log.info("Updating Control for %s", control->name);
    if ( control->dstempsensor != NULL )
    {
        control->tempF = control->dstempsensor->tempF;
    }
    else if ( control->thermistor != NULL )
    {
        control->tempF = control->thermistor->tempF;
    }
    else
    {
        Log.warn(" no temperature source!");
    }

    if ( control->mode == AUTO_MODE_PID )
    {
        control->input = control->tempF;
        control->error = control->target - control->input;
        if ( control->pid.Compute() )
        {
            // returns true when a new computation has been done
            // ie: new window
            control->window_start = millis();
            control->window_end = millis() + control->output;
            Log.info(" %s PID Output: %3.2f %3.2f %3.2f %3.2f %ld", control->name, control->target, control->input, control->error, control->output, control->window_end);
        }
    }
}

// this puppy is special
void update_chiller()
{
    Log.info("updating chiller");

    bool state = FALSE;

    if ( chiller.mode == AUTO_MODE_AUTO )
    {
        if ( chiller.dstempsensor->tempF > chiller.target )
        {
            state = FALSE;
        }
        else
        {
            state = TRUE;
        }
    }
    else if ( chiller.mode == AUTO_MODE_ON )
    {
        state = TRUE;
    }
    else if ( chiller.mode == AUTO_MODE_OFF )
    {
        state = FALSE;
    }
    else
    {
        Log.warn(" Unknown mode requested: %d", chiller.mode);
    }

    // if we decide the A/C should be on, turn on the heater PID
    if ( chiller.state != state )
    {
        chiller.state = state;
        chiller.timer_last = millis();
        if ( state )
        {
            // make sure fan isn't scheduled to turn off
            //Timer_Chiller_Check_Heater.stop();
            //Timer_Chiller_Fan_Off.stop();
            actuate(&chiller.fan, TRUE);

            chiller.heater->mode = AUTO_MODE_PID;
            chiller.heater->target = chiller.control_set_temperature + 20;
        }
        else
        {
            chiller.heater->mode = AUTO_MODE_OFF;
            //Timer_Chiller_Check_Heater.start();
        }
    }
}

void chiller_check_heater()
{
    Log.trace(" check chiller heater: %2.2f < %2d", chiller.heater->tempF, chiller.control_set_temperature);
    if ( chiller.state == FALSE )
    {
        if ( chiller.heater->tempF < ( chiller.control_set_temperature - 2 ) )
        {
            Log.info(" heater is back down below control_set_temperature, marking Chiller off");
            chiller.state = FALSE;
            chiller.timer_last = millis();
            // schedule the fan to turn off
            //Timer_Chiller_Fan_Off.start();
        }
        else
        {
            //Timer_Chiller_Check_Heater.start();
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
    if ( control->mode == AUTO_MODE_PID )
    {
        if ( millis() >= control->window_start && millis() <= control->window_end )
        {
            actuate(&control->actuator, TRUE);
        }
        else
        {
            actuate(&control->actuator, FALSE);
        }
    }
    else if ( control->mode == AUTO_MODE_ON )
    {
        actuate(&control->actuator, TRUE);
    }
    else if ( control->mode == AUTO_MODE_OFF )
    {
        actuate(&control->actuator, FALSE);
    }
    else
    {
        Log.warn(" Invalid control mode: %d", control->mode);
    }
}

// do the actual on/off
void actuate(Actuator *actuator, bool on)
{
    if ( on )
    {
        if ( ! actuator->state )
        {
            Log.trace("  actuator: turning %s ON", actuator->name);
            if ( actuator->isMcp )
            {
                actuator->mcp->digitalWrite(actuator->pin, HIGH);
            }
            else if ( actuator->isWebPowerSwitch )
            {
                String path = WebPowerSwitch_BaseUrl;
                path += actuator->pin;
                path += "=ON";
                WebPowerSwitch_Request.path = path.c_str();
                Log.trace("  actuator: wps path: %s", path.c_str());
                http.get(WebPowerSwitch_Request, WebPowerSwitch_Response, WebPowerSwitch_Headers);
                Log.trace(" Response Status: %d", WebPowerSwitch_Response.status);
                Log.trace(" Response Body: %s", WebPowerSwitch_Response.body.c_str());
            }
            else
            {
                digitalWrite(actuator->pin, HIGH);
            }
            actuator->state = TRUE;
            actuator->timer_last = millis();
        }
    }
    else
    {
        if ( actuator->state )
        {
            Log.trace("  actuator: turning %s OFF", actuator->name);
            if ( actuator->isMcp )
            {
                actuator->mcp->digitalWrite(actuator->pin, LOW);
            }
            else if ( actuator->isWebPowerSwitch )
            {
                String path = WebPowerSwitch_BaseUrl;
                path += actuator->pin;
                path += "=OFF";
                WebPowerSwitch_Request.path = path.c_str();
                Log.trace("  actuator: wps path: %s", path.c_str());
                http.get(WebPowerSwitch_Request, WebPowerSwitch_Response, WebPowerSwitch_Headers);
                Log.trace(" Response Status: %d", WebPowerSwitch_Response.status);
                Log.trace(" Response Body: %s", WebPowerSwitch_Response.body.c_str());
            }
            else
            {
                digitalWrite(actuator->pin, LOW);
            }
            actuator->state = FALSE;
            actuator->timer_last = millis();
        }
    }
}

// schedule everything to be turned off
void all_off()
{
    control_F1.mode = AUTO_MODE_OFF;
    control_F2.mode = AUTO_MODE_OFF;
    control_Heater.mode = AUTO_MODE_OFF;
    chiller.mode = AUTO_MODE_OFF;
}

void read_temperatures()
{
    Log.info("Refreshing temperatures");
    own.reset();
    own.skip();
    own.write(0x44);
    own.reset();
    // schedule a reading from ds sensors
    Log.trace("starting ds timer");
    Timer_Read_DS_Temperatures.start();

    byte i = 0;

    Log.info("Reading thermistor temperatures");
    for ( i = 0 ; i < THERMISTOR_COUNT ; i ++ )
    {
        thermistors[i].tempF = convertTempCtoF(readTempC(&thermistors[i]));
    }
    Log.trace("Done reading thermistor temperatures");

    // start up dependent timers
    if ( ! Timer_Update_Pids.isActive() )
    {
        Log.trace("starting pids timer");
        Timer_Update_Pids.start();
    }
    // if ( ! Timer_Update_Chiller.isActive() )
    // {
    //     Log.trace("starting chiller timer");
    //     Timer_Update_Chiller.start();
    // }
    Log.trace("Additional timers launched");
}

void read_ds_temperatures()
{
    char buffer[10] = "DS TEMP";
    displayLine(1, buffer, true);
    Log.info("Reading ds temperatures");

    byte i = 0;
    float therm = INVALID_TEMPERATURE;

    own.reset();
    for ( i = 0 ; i < DS_SENSOR_COUNT ; i ++ )
    {
        if ( ds_temp_sensor[i].present )
        {
            own.reset();
            own.select(ds_temp_sensor[i].addr);
            if ( own.read() )
            {
                therm = readTempC(&ds_temp_sensor[i]);
                if ( therm > INVALID_TEMPERATURE )
                {
                    ds_temp_sensor[i].tempF = convertTempCtoF(therm);
                }
            }
        }
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
    for ( i = 0 ; i < TEMPSAMPLES ; i++ )
    {
        average += analogRead(thermistor->pin);
        delay(10);
    }
    average /= TEMPSAMPLES;

    #ifdef THERM_DEBUG
        Log.info("Average analog reading %f", average);
    #endif

    // convert the value to resistance
    average = 4095 / average - 1;
    average = SERIESRESISTOR / average;
    #ifdef THERM_DEBUG
        Log.info("Thermistor resistance %f", average);
    #endif

    float steinhart;
    steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= TEMPCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C

    #ifdef THERM_DEBUG
        Log.info("Temperature %f *C", steinhart, steinhartf);
    #endif

    return steinhart;
}

float readTempC(DSTempSensor *dstemp)
{
    byte i;
    uint8_t data[12];
    float celsius;
    unsigned int raw;

    own.reset();
    own.select(dstemp->addr);
    own.write(0xBE);

    for ( i = 0 ; i < 9 ; i ++ )
    {
        data[i] = own.read();
    }

    uint8_t crc = OneWire::crc8(data, 8);
    if ( crc != data[8] ) {
        Log.warn(" invalid crc: %d != %d", crc, data[8]);
        return INVALID_TEMPERATURE;
    }

    raw = (data[1] << 8) | data[0];
    //Log.trace(" %s => data[0]: %d ; data[1]: %d => raw: %d", dstemp->name, data[0], data[1], raw);

    celsius = (float)raw / 16.0;

    //Log.trace(" %s => tempC: %f", dstemp->name, celsius);

    return celsius;
}

float convertTempCtoF(float tempC)
{
    return tempC * 1.8 + 32.0;
}

void button_onPress(Button* button)
{
    //Log.trace("%s was clicked.", button->name);
}

void button_onRelease(Button* button)
{
    //Log.trace("%s was clicked.", button->name);
}

void button_onLongClick(Button* button)
{
    //Log.trace("%s was clicked.", button->name);
}

void button_onClick(Button* button)
{
    Log.trace("%s was clicked.", button->name);
    if ( strcmp("Right", button->name) == 0 )
    {
        if ( chiller.heater->mode == AUTO_MODE_PID )
        {
            chiller.heater->mode = AUTO_MODE_ON;
        }
        else if ( chiller.heater->mode == AUTO_MODE_OFF )
        {
            chiller.heater->mode = AUTO_MODE_PID;
        }
        else
        {
            chiller.heater->mode = AUTO_MODE_OFF;
        }
    }
    else if ( strcmp("Up", button->name) == 0 )
    {
        scanOWN();
    }
    else if ( strcmp("Down", button->name) == 0 )
    {
        if ( chiller.mode == AUTO_MODE_AUTO )
        {
            chiller.mode = AUTO_MODE_ON;
        }
        else if ( chiller.mode == AUTO_MODE_OFF )
        {
            chiller.mode = AUTO_MODE_AUTO;
        }
        else
        {
            chiller.mode = AUTO_MODE_OFF;
        }
    }
    else if ( strcmp("Left", button->name) == 0 )
    {
        if ( fermenters[F_FERMENTER_1].control->mode == AUTO_MODE_PID )
        {
            fermenters[F_FERMENTER_1].control->mode = AUTO_MODE_ON;
        }
        else if ( fermenters[F_FERMENTER_1].control->mode == AUTO_MODE_OFF )
        {
            fermenters[F_FERMENTER_1].control->mode = AUTO_MODE_PID;
        }
        else
        {
            fermenters[F_FERMENTER_1].control->mode = AUTO_MODE_OFF;
        }
    }
    else if ( strcmp("Sel", button->name) == 0 )
    {
        if ( fermenters[F_FERMENTER_2].control->mode == AUTO_MODE_PID )
        {
            fermenters[F_FERMENTER_2].control->mode = AUTO_MODE_ON;
        }
        else if ( fermenters[F_FERMENTER_2].control->mode == AUTO_MODE_OFF )
        {
            fermenters[F_FERMENTER_2].control->mode = AUTO_MODE_PID;
        }
        else
        {
            fermenters[F_FERMENTER_2].control->mode = AUTO_MODE_OFF;
        }
    }
    else if ( strcmp("Off", button->name) == 0 )
    {
        all_off();
    }
    else
    {
        Log.info("%s was clicked.", button->name);
    }
}

bool isBlynkConnected = FALSE;
BLYNK_CONNECTED()
{
    if ( !isBlynkConnected )
    {
        isBlynkConnected = TRUE;
        Blynk.syncAll();
    }
}

BLYNK_WRITE(V3)
{
    int y = param.asInt();
    String name;
    switch (y)
    {
        case MENU_ON : name = "ON";
        case MENU_OFF : name = "OFF";
        case MENU_PID : name = "PID";
        case MENU_AUTO : name = "AUTO";
    }
    Log.error("Invalid menu item from Heater Control! %d", y);
    Log.info("blynk -> Setting Heater to %s", name.c_str());
}
BLYNK_WRITE(V5)
{
    int y = param.asInt();
    String name;
    switch (y)
    {
        case MENU_ON : name = "ON";
        case MENU_OFF : name = "OFF";
        case MENU_PID : name = "PID";
        case MENU_AUTO : name = "AUTO";
    }
    //fermenters[0].mode = y;
    Log.info("blynk -> Setting F1 Mode to %s", name.c_str());
}
BLYNK_WRITE(V6)
{
    int y = param.asInt();
    String name;
    switch (y)
    {
        case MENU_ON : name = "ON";
        case MENU_OFF : name = "OFF";
        case MENU_PID : name = "PID";
        case MENU_AUTO : name = "AUTO";
    }
    //fermenters[1].mode = y;
    Log.info("blynk -> Setting F2 Mode to %s", name.c_str());
}
BLYNK_WRITE(V7)
{
    int y = param.asInt();
    Log.info("blynk -> Setting F1 Target to %d", y);
    //fermenters[0].targetTemp = y;
}
BLYNK_WRITE(V8)
{
    int y = param.asInt();
    Log.info("blynk -> Setting F2 Target to %d", y);
    //fermenters[1].targetTemp = y;
}
BLYNK_WRITE(V11)
{
    int y = param.asInt();
    Log.info("blynk -> Setting Heater PID Target to %d", y);
}
