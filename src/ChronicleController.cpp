#include "config.h"
#include "keys.h"
#include "lib/blynk.h"
//#define BLYNK_PRINT Serial

Adafruit_MCP23017 mcp;
byte achState = LOW;
Adafruit_SSD1306 display;
OneWire own(OWNPIN);
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);
Adafruit_MQTT_Publish aio_photon_temp_9e = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/photon-temp-9e");

SerialLogHandler logHandler(LOG_LEVEL_ALL);

char buf[10];

unsigned long lastTemperatureTime = 0;
const long temperatureInterval = 5000;
const int dsTempConvertTime = 250;
bool dsTempIsConverting = FALSE;

const byte DSCOUNT = 5;
DSTempSensor dsTemp[DSCOUNT] = {
    {
        "04-73", // Fermenter 1
        {0x28, 0xFF, 0x93, 0x76, 0x71, 0x16, 0x4, 0x73},
        1, NULL,
        0, FALSE
    },
    {
        "05-9E", // Fermenter 2
        {0x28, 0xFF, 0x19, 0xE7, 0x70, 0x16, 0x5, 0x9E},
        2, &aio_photon_temp_9e,
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

const byte FERMENTERCOUNT = 2;
Fermenter fermenters[FERMENTERCOUNT] =
{
    {
        "F1",
        &dsTemp[0],
        65,
        MENU_OFF
    },
    {
        "F2",
        &dsTemp[1],
        65,
        MENU_OFF
    }
};

const byte THERMISTORCOUNT = 1;
Thermistor thermistors[THERMISTORCOUNT] = {
    { "A/C TStat", A2, 0, 4, NULL }
};

Button buttons[BUTTON_COUNT] = {
    { "Left", 13, &mcp },
    { "Right", 14, &mcp },
    { "Up", 15, &mcp },
    { "Down", 4, &mcp },
    { "Sel", 3, &mcp },
    { "Off", 2, &mcp }
};

// if chronicle.therm.getReading() > targetTemp - autoThesholdHigh : chronicle.cool.on();
// if chronicle.therm.getReading() > targetTemp - autoThesholdLow : chronicle.warm.on();

void setup() {
    Serial.begin(9600);
    delay(2000); // allow time to connect serial monitor

    Log.info("System started");
    Log.info("Device ID: %s", (const char*)System.deviceID());
    Log.info("System version: %s", (const char*)System.version());
    Log.info("App version: %s", (const char*)APP_VERSION);

    Log.info("setting up blynk");
    Blynk.begin(BLYNK_KEY);

    Log.info("connecting up AIO");
    int8_t aio_state = mqtt.connect();
    if ( aio_state != 0 )
    {
        Log.warn("Unable to connect to aio: %s", mqtt.connectErrorString(aio_state));
    }


    mcp.begin();
    mcp.pinMode(ACHPIN, OUTPUT);
    mcp.digitalWrite(ACHPIN, LOW);
    pinMode(ACTPIN, INPUT);

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
    snprintf(buf, sizeof(buf), "Ready %s", (const char*)APP_VERSION);
    display.print(buf);
    display.display();
/*
    **********
    A 76.0 02
    1 81.4 OF
    2 71.2 OF
    C 71.1 OF
    **********
*/

    Log.info("Setting up buttons");

    setup_buttons(buttons);
    delay(500);

    Log.info("Setting up OWN");
    scanOWN();
    delay(500);
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

    delay(500);
}

void loop()
{
    float therm;
    byte i = 0;
    unsigned long now = millis();
    char tbuf[5];

    Blynk.run();

    check_buttons(buttons);

    mcp.digitalWrite(ACHPIN, achState);

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
        for ( i = 0 ; i < THERMISTORCOUNT ; i ++ )
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
            snprintf(buf, sizeof(buf), "H %2.1f %s", thermistors[0].tempF, APP_VERSION);
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
                Log.info("Mode %d", fermenters[0].mode);
            }
            Log.info("setting 0 to %s", tbuf);
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

        Blynk.virtualWrite(3, achState);
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
                        if ( dsTemp[i].aioFeed )
                        {
                            Log.info(" updating aio feed");
                            dsTemp[i].aioFeed->publish(therm);
                        }
                    }
                }
                else
                {
                    Log.info(" %s -> not ready yet :(", buf);
                }
            }
        }
    }
}

void resetOWN()
{
    byte i = 0;
    for ( i = 0 ; i < DSCOUNT ; i ++ )
    {
        dsTemp[i].tempF = 0;
        dsTemp[i].present = FALSE;
    }
}

void scanOWN()
{
    uint8_t addr[8];
    char buf[30];
    byte i = 0;

    for ( i = 0 ; i < DSCOUNT ; i ++ )
    {
        dsTemp[i].present = FALSE;
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
        sprintf(buf, "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
        Log.info("Found: %s", buf);
        for ( i = 0 ; i < DSCOUNT ; i ++ )
        {
            uint8_t addrCmp = memcmp(addr, dsTemp[i].addr, 8);
            if ( addrCmp == 0 )
            {
                Log.info(" %s at index %d", dsTemp[i].name, i);
                dsTemp[i].present = TRUE;
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
        Log.info(" invalid crc: %d != %d", crc, data[8]);
        return -123;
    }

    raw = (data[1] << 8) | data[0];

    celsius = (float)raw / 16.0;

    return celsius;
}

float convertTempCtoF(float tempC)
{
    return tempC * 1.8 + 32.0;
}

void button_onPress(Button* button)
{

}

void button_onRelease(Button* button)
{

}

void button_onLongClick(Button* button)
{

}

void button_onClick(Button* button)
{
    if ( strcmp("Sel", button->name) == 0 )
    {
        Log.info("'Sel' was clicked.");
        if ( achState == HIGH )
        {
            achState = LOW;
        }
        else
        {
            achState = HIGH;
        }
    }
    else if ( strcmp("Off", button->name) == 0 )
    {
        Log.info("'Off' was clicked.");
        scanOWN();
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

BLYNK_WRITE(V5)
{
    int y = param.asInt();
    String name;
    switch (y)
    {
        case MENU_ON : name = "ON";
        case MENU_OFF : name = "OFF";
        case MENU_AUTO : name = "AUTO";
    }
    fermenters[0].mode = y;
    Log.info("Setting F1 Mode to %s", name.c_str());
}
BLYNK_WRITE(V6)
{
    int y = param.asInt();
    String name;
    switch (y)
    {
        case MENU_ON : name = "ON";
        case MENU_OFF : name = "OFF";
        case MENU_AUTO : name = "AUTO";
    }
    fermenters[1].mode = y;
    Log.info("Setting F2 Mode to %s", name.c_str());
}
BLYNK_WRITE(V7)
{
    int y = param.asInt();
    Log.info("Setting F1 Target to %d", y);
    fermenters[0].targetTemp = y;
}
BLYNK_WRITE(V8)
{
    int y = param.asInt();
    Log.info("Setting F2 Target to %d", y);
    fermenters[1].targetTemp = y;
}
BLYNK_WRITE(V3)
{
    int y = param.asInt();
    Log.info("Setting Heater to %d", y);
    achState = y;
}
