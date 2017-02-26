#include "config.h"

Adafruit_MCP23017 mcp;
byte achState = LOW;
Adafruit_SSD1306 display(OLED_RESET);
OneWire own(OWNPIN);
char lcdLineBuf[LCDLINELENGTH];

unsigned long previousTempMillis = 0;
unsigned long previousOwnMillis = 0;
const long tempInterval = 1000;
const long ownInterval = 3500;

const byte DSCOUNT = 6;

uint8_t dsAddrs[DSCOUNT][8] = {
    {0x28, 0x37, 0xD3, 0x60, 0x4, 0x0, 0x0, 0xC9},
    {0x28, 0xFF, 0x93, 0x76, 0x71, 0x16, 0x4, 0x73},
    {0x28, 0xFF, 0x19, 0xE7, 0x70, 0x16, 0x5, 0x9E},
    {0x28, 0xFF, 0xEF, 0xE1, 0x70, 0x16, 0x5, 0xAD},
    {0x28, 0xFF, 0x2A, 0xEA, 0x70, 0x16, 0x5, 0xC9},
    {0x28, 0xFF, 0x26, 0x5A, 0x71, 0x16, 0x4, 0x8C}
};

bool addrPresent[DSCOUNT] = {
    FALSE, FALSE, FALSE, FALSE, FALSE, FALSE
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

    mcp.begin();
    mcp.pinMode(ACHPIN, OUTPUT);
    mcp.digitalWrite(ACHPIN, LOW);
    pinMode(ACTPIN, INPUT);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the diy display)

    // Show the Adafruit splashscreen
    display.display();
    delay(500);

    debug("Setting up display");
    // Clear the buffer.
    display.clearDisplay();

    // text display tests
    display.setTextSize(2);
    display.setTextColor(WHITE, 0);
    display.setCursor(0,0);
    display.print("Ready v7");
    display.display();
    // text size 2: 4x10
    //             1234567890
    //display.print("  AP Cu Ta");
    //display.print("1 YY 65 60");
    //display.print("2 YY 65 60");
//    display.print("C N  65 40");
//    display.print("A  65 50% ");
    //display.setTextSize(1);
    // text size 1: 8x21
    //               123456789012345678901
    //display.print("Chiller     Y Y 65 40");
    //display.print("Ambient      65 50%  ");

/*
    Ready v7
    %2.1f ON

*/

    debug("Setting up buttons");

    setup_buttons(buttons);
    delay(500);

    debug("Setting up OWN");
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
    uint8_t *addr;
    char buf[10];
    float therm;
    unsigned long now = millis();
    check_buttons(buttons);

    if ( now - previousTempMillis >= tempInterval )
    {
        previousTempMillis = now;
        therm = readTempC(ACTPIN);
        therm = convertTempCtoF(therm);

        display.setCursor(0, LCDLINEHEIGHT * 1);
        memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
        snprintf(lcdLineBuf, LCDLINELENGTH, "%2.1f %d  ", therm, achState);
        display.print(lcdLineBuf);
        display.display();

        own.reset();
        own.skip();
        own.write(0x44);
        own.reset();
    }
    if ( now - previousOwnMillis >= ownInterval )
    {
        previousOwnMillis = now;

        own.reset();
        for ( byte i = 0 ; i < DSCOUNT ; i ++ )
        {
            if ( addrPresent[i] )
            {
                addr = dsAddrs[i];
                sprintf(buf, "%02X-%02X", addr[6], addr[7]);

                own.reset();
                own.select(addr);
                if ( own.read() )
                {
                    therm = readTempC(addr);
                    if ( therm > -123 ) // invalid crc, skip
                    {
                        therm = convertTempCtoF(therm);
                        debug(" %s -> %2.2f", buf, therm);
                    }
                }
                else
                {
                    debug(" %s -> not ready yet :(", buf);
                }
            }
        }
    }
}

void scanOWN()
{
    uint8_t addr[8];
    char buf[30];

    debug("Searching OWN");
    if ( own.reset() == 1 )
    {
        debug("Network present");
    }
    else
    {
        debug("Network problem :(");
    }

    own.reset_search();
    // search own for sensors
    while(own.search(addr))
    {
        sprintf(buf, "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
        debug("Found: %s", buf);
        for ( byte i = 0 ; i < DSCOUNT ; i ++ )
        {
            uint8_t addrCmp = memcmp(addr, dsAddrs[i], 8);
            debug(" addrCmp: %d", addrCmp);
            if ( addrCmp == 0 )
            {
                debug(" at index %d", i);
                addrPresent[i] = TRUE;
            }
        }
    }
    own.reset_search();
}

#define TEMPSAMPLES 10
#define SERIESRESISTOR 10000
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
#define TEMPCOEFFICIENT 3950

float readTempC(byte pin)
{
    byte i;
    float average = 0;

    // take N samples in a row, with a slight delay
    for ( i = 0 ; i < TEMPSAMPLES ; i++ )
    {
        average += analogRead(pin);
        delay(10);
    }
    average /= TEMPSAMPLES;

    #ifdef THERM_DEBUG
        debug("Average analog reading %f", average);
    #endif

    // convert the value to resistance
    average = 4095 / average - 1;
    average = SERIESRESISTOR / average;
    #ifdef THERM_DEBUG
        debug("Thermistor resistance %f", average);
    #endif

    float steinhart;
    steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= TEMPCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C

    #ifdef THERM_DEBUG
        debug("Temperature %f *C", steinhart, steinhartf);
    #endif

    return steinhart;
}

float readTempC(uint8_t addr[8])
{
    byte i;
    uint8_t data[12];
    float celsius;
    unsigned int raw;

    own.reset();
    own.select(addr);
    own.write(0xBE);

    for ( i = 0 ; i < 9 ; i ++ )
    {
        data[i] = own.read();
    }

    uint8_t crc = OneWire::crc8(data, 8);
    if ( crc != data[8] ) {
        debug(" invalid crc: %d != %d", crc, data[8]);
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


#include "config.h"

void button_onPress(Button* button)
{
    display.setCursor(0, LCDLINEHEIGHT * 2);
    memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
    snprintf(lcdLineBuf, LCDLINELENGTH, LCDBLANKLINE);
    display.print(lcdLineBuf);
    display.display();
    display.setCursor(0, LCDLINEHEIGHT * 2);
    memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
    snprintf(lcdLineBuf, LCDLINELENGTH, "Press %s", button->name);
    display.print(lcdLineBuf);
    display.display();
}

void button_onRelease(Button* button)
{
    display.setCursor(0, LCDLINEHEIGHT * 3);
    memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
    snprintf(lcdLineBuf, LCDLINELENGTH, LCDBLANKLINE);
    display.print(lcdLineBuf);
    display.display();
    display.setCursor(0, LCDLINEHEIGHT * 3);
    memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
    snprintf(lcdLineBuf, LCDLINELENGTH, "Rel %s", button->name);
    display.print(lcdLineBuf);
    display.display();
}

void button_onLongClick(Button* button)
{
    display.setCursor(0, LCDLINEHEIGHT * 3);
    memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
    snprintf(lcdLineBuf, LCDLINELENGTH, LCDBLANKLINE);
    display.print(lcdLineBuf);
    display.display();
    display.setCursor(0, LCDLINEHEIGHT * 3);
    memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
    snprintf(lcdLineBuf, LCDLINELENGTH, "Long %s", button->name);
    display.print(lcdLineBuf);
    display.display();
}

void button_onClick(Button* button)
{
    display.setCursor(0, LCDLINEHEIGHT * 3);
    memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
    snprintf(lcdLineBuf, LCDLINELENGTH, LCDBLANKLINE);
    display.print(lcdLineBuf);
    display.display();
    display.setCursor(0, LCDLINEHEIGHT * 3);
    memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
    snprintf(lcdLineBuf, LCDLINELENGTH, "Click %s", button->name);
    display.print(lcdLineBuf);
    display.display();

    if ( strcmp("Sel", button->name) == 0 )
    {
        debug("'Sel' was clicked.");
        if ( achState == HIGH )
        {
            achState = LOW;
        }
        else
        {
            achState = HIGH;
        }
        mcp.digitalWrite(ACHPIN, achState);
    }
    else if ( strcmp("Off", button->name) == 0 )
    {
        debug("'Off' was clicked.");
        scanOWN();
    }
}

void debug(String message) {
    char msg [50];
    sprintf(msg, message.c_str());
    //Particle.publish("chronicle-debug", msg);
    Serial.println(message);
}
void debug(String message, int value) {
    char msg [50];
    sprintf(msg, message.c_str(), value);
    debug(msg);
}
void debug(String message, float value) {
    char msg [50];
    sprintf(msg, message.c_str(), value);
    debug(msg);
}
void debug(String message, float value, float value2) {
    char msg [50];
    sprintf(msg, message.c_str(), value, value2);
    debug(msg);
}
void debug(String message, int value, int value2) {
    char msg [50];
    sprintf(msg, message.c_str(), value, value2);
    debug(msg);
}
void debug(String message, char *value) {
    char msg [50];
    sprintf(msg, message.c_str(), value);
    debug(msg);
}
void debug(String message, char *value, float value2) {
    char msg [50];
    sprintf(msg, message.c_str(), value, value2);
    debug(msg);
}
