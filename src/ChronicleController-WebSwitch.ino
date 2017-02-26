#include "config.h"

Adafruit_MCP23017 mcp;
byte achState = LOW;
Adafruit_SSD1306 display(OLED_RESET);
OneWire own(OWNPIN);
char lcdLineBuf[LCDLINELENGTH];

unsigned long previousMillis = 0;
const long tempInterval = 1000;

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
    pinMode(ACTPIN, INPUT);

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the diy display)

    // Show the Adafruit splashscreen
    display.display();
    delay(500);

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
    setup_buttons(buttons);
}

void loop()
{
    unsigned long now = millis();
    check_buttons(buttons);

    if ( now - previousMillis >= tempInterval )
    {
        previousMillis = now;
        float therm = readTempC(ACTPIN);
        therm = convertTempCtoF(therm);

        display.setCursor(0, LCDLINEHEIGHT * 1);
        memset(lcdLineBuf, 0, sizeof(lcdLineBuf));
        snprintf(lcdLineBuf, LCDLINELENGTH, "%2.1f %d  ", therm, achState);
        display.print(lcdLineBuf);
        display.display();
    }
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
