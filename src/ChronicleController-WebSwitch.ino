#include "config.h"

void button_onPress(Button* button);
void button_onRelease(Button* button);
void button_onClick(Button* button);
void button_onLongClick(Button* button);
float readThermF(byte pin);

DS18B20 ds18b20 = DS18B20(D1);
Adafruit_SSD1306 display;
Adafruit_MCP23017 mcp;

typedef struct WebPowerSwitch
{
    IPAddress ip;
    String Auth;
    int port;
} WebPowerSwitch;

typedef struct Thermistor
{
    byte pin;
    double lastReading;
    unsigned long lastReadingTime;
    int seriesResistor;
    int norminalResistance;   // resistance at nominal temperature
    byte nominalTemperature;  // temp. for nominal resistance (almost always 25 C)
    int tempCoefficient; // The beta coefficient of the thermistor (usually 3000-4000)
    byte sampleCount;    // how many samples to take and average, more takes longer
    byte sampleDelay;
} Thermistor;

typedef enum RelayType {GPIO,MCP,WEB} RelayType;

typedef struct Relay
{
    byte pin;               // pin number for GPIO,MCP, Outlet Number for Web
    RelayType type;
    Adafruit_MCP23017* mcp;
    WebPowerSwitch* web;
    bool state;
    unsigned long lastTime;

    int minOnTime;
    int minOffTime;
};

Button buttons[BUTTON_COUNT] = {
    { "Left", 13, &mcp },
    { "Right", 14, &mcp },
    { "Up", 15, &mcp },
    { "Down", 4, &mcp },
    { "Sel", 3, &mcp },
    { "Off", 2, &mcp }
};

typedef struct Chronicle
{
    char name[10];

    Thermistor therm;
    double targetTemp;

    Relay cool;
    Relay warm;

    bool enableAuto;
    byte autoThesholdHigh;
    byte autoThesholdLow;
} Chronicle;

// if chronicle.therm.getReading() > targetTemp - autoThesholdHigh : chronicle.cool.on();
// if chronicle.therm.getReading() > targetTemp - autoThesholdLow : chronicle.warm.on();

typedef struct TempSource
{
    char name[10];

    Thermistor temp;
    double targetTemp;

    Relay relay;

    bool enableAuto;
    byte autoThesholdHigh;
    byte autoThesholdLow;
} TempSource;

void setup() {
    Serial.begin(9600);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the diy display)

    // Show image buffer on the display hardware.
    // Since the buffer is intialized with an Adafruit splashscreen
    // internally, this will display the splashscreen.
    display.display();
    delay(500);

    // Clear the buffer.
    display.clearDisplay();

    // text display tests
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("Ready v6");
    display.display();
    //display.setTextSize(2);
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

    mcp.begin();
    setup_buttons(buttons);

    //float therm = readThermF(THERM_COLD_PIN);
    float therm = 0;

    char msg[11];
    snprintf(msg, 11, "1 NN %2.f 60", therm);
    display.print(msg);
    display.display();
}

int lastPrint = 0;
int printDelay = 2000;
int mcpPin = 0;

void loop()
{
    int gpioVal;

    check_buttons(buttons);

    debug("DS Temp: %2.2f", getDSTemp());
    delay(2000);
}

void button_onPress(Button* button)
{
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Press ");
    display.println(button->name);
    display.display();
}

void button_onRelease(Button* button)
{
    display.print("Release ");
    display.println(button->name);
    display.display();
}

void button_onLongClick(Button* button)
{
    display.print("Long Click ");
    display.println(button->name);
    display.display();
}

void button_onClick(Button* button)
{
    display.print("Click");
    display.println(button->name);
    display.display();

    //float therm = readThermF(THERM_COLD_PIN);
    float therm = 0;
    display.print("Cur: ");
    display.println(therm);
    display.display();
}

#define TEMPSAMPLES 10
#define SERIESRESISTOR 10000
#define THERMISTORNOMINAL 1000
#define TEMPERATURENOMINAL 1000
#define TEMPCOEFFICIENT 1000

float readThermF(byte pin)
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
    float steinhartf = (steinhart * 9.0)/ 5.0 + 32.0;

    #ifdef THERM_DEBUG
        debug("Temperature %f *C (%f *F)", steinhart, steinhartf);
    #endif

    return steinhartf;
}

float getDSTemp(){
    float celsius = -99;
    float fahrenheit = -99;
    int dsAttempts = 0;
    if(!ds18b20.search()){
      ds18b20.resetsearch();
      celsius = ds18b20.getTemperature();
      debug(" celsius: %2.2f", celsius);
      while (!ds18b20.crcCheck() && dsAttempts < 4){
        debug(" Caught bad value.");
        dsAttempts++;
        debug(" Attempts to Read: %d", dsAttempts);
        if (dsAttempts == 3){
          delay(1000);
        }
        ds18b20.resetsearch();
        celsius = ds18b20.getTemperature();
        continue;
      }
      dsAttempts = 0;
      fahrenheit = ds18b20.convertToFahrenheit(celsius);
      debug(" fahrenheit: %2.2f", fahrenheit);
    }
    return fahrenheit;
}
