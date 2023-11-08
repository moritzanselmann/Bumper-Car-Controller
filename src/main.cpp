/*

Project to control a bumper car using a ESP32 microcontroller.
The car is using a 12V car battery to run a Bosch starter Motor.
When a coin is inserted in a coin acceptor a pedal becomes active to drive the car.

Key components are:

              DFRobot FireBeetle 2 ESP32-E https://www.dfrobot.com/product-2231.html
              DFRobot Gravity: IO Shield for FireBeetle 2 https://www.dfrobot.com/product-2395.html
              DFRobot Gravity: I2C Digital Wattmeter https://www.dfrobot.com/product-1827.html
              DFRobot Gravity: Digital 10A Relay Module https://www.dfrobot.com/product-1572.html
              DD2712SA 2.5A DC DC Step Down Converter Module
              Neopixel WS2812B RGB LED
              generic coin acceptor
              magnetic contact switch for the pedal

*/

#include <Arduino.h>        //include Arduino
#include <DFRobot_INA219.h> //DFRobot I2C Digital Wattmeter
#include <ezButton.h>       //ezButton Library
#include <FastLED.h>        //FastLED Library

#define DEBUG 1 // 1 for debug messages on the serial monitor or 0 to disable

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#define NUM_LEDS 1       // number of LED´s for FastLed
#define BRIGHTNESS 16    // set the brightness of the LED`S from 0-255
#define DATA_PIN1 D0     // PIN for the external Neopixel WS2812B RGB LED for FastLED
#define DATA_PIN2 5      // PIN for the builtin RGB LED of the FireBeetle 2 for FastLED
#define DEBOUNCE_TIME 25 // the debounce time in milliseconds for the coin acceptor
#define motorRelais D7   // PIN for the Relay Module

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4); // DFRobot I2C Digital Wattmeter
CRGB leds[NUM_LEDS];                                   // for FastLed
ezButton coinAcceptor(D2);                             // create ezButton object for the coin acceptor attached to pin D2
ezButton pedalButton(D3);                              // create ezButton object for the pedal magnetic switch attached to pin D3

bool rideAllowed = false;

float voltageReading = 0;        // Voltage reading of the battery
float batteryCuttOffVoltage = 0; // Voltage to disable the ride
float batteryLowVoltage = 0;     // minimal Voltage to start a ride

const long ledTimout = 3000;      // LED timeout
const long rideDuration = 180000; // duration of one ride in milliseconds

unsigned long lastWattmeterReading = 0;
unsigned long ledOffTimer = 0;
unsigned long rideEndtTime = 0;
unsigned long rideTimeRemaining = 0;

void setup()
{
  Serial.begin(115200);
  coinAcceptor.setDebounceTime(DEBOUNCE_TIME);
  pedalButton.setDebounceTime(DEBOUNCE_TIME);
  FastLED.addLeds<NEOPIXEL, DATA_PIN1>(leds, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN2>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  leds[0] = CRGB::Black;
  FastLED.show();
  pinMode(motorRelais, OUTPUT); 
  Serial.println("SETUP COMPLETE");
}

void loop()
{
  coinAcceptor.loop();
  pedalButton.loop();

  unsigned long currentTime = millis();

  /*
  Time Checker
  */

  if (currentTime > rideEndtTime)
  {
    rideTimeRemaining = 0;
    digitalWrite(motorRelais, LOW); // disable MOSFET Power Controller
  }
  else
  {
    rideTimeRemaining = rideEndtTime - currentTime;
  }

  /*
  Coin Acceptor Button
  */

  if (coinAcceptor.isPressed())
  {
    if (currentTime < rideEndtTime) // extends the duration of the ride by const long rideDuration
    {
      rideTimeRemaining = rideEndtTime + rideDuration;
      rideEndtTime = rideTimeRemaining;
      ledOffTimer = rideEndtTime + ledTimout;
      debug("Timer has extended. New time remaining ");
      debugln((rideEndtTime - currentTime) / 1000);
    }
    else // starts the ride
    {
      rideEndtTime = currentTime + rideDuration;
      rideTimeRemaining = rideEndtTime - currentTime;
      ledOffTimer = rideEndtTime + ledTimout;
      debug("Timer has started. Time Remaining ");
      debugln(rideDuration / 1000);
    }
  }

  /*
  Pedal Button
  */

  if (pedalButton.isPressed() && rideAllowed != false) // if the pedal is pressed and a interval is running
  {
    if (rideTimeRemaining != 0)
    {
      digitalWrite(motorRelais, HIGH); // switch MOSFET Power Controller with pedal
      debug("Pedal is pressed. Time remaining ");
      debugln(rideTimeRemaining / 1000);
    }
    else
    {
      debugln("Pedal is pressed but timer has expired");
    }
  }

  if (pedalButton.isReleased())
  {
    digitalWrite(motorRelais, LOW);
  }

  /*
  Status LED
  */

  if (rideTimeRemaining > 3000)
  {
    leds[0] = CRGB::Green;
    FastLED.show();
  }
  else if (rideTimeRemaining > 0 && rideTimeRemaining < 3000)
  {
    leds[0] = CRGB::Orange;
    FastLED.show();
  }
  if (currentTime > rideEndtTime && currentTime < ledOffTimer)
  {
    leds[0] = CRGB::Red;
    FastLED.show();
  }

  if (currentTime > ledOffTimer)
  {
    leds[0] = CRGB::Black;
    FastLED.show();
  }

  /*
Wattmeter
*/

  if (lastWattmeterReading < currentTime)
  {
    debug("Battery Voltage:   ");
    debug(ina219.getBusVoltage_V());
    debugln("V");

    voltageReading = ina219.getBusVoltage_V();

    debug("Ride Allowed:      ");
    debugln(rideAllowed);
    debugln("");
    lastWattmeterReading = currentTime + 5000;
  }
  if (voltageReading > batteryLowVoltage)
  {
    rideAllowed = true;
  }
  if (voltageReading < batteryCuttOffVoltage)
  {
    rideAllowed = false;
    digitalWrite(motorRelais, LOW);
  }
}