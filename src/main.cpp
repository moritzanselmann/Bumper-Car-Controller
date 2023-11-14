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

#define NUM_INTERNAL_LEDS 1 // number of LED´s for FastLed
#define NUM_EXTERNAL_LEDS 7 // number of LED´s for FastLed
#define BRIGHTNESS 32       // set the brightness of the LED`S from 0-255
#define DATA_PIN1 5         // PIN for the builtin RGB LED of the FireBeetle 2 for FastLED
#define DATA_PIN2 D13       // PIN for the external Neopixel WS2812B RGB LED for FastLED
#define DEBOUNCE_TIME 25    // the debounce time in milliseconds for the coin acceptor
#define motorRelais D10     // PIN for the relay module

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4); // DFRobot I2C Digital Wattmeter
CRGB internalLed[NUM_INTERNAL_LEDS];                   // for FastLed
CRGB externalLeds[NUM_EXTERNAL_LEDS];                  // for FastLed
ezButton coinAcceptor(D11);                            // create ezButton object for the coin acceptor attached to pin D2
ezButton pedalButton(D12);                             // create ezButton object for the pedal magnetic switch attached to pin D3

bool rideAllowed = false;

float voltageReading = 0;        // Voltage reading of the battery
float batteryCuttOffVoltage = 0; // Voltage to disable the ride
float batteryLowVoltage = 0;     // minimal Voltage to start a ride

const long ledTimout = 10000;    // LED timeout
const long rideDuration = 20000; // duration of one ride in milliseconds

unsigned long lastWattmeterReading = 0;
unsigned long ledOffTimer = 0;
unsigned long rideEndtTime = 0;
unsigned long rideTimeRemaining = 0;

uint8_t startIndex;
uint8_t colorIndex;

void setup()
{
  Serial.begin(115200);
  coinAcceptor.setDebounceTime(DEBOUNCE_TIME);
  pedalButton.setDebounceTime(DEBOUNCE_TIME);
  FastLED.addLeds<NEOPIXEL, DATA_PIN1>(internalLed, NUM_INTERNAL_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN2>(externalLeds, NUM_EXTERNAL_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  internalLed[0] = CRGB::Black;
  externalLeds[NUM_EXTERNAL_LEDS] = CRGB::Black;
  FastLED.show();
  pinMode(motorRelais, OUTPUT);
  Serial.println("SETUP COMPLETE");
  Serial.println(" ");
}

void loop()
{
  coinAcceptor.loop();
  pedalButton.loop();

  unsigned long currentTime = millis();

  /*
  checks if the duration of the ride has passed
  */

  if (currentTime > rideEndtTime)
  {
    rideTimeRemaining = 0;
    digitalWrite(motorRelais, LOW); // if durartion has passed the motor will be disabled
  }
  else
  {
    rideTimeRemaining = rideEndtTime - currentTime; // updates the time remaining until the ride ends
  }

  /*
  Coin Acceptor, starts the ride or extends one if allready running
  */

  if (coinAcceptor.isPressed() && rideAllowed != false)
  {
    if (currentTime < rideEndtTime) // extends the duration of the ride by the value set for rideDuration if a ride is active
    {
      rideTimeRemaining = rideEndtTime + rideDuration;
      rideEndtTime = rideTimeRemaining;
      ledOffTimer = rideEndtTime + ledTimout;
      debug("Timer has extended. New time remaining ");
      debugln((rideEndtTime - currentTime) / 1000);
    }
    else // starts the ride if no ride is active
    {
      rideEndtTime = currentTime + rideDuration;
      rideTimeRemaining = rideEndtTime - currentTime;
      ledOffTimer = rideEndtTime + ledTimout;
      debug("Timer has started. Time Remaining ");
      debugln(rideDuration / 1000);
    }
  }

  /*
  Pedal Button, control the relay module with the pedal to drive the car
  */

  if (pedalButton.isPressed() && rideAllowed != false) // if the pedal is pressed and a ride is running
  {
    if (rideTimeRemaining != 0)
    {
      digitalWrite(motorRelais, HIGH);
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
  Status Internal LED
  */

  if (rideTimeRemaining > 10000 && rideAllowed != false)
  {
    internalLed[0] = CRGB::Green;
    FastLED.show();
  }
  else if (rideTimeRemaining > 0 && rideTimeRemaining < 10000 && rideAllowed != false)
  {
    internalLed[0] = CRGB::Orange;
    FastLED.show();
  }
  if (currentTime > rideEndtTime && currentTime < ledOffTimer && rideAllowed != false)
  {
    internalLed[0] = CRGB::Red;
    FastLED.show();
  }

  if (currentTime > ledOffTimer)
  {
    internalLed[0] = CRGB::Black;
    FastLED.show();
  }

  /*
  External LEDS
  */

  if (rideTimeRemaining > 10000 && rideAllowed != false)
  {
    fill_solid(externalLeds, NUM_EXTERNAL_LEDS, CRGB::Green);
    FastLED.show();
  }
  else if (rideTimeRemaining > 0 && rideTimeRemaining < 10000 && rideAllowed != false)
  {
    fill_solid(externalLeds, NUM_EXTERNAL_LEDS, CRGB::Orange);
    FastLED.show();
  }
  if (currentTime > rideEndtTime && currentTime < ledOffTimer && rideAllowed != false)
  {
    fill_solid(externalLeds, NUM_EXTERNAL_LEDS, CRGB::Red);
    FastLED.show();
  }

  if (currentTime > ledOffTimer && rideAllowed == 1)
  {

    /*
  Ide Animation External LEDS
  */

    EVERY_N_MILLISECONDS(10)
    {
      colorIndex = startIndex;
      for (int i = 0; i < NUM_EXTERNAL_LEDS; i++)
      {
        externalLeds[i] = ColorFromPalette(RainbowColors_p, colorIndex, 255, LINEARBLEND);
        colorIndex = colorIndex + 10;
      }

      FastLED.show();
      startIndex = startIndex + 1;
    }
  }

  /*
  Error Message External LEDS
  */

  if (rideAllowed == 0)
  {
    EVERY_N_MILLISECONDS(1200)
    {
      static boolean errorBlink;
      errorBlink = !errorBlink;
      if (errorBlink == 1)
      {
        for (int i = 0; i < 7; i++)
        {
          externalLeds[i] = CRGB::Red;
        }
      }
      else
      {
        for (int i = 0; i < 7; i++)
        {
          externalLeds[i] = CRGB::Black;
        }
      }
    }
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
