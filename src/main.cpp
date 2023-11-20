/*

Project to control a bumper car using a ESP32 microcontroller.
The car is using a 12V car battery to run a Bosch starter Motor.
When a coin is inserted in a coin acceptor a pedal becomes active to drive the car.

Key components are:

              DFRobot FireBeetle 2 ESP32-E https://www.dfrobot.com/product-2231.html
              DFRobot Gravity: IO Shield for FireBeetle 2 https://www.dfrobot.com/product-2395.html
              DFRobot Gravity: I2C Digital Wattmeter https://www.dfrobot.com/product-1827.html
              DFRobot Gravity: Digital 10A Relay Module https://www.dfrobot.com/product-1572.html
              DD2712SA 6.5-27V 2.5A DC DC Step Down Converter Module
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

#define NUM_INTERNAL_LEDS 1 // number of LEDs for FastLed
#define NUM_EXTERNAL_LEDS 7 // number of LEDs for FastLed
#define BRIGHTNESS 32       // set the brightness of the LED`S from 0-255
#define DATA_PIN1 5         // PIN for the builtin RGB LED of the FireBeetle 2 for FastLED
#define DATA_PIN2 D13       // PIN for the external Neopixel WS2812B RGB LED for FastLED
#define DEBOUNCE_TIME 25    // the debounce time in milliseconds for the coin acceptor
#define motorRelay D10      // PIN for the relay module

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4); // DFRobot I2C Digital Wattmeter
CRGB internalLed[NUM_INTERNAL_LEDS];                   // for FastLed
CRGB externalLeds[NUM_EXTERNAL_LEDS];                  // for FastLed
ezButton coinAcceptor(D11);                            // create ezButton object for the coin acceptor attached to pin D2
ezButton pedalButton(D12);                             // create ezButton object for the pedal magnetic switch attached to pin D3

bool isRideAllowed = false;

float batteryVoltageReading = 0;       // Voltage reading of the battery
float batteryCutoffVoltage = 0; // Voltage to disable the ride
float batteryLowVoltage = 0;    // minimal Voltage to start a ride

const long ledOffDelay = 10000;    // LED timeout
const long rideDuration = 180000; // duration of one ride in milliseconds

unsigned long lastWattmeterReading = 0;
unsigned long ledOffTimer = 0;
unsigned long rideExpirationTime = 0;
unsigned long rideRemainingTime = 0;

uint8_t startIndex;
uint8_t colorIndex;

void idleAnimation1();
void idleAnimation2();
void idleAnimation3();

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
  pinMode(motorRelay, OUTPUT);
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

  if (currentTime > rideExpirationTime)
  {
    rideRemainingTime = 0;
    digitalWrite(motorRelay, LOW); // if duration has passed the motor will be disabled
  }
  else
  {
    rideRemainingTime = rideExpirationTime - currentTime; // updates the time remaining until the ride ends
  }

  /*
  Coin Acceptor, starts the ride or extends one if already running
  */

  if (coinAcceptor.isPressed() && isRideAllowed != false)
  {
    if (currentTime < rideExpirationTime) // extends the duration of the ride by the value set for rideDuration if a ride is active
    {
      rideRemainingTime = rideExpirationTime + rideDuration;
      rideExpirationTime = rideRemainingTime;
      ledOffTimer = rideExpirationTime + ledOffDelay;
      debug("Timer has extended. New time remaining ");
      debugln((rideExpirationTime - currentTime) / 1000);
    }
    else // starts the ride if no ride is active
    {
      rideExpirationTime = currentTime + rideDuration;
      rideRemainingTime = rideExpirationTime - currentTime;
      ledOffTimer = rideExpirationTime + ledOffDelay;
      debug("Timer has started. Time Remaining ");
      debugln(rideDuration / 1000);
    }
  }

  /*
  Pedal Button, control the relay module with the pedal to drive the car
  */

  if (pedalButton.isPressed() && isRideAllowed != false) // if the pedal is pressed and a ride is running
  {
    if (rideRemainingTime != 0)
    {
      digitalWrite(motorRelay, HIGH);
      debug("Pedal is pressed. Time remaining ");
      debugln(rideRemainingTime / 1000);
    }
    else
    {
      debugln("Pedal is pressed but timer has expired");
    }
  }

  if (pedalButton.isReleased())
  {
    digitalWrite(motorRelay, LOW);
  }

  /*
  Status Internal LED
  */

  if (rideRemainingTime > 10000 && isRideAllowed != false)
  {
    internalLed[0] = CRGB::Green;
    FastLED.show();
  }
  else if (rideRemainingTime > 0 && rideRemainingTime < 10000 && isRideAllowed != false)
  {
    internalLed[0] = CRGB::Orange;
    FastLED.show();
  }
  if (currentTime > rideExpirationTime && currentTime < ledOffTimer && isRideAllowed != false)
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

  if (rideRemainingTime > 10000 && isRideAllowed != false)
  {
    fill_solid(externalLeds, NUM_EXTERNAL_LEDS, CRGB::Green);
    FastLED.show();
  }
  else if (rideRemainingTime > 0 && rideRemainingTime < 10000 && isRideAllowed != false)
  {
    fill_solid(externalLeds, NUM_EXTERNAL_LEDS, CRGB::Orange);
    FastLED.show();
  }
  if (currentTime > rideExpirationTime && currentTime < ledOffTimer && isRideAllowed != false)
  {
    fill_solid(externalLeds, NUM_EXTERNAL_LEDS, CRGB::Red);
    FastLED.show();
  }

  /*
Idle Animation External LEDS 1, pick one
*/

  if (currentTime > ledOffTimer && isRideAllowed == 1)
  {
    idleAnimation1();
    // idleAnimation2();
    // idleAnimation3();
  }

  /*
  Error Message External LEDS
  */

  if (isRideAllowed == 0)
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

    batteryVoltageReading = ina219.getBusVoltage_V();

    debug("Ride Allowed:      ");
    debugln(isRideAllowed);
    debugln("");

    lastWattmeterReading = currentTime + 5000;
  }
  if (batteryVoltageReading > batteryLowVoltage)
  {
    isRideAllowed = true;
  }
  if (batteryVoltageReading < batteryCutoffVoltage)
  {
    isRideAllowed = false;
    digitalWrite(motorRelay, LOW);
  }
}

void idleAnimation1()
{
  uint16_t beatA = beatsin16(30, 0, 255);
  uint16_t beatB = beatsin16(20, 0, 255);
  fill_rainbow(externalLeds, NUM_EXTERNAL_LEDS, (beatA + beatB) / 2, 8);

  FastLED.show();

  FastLED.show();
}

void idleAnimation2()
{
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

void idleAnimation3()
{
  uint16_t x;
  int scale;
  uint16_t t;

  x = 0;
  t = millis() / 5;
  scale = beatsin8(10, 10, 30);

  for (int i = 0; i < NUM_EXTERNAL_LEDS; i++)
  {
    uint8_t noise = inoise8(i * scale + x, t);
    uint8_t hue = map(noise, 50, 190, 0, 255);
    externalLeds[i] = CHSV(hue, 255, 255);
  }

  FastLED.show();
}