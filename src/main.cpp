#include <Arduino.h>
#include <DFRobot_INA219.h> //DFRobot I2C Digital Wattmeter
#include <ezButton.h>       //ezButton Library
#include <FastLED.h>        //FastLED Library
#include <Wire.h>

#define DEBOUNCE_TIME 25   // the debounce time in milliseconds
#define NUM_LEDS 1         // for FastLed
#define DATA_PIN2 5        // FastLed builtin RGB LED
#define DATA_PIN1 D0       // FastLed external LED
#define PowerController D7 // define MOSFET Power Controller to D7
#define BRIGHTNESS 16

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4);
CRGB leds[NUM_LEDS];             // for FastLed
ezButton coinAcceptorButton(D2); // create ezButton object that attach to pin D2
ezButton pedalButton(D3);        // create ezButton object that attach to pin D3

bool rideAllowed = false;

float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;
float voltageReading = 0;           // Voltage reading of the battery
float batteryCuttOffVoltage = 10; // Voltage to disable the ride
float batteryLowVoltage = 11;     // minimal Voltage to start a ride

const long ledTimout = 3000;     // LED timeout
const long rideDuration = 180000; // duration of one ride in milliseconds

unsigned long lastWattmeterReading = 0;
unsigned long ledOffTimer = 0;
unsigned long rideEndtTime = 0;
unsigned long rideTimeRemaining = 0;

void setup()
{
  Serial.begin(115200);
  coinAcceptorButton.setDebounceTime(DEBOUNCE_TIME); // set debounce time to DEBOUNCE_TIME milliseconds
  pedalButton.setDebounceTime(DEBOUNCE_TIME);        // set debounce time to DEBOUNCE_TIME milliseconds
  FastLED.addLeds<NEOPIXEL, DATA_PIN1>(leds, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN2>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  leds[0] = CRGB::Black;
  FastLED.show();
  pinMode(PowerController, OUTPUT); // set D7 to output
  ina219.linearCalibrate(ina219Reading_mA, extMeterReading_mA);
  Serial.println();
}

void loop()
{
  coinAcceptorButton.loop();
  pedalButton.loop();

  unsigned long currentTime = millis();

  /*
  Time Checker
  */

  if (currentTime > rideEndtTime)
  {
    rideTimeRemaining = 0;
    digitalWrite(PowerController, LOW); // disable MOSFET Power Controller
  }
  else
  {
    rideTimeRemaining = rideEndtTime - currentTime;
  }

  /*
  Coin Acceptor Button
  */

  if (coinAcceptorButton.isPressed())
  {
    if (currentTime < rideEndtTime) // extends the duration of the ride by const long rideDuration
    {
      rideTimeRemaining = rideEndtTime + rideDuration;
      rideEndtTime = rideTimeRemaining;
      ledOffTimer = rideEndtTime + ledTimout;
      Serial.print("Timer has extended. New time remaining ");
      Serial.println((rideEndtTime - currentTime) / 1000);
    }
    else // starts the ride
    {
      rideEndtTime = currentTime + rideDuration;
      rideTimeRemaining = rideEndtTime - currentTime;
      ledOffTimer = rideEndtTime + ledTimout;
      Serial.print("Timer has started. Time Remaining ");
      Serial.println(rideDuration / 1000);
    }
  }

  /*
  Pedal Button
  */

  if (pedalButton.isPressed() && rideAllowed != false) // if the pedal is pressed and a interval is running
  {
    if (rideTimeRemaining != 0)
    {
      digitalWrite(PowerController, HIGH); // switch MOSFET Power Controller with pedal
      // Serial.println("Pedal is pressed and timer is running");
      Serial.print("Pedal is pressed. Time remaining ");
      Serial.println(rideTimeRemaining / 1000);
    }
    else
    {
      // Serial.println("Pedal is pressed but timer has expired");
    }
  }

  if (pedalButton.isReleased())
  {
    digitalWrite(PowerController, LOW);
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
    Serial.print("Battery Voltage:   ");
    Serial.print(ina219.getBusVoltage_V());
    Serial.println("V");
    // Serial.print("ShuntVoltage: ");
    // Serial.print(ina219.getShuntVoltage_mV(), 3);
    // Serial.println("mV");
    // Serial.print("Current:      ");
    // Serial.print(ina219.getCurrent_mA(), 1);
    // Serial.println("mA");
    // Serial.print("Power:        ");
    // Serial.print(ina219.getPower_mW(), 1);
    // Serial.println("mW");
    // Serial.println("");
    voltageReading = ina219.getBusVoltage_V();
    Serial.print("Ride Allowed:      ");
    Serial.println(rideAllowed);
    Serial.println("");
    lastWattmeterReading = currentTime + 5000;
  }
  if (voltageReading > batteryLowVoltage)
  {
    rideAllowed = true;
  }
  if (voltageReading < batteryCuttOffVoltage)
  {
    rideAllowed = false;
    digitalWrite(PowerController, LOW);
  }
}