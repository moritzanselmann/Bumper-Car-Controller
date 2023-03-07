#include <Arduino.h>
#include <ezButton.h> //ezButton Library
#include <FastLED.h>  //FastLED Library

#define DEBOUNCE_TIME 50   // the debounce time in milliseconds
#define NUM_LEDS 1         // for FastLed
#define DATA_PIN 5         // for FastLed
#define PowerController D7 // define MOSFET Power Controller to D7

ezButton coinAcceptorButton(D2); // create ezButton object that attach to pin D2
ezButton pedalButton(D3);        // create ezButton object that attach to pin D3

CRGB leds[NUM_LEDS]; // for FastLed

const long ledTimout = 3000;    // LED timeout
const long rideDuration = 5000; // duration of one ride in milliseconds

unsigned long ledOffTimer = 0;
unsigned long rideEndtTime = 0;
unsigned long rideTimeRemaining = 0;

void setup()
{
  Serial.begin(115200);
  coinAcceptorButton.setDebounceTime(DEBOUNCE_TIME); // set debounce time to DEBOUNCE_TIME milliseconds
  pedalButton.setDebounceTime(DEBOUNCE_TIME);        // set debounce time to DEBOUNCE_TIME milliseconds
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();
  pinMode(PowerController, OUTPUT); // set D7 to output
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

  if (pedalButton.isPressed()) // if the pedal is pressed and a interval is running
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

}