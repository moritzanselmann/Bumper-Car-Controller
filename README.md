# Bumper-Car-Controller

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