/*
            \\----------------------------------------------\\
            |\\      ------------------------------------    \\\
            |  \\    -------------------------------           \\\
            |   ==================================================\
            |   Hello, it is Mark Gauntlet V5 ultimate code        \
            |                                                       \
            |   To make it work properly - be sure, that all         \                   \\\\
            \\  connected right and libs are edited or downloaded     \             \\\\\   \\\\\\\\\
           //\\ from this repository   ✓                               \    \\\\\\\\\               \\\\
         //\\\\\============================================\\\\\\\\\\\\\\\\                           \
        \\\\\\\\\=======================================================\ ----                          \\
-------\----   \\\\=====================================================\\ -----------                   \\                       ///////////////////////
------------     \\\=====================================================\\  - - ---- ----- -----         \\////////////////////// ..........................
----- \\         \\=======================================================\\\       - -- ---- - -\\\\\\\\\\\\\\\\\\\\\\\\ \\\\\\\\\\\  \...................
---- \\\         \\\=======================================================\\\                 \\-\\       \\\\\\\ \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
---- \\\         \\=========================================================\\               __\___\\_________\\\\\\\\\\\\\\\ \\\\\\\\\  \..................
---- \= =========\\==========================================================\\                     \\          _\\\\\\\\\\\\\\\\\\\\\\\\\..................
----  ===        \\==========================================================\\                     \\           \\\\\\\\  \\\\\ \\\\\\\ \\\\\\\\\\\\\\\\\\\\\
----             \\\=========================================================\\     ---             \    \\\\\\\\\\\\\\\\\\\     \  \\\\\\\\.................
----                                                                              ---           - \\\\\\\\        \\\  \\\\\
----                                                                             -     ------------\                 \\\\\ \\
----     ----                                                          -  - ------ -- --- -                            \\\\\\\
----    -------------------------- - ------------------------------ --    ----  - -                                      \\\ \
- ---  ---                       ---------              ------------------    --                                         \ \\\\
--                                     --- --- -- -------                                                                   \
-------------------------------- -- - -

Now about functional:
1.Reading data from UI
  1.1 Getting mode and channel from 2 switches                        
  1.2 Getting X and Y data from joystick + interpr. into stable values
  1.3 Getting potentiometr data + filter
  1.4 Getting buttons data
  1.5 Checking gesture sensor data 
  1.6 Getting data from both flex. sensors
  1.7 Getting data from EMG sensor

2. Reading from sensors to determine the position
  2.1 Reading from magnetometer
  2.2 Reading from both acselerometers
  2.3 Reading from both gyroscopes
  2.4 Getting coordinates from GPS module
  2.5 Getting altitude from pressure sensor 

3. Getting data about enviroment and general stuff
  3.1 Read data from thermometer
  3.2 Read data from humid. sensor
  3.3 Read data from tVOC sensor
  3.4 Read data from O2 sensor
  3.5 Read data from CO2 sensor 
  3.6 Getting real time from RTC module
  3.7 Getting commands from voice recognition module 

4. Sending data through wireless com. devices
  4.1 Sending data via NRF24L01+ (100 mW)
  4.2 Sending data via LoRa module 
  4.3 Sending data via bluetooth module
  4.4 Sending data via ESP8266 (WiFi)

5. UI and cool features
  5.1 Menu on display + touchscreen response
  5.2 Patterns of illumination of the address LED strip
  5.3 Two-step activation
  5.4 The ability to play music and any replicas

*/


#include <Arduino.h>

#include <SPI.h>

#include <Wire.h>

#include <SoftwareSerial.h>

#include "Adafruit_GFX.h"
#include <MCUFRIEND_kbv.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_MPU6050.h>

#include "microLED.h"

#include <LoRa.h>

#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#include <iarduino_GPS_NMEA.h>

#include <iarduino_RTC.h>

#include <Adafruit_BME280.h>

#include "Adafruit_APDS9960.h"

#include <SHT3x.h> 

#include "Adafruit_SGP30.h"

#include "VoiceRecognitionV3.h"

#include <DFPlayer_Mini_Mp3.h>

MCUFRIEND_kbv tft;

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_MPU6050 mpu;

#define NUM_LEDS 8  // кол-во диодов
#define LED_PIN 8   // пин подключения

#define REPLACE_FASTLED // пункт 0
#define COLOR_DEBTH 3   // пункт 1
LEDdata leds[NUM_LEDS];

microLED strip(leds, NUM_LEDS, LED_PIN);

RF24 radio(46, 53);

iarduino_GPS_NMEA gps; 

iarduino_RTC watch(RTC_DS1307); 

Adafruit_BME280 bme;

Adafruit_APDS9960 apds;

SHT3x SHT; 

Adafruit_SGP30 sgp;

VR myVR(2,3);


#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define BLUEWHITE   0xEFFF


void setup() {
  Serial.begin(9600);
    uint32_t when = millis();
    uint16_t ID = tft.readID(); 
    Serial.print("ID = 0x");
    Serial.println(ID, HEX);
    if (ID == 0xD3D3) ID = 0x9481; // write-only shield
    tft.begin(ID);
    tft.setRotation(3); 
    tft.fillScreen(0xAD4E);
  tft.drawCircle(100,100,20,0xFFE0);
  tft.fillCircle(100,100,20,0x0000);
  tft.drawCircle(100,100,18,0xAD4E);
  tft.fillCircle(100,100,18,0xAD4E);
  tft.setCursor(0,6);
  tft.setTextColor(0x0000);
  tft.setTextSize(2);
  tft.println(" mode:        channel:");
  tft.println("");
  tft.println(" buttons: ");
  tft.println("");
  tft.println(" pot: ");
  tft.println("");
  tft.println(" gyro: ");
  tft.println("");
  tft.println(" mag: ");
  tft.println("");
  tft.println(" acs: ");
  tft.println("");
  tft.println(" JoyXY: ");
  //tft.println("mode: ");
  
}
void loop() {
  // put your main code here, to run repeatedly:

}