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

// LET THEREEE BEEE PINOUTTTT :)

// LCD (parallel interface goes to digital 2-9 pins)
///////////////////////
#define LCD_CS    A3 // Chip Select goes to Analog 3
#define LCD_CD    A2 // Command/Data goes to Analog 2
#define LCD_WR    A1 // LCD Write goes to Analog 1
#define LCD_RD    A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pi
///////////////////////

// NRF24L01+
//////////////////////
#define NRF_CE 28   //
#define NRF_CS 31   //
//////////////////////

// LoRa
////////////////////////
#define LORA_NSS  22  //
#define LORA_RST  23  //
#define LORA_DIO0 10  //
////////////////////////

// ESP
#define ESP_SERIAL Serial2

// Bluetooth
#define BLUETOOTH_SERIAL Serial1

// GPS
#define GPS_SERIAL Serial3

// mp3 player sotf. uart
////////////////////
#define MP3_TX 30 //
#define MP3_RX 29 //
////////////////////

// Voice recog. module
////////////////////
#define VR_TX 48  //
#define VR_RX 24  //
////////////////////

// LED (28 leds)
////////////////////////
#define LED_PIN 26    //
#define NUM_LEDS 28   // number of leds in strip
#define COLOR_DEBTH 3 // 3 - max
////////////////////////
// O2
#define O2_SENSOR A7

// EMG 
////////////////////////
#define EMG_AOUT A5   //
#define EMG_CS   A6   //
////////////////////////

// thermometr
#define DS18B20_PIN 13

// co2 sensor pwm pin
#define CO2_PWM 11

// Potentiomer pin
#define POT_PIN A14

// Joystick
////////////////////
#define JOY_X A15 //
#define JOY_Y A13 //
////////////////////
// buttons
const int buttons_pins[4]={A11, A9, A10, 27};

// switchers 
//////////////////////////////////////////////////////////
const int sw1_pins[8]={32, 33, 34, 35, 36, 37, 38, 39}; //
const int sw2_pins[8]={40, 41, 42, 43, 44, 45, 46, 47}; //
//////////////////////////////////////////////////////////

// baudrates (most of them depends on modules)
//////////////////////////////////////////
#define GPS_BAUDRATE           9600     //
#define ESP_BAUDRATE           115200   //
#define BLUETOOTH_BAUDRATE     9600     //
#define MP3_BAUDRATE           9600     //
#define VR_BAUDRATE            9600     //
#define COMMUNICATION_BAUDRATE 115200   //
//////////////////////////////////////////

// Some colors
////////////////////////////////
#define BLACK      0x0000     //
#define BLUE       0x001F     //
#define RED        0xF800     //
#define GREEN      0x07E0     //
#define CYAN       0x07FF     //  
#define MAGENTA    0xF81F     //
#define YELLOW     0xFFE0     //
#define DARKYELLOW 0x5AE3     //
#define WHITE      0xFFFF     //
#define BLUEWHITE  0xEFFF     //
////////////////////////////////

// NRF_SETTINGS
//////////////////////////////////////////
#define NRF_CHANNEL 103                 // you can choose in range 0-127
#define NRF_DATA_RATE RF24_250KBPS      // RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define NRF_PA_LEVEL RF24_PA_MAX        // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm
#define NRF_PIPE  0x1234567899LL        // 
#define NRF_AUTO_ASK false              //
#define NRF_ENABLE_READING_PIPE false   //
#define NRF_READING_PIPE 0x9987654321LL //
//////////////////////////////////////////

// LORA_SETTINGS
//////////////////////////////////
#define LORA_FR long(433E6)     // 433-435 (best)
#define LORA_CRC true           // 
#define LORA_CODING_RATE 8      // 5-11 (mb)
#define LORA_SIG_BW 250E3       // 250E3 - max, look at your lib .h file for more
#define LORA_SP_FACTOR 8        // 8 - best for RA-01 module (at least for mine)
//////////////////////////////////

#include <Arduino.h>              // default lib

////////////////////////////////////
#include <SPI.h>                  // interfaces
#include <Wire.h>                 //
#include <SoftwareSerial.h>       //
////////////////////////////////////

////////////////////////////////////
#include "Adafruit_GFX.h"         // for tft lcd
#include <MCUFRIEND_kbv.h>        //
////////////////////////////////////

////////////////////////////////////
#include <Adafruit_Sensor.h>      //
#include <Adafruit_LSM303_U.h>    // acsels + mag. sensor
#include <Adafruit_MPU6050.h>     //
////////////////////////////////////

#include "microLED.h"             // LED strip lib

#include <LoRa.h>                 // Simple LoRa lib

//////////////////////////
#include "nRF24L01.h"   // NRF lib files
#include "RF24.h"       //
#include "printf.h"     //
//////////////////////////

#include <iarduino_GPS_NMEA.h>    // as you can see - gps lib

#include <iarduino_RTC.h>         // for the clock module

#include <Adafruit_BME280.h>      // pressure (mostly) sensor 

#include "Adafruit_APDS9960.h"    // gesture sensor

#include <SHT3x.h>                // humid sensor

#include "Adafruit_SGP30.h"       // tVOC sensor

#include "VoiceRecognitionV3.h"   // VR module

#include <DFPlayer_Mini_Mp3.h>    // mp3

////////////////////////////////////////////////////////////////////////////////////////////////////////
MCUFRIEND_kbv tft;                                                                                    //
                                                                                                      //
////////////////////////////////////////////////////////////////////////////////                      //
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);         //                      //
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);   //                      //
Adafruit_MPU6050 mpu;                                                         //                      //
////////////////////////////////////////////////////////////////////////////////                      //
                                                                                                      //
//////////////////////////////////////////////////                                                    //
LEDdata leds[NUM_LEDS];                         //                                                    //
microLED strip(leds, NUM_LEDS, LED_PIN);        //                                                    //
//////////////////////////////////////////////////                                                    //
                                                                                                      //
RF24 radio(NRF_CS, NRF_CE);                                                                           //
                                                                                                      //
//////////////////////////////////                                                                    //
iarduino_GPS_NMEA gps;          //                                                                    //  
iarduino_RTC watch(RTC_DS1307); //                                                                    //
//////////////////////////////////                                                                    //  
                                                                                                      //  
//////////////////////////                                                                            //
Adafruit_BME280 bme;    //                                                                            //
Adafruit_APDS9960 apds; //                                                                            //
Adafruit_SGP30 sgp;     //                                                                            //
//////////////////////////                                                                            //
                                                                                                      //
SHT3x SHT;                                                                                            //
                                                                                                      //
VR myVR(VR_RX,VR_TX);                                                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////
struct wholeData              //
{                             //
  uint8_t mode;               // from swich 1
  uint8_t channel;            // from swich 2
  uint16_t button;            // 
  bool emg_active;            //
  bool error;                 //
  uint16_t joy_x;             //
  uint16_t joy_y;             //
  uint16_t pot;               //
  uint16_t flex_sensor_1;     //
  uint16_t flex_sensor_2;     //
  float gps_lon;              //
  float gps_lat;              //
  float pressure;             //
  float temp;                 //
  float tVOC;                 //
  float humid;                //
  long time;                  //
  int16_t acs_x;              //
  int16_t acs_y;              //
  int16_t acs_z;              //
  int8_t mag_x;               //
  int8_t mag_y;               //
  int8_t mag_z;               //
                              //
} allData;                    //
////////////////////////////////

////////////////////////////////
struct telemetri              //
{                             //
  uint8_t id=0;               //
  long time;                  //
  uint8_t mode;               //
  uint8_t channel;            //  
  uint16_t joy_x;             //
  uint16_t joy_y;             //
  uint16_t pot;               //
  uint16_t flex_sensor_1;     //
  uint16_t flex_sensor_2;     //
  int16_t acs_x;              //
  int16_t acs_y;              //
  int16_t acs_z;              //  
  int8_t mag_x;               //
  int8_t mag_y;               //
  int8_t mag_z;               //
} radioData;                  //
////////////////////////////////

void setup() 
{
  ///////////////////////////////////////////
  pinMode(NRF_CE,    OUTPUT);              //
  pinMode(NRF_CS,    OUTPUT);              //
  pinMode(LORA_NSS,  OUTPUT);              //
  pinMode(LORA_RST,  OUTPUT);              //
  pinMode(LORA_DIO0, INPUT);               //
  pinMode(MP3_TX,    INPUT);               //
  pinMode(MP3_RX,    OUTPUT);              //
  pinMode(VR_TX,     INPUT);               //
  pinMode(VR_RX,     OUTPUT);              //
  pinMode(LED_PIN,   OUTPUT);              //
  pinMode(O2_SENSOR, INPUT);               //
  pinMode(EMG_AOUT,  INPUT);               //
  pinMode(EMG_CS,    OUTPUT);              //
  pinMode(CO2_PWM,   INPUT);               //
  pinMode(POT_PIN,   INPUT);               //
  pinMode(JOY_X,     INPUT);               //
  pinMode(JOY_Y,     INPUT);               //
  pinMode(buttons_pins[0], INPUT_PULLUP);  //
  pinMode(buttons_pins[1], INPUT_PULLUP);  //
  pinMode(buttons_pins[2], INPUT_PULLUP);  //
  pinMode(buttons_pins[3], INPUT_PULLUP);  //
  for(int i = 0; i < 8; i++)               //
    pinMode(sw1_pins[i], INPUT_PULLUP);    //
  for(int i = 0; i < 8; i++)               // 
    pinMode(sw2_pins[i], INPUT_PULLUP);    //
  ///////////////////////////////////////////

  Serial.begin(COMMUNICATION_BAUDRATE);

  
}
void loop() {
  // put your main code here, to run repeatedly:

}