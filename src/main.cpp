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

#include <Arduino.h>              // default lib

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

// flex sesors
//////////////////////////
#define FLEX_1_PIN A12  //
#define FLEX_2_PIN A8   //
//////////////////////////

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
///////////////////////////////////
//#define СBLACK      0x0000     //
//#define СBLUE       0x001F     //
//#define СRED        0xF800     //
//#define СGREEN      0x07E0     //
//#define СCYAN       0x07FF     //  
//#define СMAGENTA    0xF81F     //
//#define СYELLOW     0xFFE0     //
//#define СDARKYELLOW 0x5AE3     //
//#define СWHITE      0xFFFF     //
//#define СBLUEWHITE  0xEFFF     //
///////////////////////////////////

// ds18b20 oneWire commands
//////////////////////////////////////////
#define OW_SKIP_ROM 0xCC                // 
#define OW_DS18B20_CONVERT_T 0x44       //
#define OW_DS18B20_READ_SCRATCHPAD 0xBE //
#define DS18B20_SCRATCHPAD_SIZE 9       //
//////////////////////////////////////////

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

////////////////////////////////////
#include <SPI.h>                  // interfaces
#include <Wire.h>                 //
#include <SoftwareSerial.h>       //
#include "OneWire.h"              //
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

#include <TinyGPS.h>              // as you can see - gps lib

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
RF24 radio(NRF_CE, NRF_CS);                                                                           //
                                                                                                      //
//////////////////////////////////                                                                    //
TinyGPS gps;                    //                                                                    //  
iarduino_RTC watch(2);          //                                                                    //
//////////////////////////////////                                                                    //  
                                                                                                      //
SoftwareSerial mp3Serial(MP3_RX, MP3_TX);                                                             //
                                                                                                      //  
//////////////////////////                                                                            //
Adafruit_BME280 bme;    //                                                                            //
Adafruit_APDS9960 apds; //                                                                            //
Adafruit_SGP30 sgp;     //                                                                            //
OneWire ds(DS18B20_PIN);//                                                                            //
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
  unsigned long time;         //
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
  unsigned long time;         //
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


//////////////////////////////
void sendNrf();             //
void sendLoRa(uint8_t msg); //
void sendBlt(uint8_t msg);             //
uint8_t getMode();          //
uint8_t getChannel();       //
uint16_t getButtons();      //
void getJoyData();          //
uint16_t getPotData();      //
uint8_t getGesture();       //
void readAcs();             //
void readMag();             //
void dispInfo();            //
uint16_t getCo2Data();      //
float getO2Data();          //
float getHumid();           //
uint16_t getTVOCdata();     //
void dispStrip();           //
void readEmg();             //
void getTime();             //
void getTemp();             //
void getGPSdata();          //
void getFlexSensorsData();  //
bool ds18b20_r_t(float & t);//
bool ds18b20_convert_t();   //
                            //
void mode1();               //
void mode2();               //
void mode3();               //
void mode4();               //
void mode5();               //
void mode6();               //
void mode7();               //
void mode8();               //
void mode9();               //
void mode10();              //
void mode11();              //
//////////////////////////////

uint16_t last_pot;
uint16_t last_joyX;
uint16_t last_joyY;
uint16_t last_button;
uint16_t last_channel;
uint16_t last_mod;
uint16_t last_acsx;
uint16_t last_acsy;
uint16_t last_acsz;
uint16_t last_magx;
uint16_t last_magy;
uint16_t last_magz;

uint32_t clean_timer;

uint32_t ds18b20_timer; 

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
  pinMode(POT_PIN,   INPUT_PULLUP);        //
  pinMode(JOY_X,     INPUT_PULLUP);        //
  pinMode(JOY_Y,     INPUT_PULLUP);        //
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
  
  //////////////////////////////////////////////////////// LoRa config
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);          // 
  if (!LoRa.begin(LORA_FR))                             // 
  {                                                     //
    Serial.println("Starting LoRa failed!");            //
  }                                                     //
  else                                                  //
  {                                                     //
    Serial.println("LoRa started sucsessfully");        //
    LoRa.enableCrc();                                   //
    LoRa.setCodingRate4(LORA_CODING_RATE);              //
    LoRa.setSignalBandwidth(LORA_SIG_BW);               //
    LoRa.setSpreadingFactor(LORA_SP_FACTOR);            //
    LoRa.explicitHeaderMode();                          //
  }                                                     //
  ////////////////////////////////////////////////////////

  ////////////////////////////////////// NRF config (you should check it in #define part)
  printf_begin();                     //
  radio.begin();                      //
  radio.setChannel(NRF_CHANNEL);      //
  radio.setDataRate(NRF_DATA_RATE);   //
  radio.setPALevel(NRF_PA_LEVEL);     //
  radio.openWritingPipe(NRF_PIPE);    //
  radio.setAutoAck(NRF_AUTO_ASK);     //
  radio.printDetails();               //
  //////////////////////////////////////

  ESP_SERIAL.begin(ESP_BAUDRATE);
  BLUETOOTH_SERIAL.begin(BLUETOOTH_BAUDRATE);
  GPS_SERIAL.begin(GPS_BAUDRATE);
  mp3Serial.begin(9600);
  myVR.begin(9600);

  uint16_t ID = tft.readID(); 
  Serial.print("ID = 0x");
  Serial.println(ID, HEX);
  if (ID == 0xD3D3) ID = 0x9481; // write-only shield
  tft.begin(ID);
  tft.setRotation(3); 
  tft.fillScreen(TFT_YELLOW);

  mp3_set_serial (mp3Serial);    
  mp3_set_volume (15);

  //if (!mpu.begin()) {
  //  Serial.println("Failed to find MPU6050 chip");
  //}

  if(!apds.begin()){
    Serial.println("failed to initialize APDS9960.");
  }
  apds.enableProximity(true);
  apds.enableGesture(true);

  mag.enableAutoRange(true);
  if(!mag.begin()){
    Serial.println("No LSM303 detected.");
  }
  if(!accel.begin()){
    Serial.println("No LSM303 detected.");
  }

  strip.setBrightness(30);
  strip.fill(YELLOW);
  strip.show();

  watch.begin();

  unsigned status;
    
  // default settings
  status = bme.begin();  
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }

  sgp.begin();
  SHT.Begin();




}
void loop() 
{
  readAcs();
  getMode();
  getChannel();
  getButtons();
  getJoyData();
  getPotData();
  readMag();
  dispInfo();

}

void sendNrf()
{
  radioData.time=allData.time;
  radioData.mode=allData.mode;
  radioData.channel=allData.channel;
  radioData.joy_x=allData.joy_x;
  radioData.joy_y=allData.joy_y;
  radioData.pot=allData.pot;
  radioData.flex_sensor_1=allData.flex_sensor_1;
  radioData.flex_sensor_2=allData.flex_sensor_2;
  radioData.acs_x=allData.acs_x;
  radioData.acs_y=allData.acs_y;
  radioData.acs_z=allData.acs_z;
  radioData.mag_x=allData.mag_x;
  radioData.mag_y=allData.mag_y;
  radioData.mag_z=allData.mag_z;
  radio.write(&radioData, sizeof(radioData));
}            
void sendLoRa(uint8_t msg)
{
  LoRa.beginPacket();
  switch (msg)
  {
  case 0:
    LoRa.print(allData.gps_lat, 7);
    LoRa.print(" ");
    LoRa.print(allData.gps_lon, 7);
    break;
  case 1:
    LoRa.print(allData.acs_x);
    LoRa.print(" ");
    LoRa.print(allData.acs_y);
    LoRa.print(" ");
    LoRa.print(allData.acs_z);
    break;
  case 2:
    LoRa.print(allData.mag_x);
    LoRa.print(" ");
    LoRa.print(allData.mag_y);
    LoRa.print(" ");
    LoRa.print(allData.mag_z);
    break;
  case 3:
    LoRa.print(allData.pot);
    LoRa.print(" ");
    LoRa.print(allData.joy_x);
    LoRa.print(" ");
    LoRa.print(allData.joy_y);
    break;
  case 4:
    LoRa.print(allData.mode);
    LoRa.print(" ");
    LoRa.print(allData.channel);
    LoRa.print(" ");
    LoRa.print(allData.button);
    break;
  }
  LoRa.endPacket(1);
}       

void sendBlt(uint8_t msg)
{
  switch (msg)
  {
  case 0:
    BLUETOOTH_SERIAL.println(millis());
    BLUETOOTH_SERIAL.println(allData.mode);
    BLUETOOTH_SERIAL.println(allData.channel);
    BLUETOOTH_SERIAL.println(allData.button);
    BLUETOOTH_SERIAL.println();
    BLUETOOTH_SERIAL.println();
    break;
  case 1:
    BLUETOOTH_SERIAL.println(millis());
    BLUETOOTH_SERIAL.println(allData.joy_x);
    BLUETOOTH_SERIAL.println(allData.joy_y);
    BLUETOOTH_SERIAL.println(allData.pot);
    BLUETOOTH_SERIAL.println();
    BLUETOOTH_SERIAL.println();
    break;
  case 2:
    BLUETOOTH_SERIAL.println(millis());
    BLUETOOTH_SERIAL.println(allData.gps_lon);
    BLUETOOTH_SERIAL.println(allData.gps_lat);
    BLUETOOTH_SERIAL.println();
    BLUETOOTH_SERIAL.println();
    break;
  case 3:
    BLUETOOTH_SERIAL.println(millis());
    BLUETOOTH_SERIAL.println(allData.temp);
    BLUETOOTH_SERIAL.println(allData.pressure);
    BLUETOOTH_SERIAL.println(allData.humid);
    BLUETOOTH_SERIAL.println();
    BLUETOOTH_SERIAL.println();
    break;
  case 4:
    BLUETOOTH_SERIAL.println(millis());
    BLUETOOTH_SERIAL.println(allData.tVOC);
    BLUETOOTH_SERIAL.println(allData.emg_active);
    BLUETOOTH_SERIAL.println(allData.flex_sensor_1);
    BLUETOOTH_SERIAL.println(allData.flex_sensor_2);
    BLUETOOTH_SERIAL.println();
    BLUETOOTH_SERIAL.println();
    break;
  case 5:
    BLUETOOTH_SERIAL.println(millis());
    BLUETOOTH_SERIAL.println(allData.mode);
    BLUETOOTH_SERIAL.println(allData.channel);
    BLUETOOTH_SERIAL.println(allData.button);
    BLUETOOTH_SERIAL.println();
    BLUETOOTH_SERIAL.println();
    break;
  case 6:
    BLUETOOTH_SERIAL.println(millis());
    BLUETOOTH_SERIAL.println(allData.acs_x);
    BLUETOOTH_SERIAL.println(allData.acs_y);
    BLUETOOTH_SERIAL.println(allData.acs_z);
    BLUETOOTH_SERIAL.println();
    BLUETOOTH_SERIAL.println();
    break;
  case 7:
    BLUETOOTH_SERIAL.println(millis());
    BLUETOOTH_SERIAL.println(allData.mag_x);
    BLUETOOTH_SERIAL.println(allData.mag_y);
    BLUETOOTH_SERIAL.println(allData.mag_z);
    BLUETOOTH_SERIAL.println();
    BLUETOOTH_SERIAL.println();
    break;
  
  }
}

uint8_t getMode()
{
  bitWrite(allData.mode, 0, (!digitalRead(sw1_pins[0])));
  bitWrite(allData.mode, 1, (!digitalRead(sw1_pins[1])));
  bitWrite(allData.mode, 2, (!digitalRead(sw1_pins[2])));
  bitWrite(allData.mode, 3, (!digitalRead(sw1_pins[3])));
  bitWrite(allData.mode, 4, (!digitalRead(sw1_pins[4])));
  bitWrite(allData.mode, 5, (!digitalRead(sw1_pins[5])));
  bitWrite(allData.mode, 6, (!digitalRead(sw1_pins[6])));
  bitWrite(allData.mode, 7, (!digitalRead(sw1_pins[7])));
  return allData.mode;                         // for debug
}         

uint8_t getChannel()
{
  bitWrite(allData.channel, 0, (!digitalRead(sw2_pins[0])));
  bitWrite(allData.channel, 1, (!digitalRead(sw2_pins[1])));
  bitWrite(allData.channel, 2, (!digitalRead(sw2_pins[2])));
  bitWrite(allData.channel, 3, (!digitalRead(sw2_pins[3])));
  bitWrite(allData.channel, 4, (!digitalRead(sw2_pins[4])));
  bitWrite(allData.channel, 5, (!digitalRead(sw2_pins[5])));
  bitWrite(allData.channel, 6, (!digitalRead(sw2_pins[6])));
  bitWrite(allData.channel, 7, (!digitalRead(sw2_pins[7])));
  return allData.channel;                         // for debug
}       

uint16_t getButtons()
{
  allData.button = 1000*!digitalRead(buttons_pins[0])+
                   100* !digitalRead(buttons_pins[1])+
                   10 * !digitalRead(buttons_pins[2])+
                   1 * !digitalRead( buttons_pins[3]);
  return allData.button;
}      

void getJoyData()
{
  //uint32_t x;
  //uint32_t y;
  //for(int i = 0; i<4; i++)
  //{
  //  x+=analogRead(JOY_X);
  //  y+=analogRead(JOY_Y);
  //}
  //x/=4;                  
  //y/=4;
  
  allData.joy_x = analogRead(JOY_X);
  allData.joy_y = analogRead(JOY_Y);
}        

uint16_t getPotData()
{
  //uint32_t potVal;
  //for(int i = 0; i<4; i++)
  //  potVal += analogRead(POT_PIN);
  //potVal /= 4;
  allData.pot = analogRead(POT_PIN);
  return allData.pot;
}      

uint8_t getGesture()
{
  uint8_t prox = apds.readProximity();
  return prox;
}       

void readAcs()
{
  sensors_event_t event;
  accel.getEvent(&event);
  allData.acs_x = event.acceleration.x;
  allData.acs_y = event.acceleration.y;
  allData.acs_z = event.acceleration.z;
}             

void readMag()
{
  sensors_event_t event;
  mag.getEvent(&event);
  allData.mag_x = event.magnetic.x;
  allData.mag_y = event.magnetic.y;
  allData.mag_z = event.magnetic.z;
  
}

void dispInfo()
{
  tft.setCursor(0,6);
  tft.setTextColor(0x0000);
  tft.setTextSize(2);
  tft.print(" mode/channel: ");
  tft.setCursor(200,6);
  tft.setTextColor(TFT_YELLOW);
  tft.print(last_mod);
  tft.setCursor(200,6);
  tft.setTextColor(0x0000);
  tft.print(allData.mode);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(260,6);
  tft.print(last_channel);
  tft.setTextColor(0x0000);
  tft.setCursor(260,6);
  tft.print(allData.channel);

  tft.setCursor(0,35);
  tft.print(" buttons: ");
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(120,35);
  tft.print(last_button);
  tft.setCursor(120,35);
  tft.setTextColor(0x0000);
  tft.println(allData.button);

  tft.setCursor(0,64);
  tft.print(" pot: ");
  tft.setCursor(90,64);
  tft.setTextColor(TFT_YELLOW);
  tft.print(last_pot);
  tft.setCursor(90,64);
  tft.setTextColor(0x0000);
  tft.println(allData.pot);

  tft.setCursor(0,93);
  tft.print(" mag: ");
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(90,93);
  tft.print(last_magx);
  tft.setTextColor(0x0000);
  tft.setCursor(90,93);
  tft.print(allData.mag_x);
  tft.print(" ");
  tft.setCursor(150,93);
  tft.setTextColor(TFT_YELLOW);
  tft.print(last_magy);
  tft.setCursor(150,93);
  tft.setTextColor(0x0000);
  tft.print(allData.mag_y);
  tft.print(" ");
  tft.setCursor(210,93);
  tft.setTextColor(TFT_YELLOW);
  tft.print(last_magz);
  tft.setTextColor(0x0000);
  tft.setCursor(210,93);
  tft.println(allData.mag_z);

  tft.setCursor(0,122);
  tft.print(" acs: ");
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(90,122);
  tft.print(last_acsx);
  tft.setCursor(90,122);
  tft.setTextColor(0x0000);
  tft.print(allData.acs_x);
  tft.print(" ");
  tft.setCursor(150,122);
  tft.setTextColor(TFT_YELLOW);
  tft.print(last_acsy);
  tft.setCursor(150,122);
  tft.setTextColor(0x0000);
  tft.print(allData.acs_y);
  tft.print(" ");
  tft.setCursor(180,122);
  tft.setTextColor(TFT_YELLOW);
  tft.print(last_acsz);
  tft.setCursor(180,122);
  tft.setTextColor(0x0000);
  tft.println(allData.acs_z);
  tft.println("");

  tft.setCursor(0,151);
  tft.print(" JoyXY: ");
  tft.setCursor(90,151);
  tft.setTextColor(TFT_YELLOW);
  tft.print(last_joyX);
  tft.setCursor(90,151);
  tft.setTextColor(0x0000);
  tft.print(allData.joy_x);
  tft.print(" ");
  tft.setCursor(160,151);
  tft.setTextColor(TFT_YELLOW);
  tft.print(last_joyY);
  tft.setCursor(160,151);
  tft.setTextColor(0x0000);
  tft.print(allData.joy_y);
  last_pot=allData.pot;
  last_joyX=allData.joy_x;
  last_joyY=allData.joy_y;
  last_button=allData.button;
  last_channel=allData.channel;
  last_mod=allData.mode;
  last_acsx=allData.acs_x;
  last_acsy=allData.acs_y;
  last_acsz=allData.acs_z;
  last_magx=allData.mag_x;
  last_magy=allData.mag_y;
  last_magz=allData.mag_z;
  if(millis()>=clean_timer)
  {
    tft.fillScreen(TFT_YELLOW);
    clean_timer+=3000;
  }
}      

uint16_t getCo2Data()
{
  return 0;
}     

float getO2Data()
{
  return 0;
}          

float getHumid()
{
  return 0;
}    

uint16_t getTVOCdata()
{
  return 0;
}     

void dispStrip()
{
  strip.fill(mCOLOR(YELLOW)); // заливаем жёлтым
  strip.show();
} 

void readEmg()
{
  // for future features
  const int maxCalm = 100;
  bool emgActive;
  for(int i = 0; i<5; i++)
  {
    if(analogRead(EMG_AOUT)>maxCalm+1)
    {
      emgActive=1;
      return;  
    }
  }
} 

void getTime()
{
  // 2 ways, first is from the RTC module 
  // but because of broken battary on rtc i will make it right later
  Serial.println(watch.gettime("d-m-Y, H:i:s, D")); //(all data) 

}  

void getTemp()
{
  if (millis() / 100 >= ds18b20_timer)    //  
  {                                       //
    if (millis() / 400 != 0)              //  
    {                                     //                                                                                    
      float temp;                         //                                                                                                                                                                        |
      if (ds18b20_r_t(temp))           //                                                                                    
        allData.temp = temp;             //                                                                                    
      else                                //                                                                                    
        allData.temp = NAN;              //          
    }                                     //    
    ds18b20_timer = millis() / 100 + 10;  //                                                                                    
    ds18b20_convert_t();                  // 
  }
}   

bool ds18b20_convert_t()
{
  if (!ds.reset()) // даем reset на шину
  {
    return false;
  }
  ds.write(OW_SKIP_ROM, 1);
  ds.write(OW_DS18B20_CONVERT_T, 1);
  return true;
}
bool ds18b20_r_t(float & t)
{
  if (!ds.reset()) // даем резет на шину
  { 
    return false;
  }
  ds.write(OW_SKIP_ROM, 1); // skip adressing
  uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
  ds.write(OW_DS18B20_READ_SCRATCHPAD, 1);
  ds.read_bytes(scratchpad, sizeof(scratchpad));
  uint8_t crc_actual = scratchpad[DS18B20_SCRATCHPAD_SIZE - 1]; // crc
  uint8_t crc_calculated = OneWire::crc8(scratchpad, DS18B20_SCRATCHPAD_SIZE - 1); // calc crc
  float temp;
  if (crc_calculated != crc_actual)
  {
    return false;
  }
  uint16_t uraw_temp;
  uraw_temp = scratchpad[0] | (static_cast<uint16_t>(scratchpad[1]) << 8);
  int16_t raw_temp;
  memcpy(&raw_temp, &uraw_temp, sizeof(raw_temp));
  temp = raw_temp / 16.f;
  t = temp;
  return true;
}

void getGPSdata()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPS_SERIAL.available())
    {
      char c = GPS_SERIAL.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    allData.gps_lon=flon;
    allData.gps_lat=flat;
    gps.get_datetime(&allData.time, &age);
  }
}  

void getFlexSensorsData()
{
  uint16_t f1, f2;
  for(int i = 0; i<4; i++)
  {
    f1+=analogRead(FLEX_1_PIN);
    f2+=analogRead(FLEX_2_PIN); 
  }
  f1 >>= 2;
  f2 >>= 2;
  allData.flex_sensor_1=f1;
  allData.flex_sensor_2=f2;
  
}  
                           
void mode1()
{
  
}   

void mode2()
{
  
}   

void mode3()
{
  
}    

void mode4()
{
  
} 

void mode5()
{
  
}    

void mode6()
{
  
} 

void mode7()
{
  
}               
void mode8()
{
  
} 

void mode9()
{
  
} 

void mode10()
{
  
}   

void mode11()
{
  
}              