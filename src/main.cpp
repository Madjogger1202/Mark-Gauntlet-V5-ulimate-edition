#include <Arduino.h>

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

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

uint16_t g_identifier;


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