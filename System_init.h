#ifndef System_init_H
#define System_init_H

#include <Arduino.h>

//#include <LoRa.h>
#include <RadioLib.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/TomThumb.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <XPowersLib.h>

#include "global.h"
#include "Messaging.h"


// GPS 
extern TinyGPSPlus gps;
extern HardwareSerial gpsSerial;  

// OLED Display
extern Adafruit_SSD1306 display;

// HSPI for SD card (shared with TFT)
extern SPIClass hspi;

// LORA VSPI
extern SPIClass spiLoRa;   
extern SX1262 radio;        

// power management
extern XPowersAXP2101 PMU;

class System
{
private:
  Node node;
  Message message;

public:


  System () {};

  void init();

  void sys_loop();

  void sendUBX(uint8_t *msg, uint8_t len);
  void enableGPS_PSM();
  void enableGPS_FullPower();

  void checkGPSFix();

  void Emergency_Broadcast();

};


#endif