#ifndef Messaging_H
#define Messaging_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
//#include <LoRa.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/TomThumb.h>
#include <SD.h>

#include "AODV.h"
#include "global.h"


// OLED Display
extern Adafruit_SSD1306 display;

// HSPI for SD card (shared with TFT)
extern SPIClass hspi;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// loading - frames  ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

  extern const unsigned char loadingframe_00_delay_0[];
  extern const unsigned char loadingframe_01_delay_0[];
  extern const unsigned char loadingframe_02_delay_0[];
  extern const unsigned char loadingframe_03_delay_0[];
  extern const unsigned char loadingframe_04_delay_0[];
  extern const unsigned char loadingframe_05_delay_0[];
  extern const unsigned char loadingframe_06_delay_0[];
  extern const unsigned char loadingframe_07_delay_0[];
  extern const unsigned char loadingframe_08_delay_0[];
  extern const unsigned char loadingframe_09_delay_0[];
  extern const unsigned char loadingframe_10_delay_0[];
  extern const unsigned char loadingframe_11_delay_0[];
  extern const unsigned char loadingframe_12_delay_0[];
  extern const unsigned char loadingframe_13_delay_0[];
  extern const unsigned char loadingframe_14_delay_0[];
  extern const unsigned char loadingframe_15_delay_0[];
  extern const unsigned char loadingframe_16_delay_0[];
  extern const unsigned char loadingframe_17_delay_0[];
  extern const unsigned char loadingframe_18_delay_0[];
  extern const unsigned char loadingframe_19_delay_0[];
  extern const unsigned char loadingframe_20_delay_0[];
  extern const unsigned char loadingframe_21_delay_0[];
  extern const unsigned char loadingframe_22_delay_0[];
  extern const unsigned char loadingframe_23_delay_0[];
  extern const unsigned char loadingframe_24_delay_0[];
  extern const unsigned char loadingframe_25_delay_0[];

  extern const int loadingallArray_LEN;
  extern const unsigned char* loadingallArray[];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// loading - frames  ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Message{
private: 
  Node node;
  AODV_VAR aodv;

  uint32_t destAddr;

  char message_payload[32];

  #define MAX_MESSAGES 100
  #define VISIBLE_MESSAGES 2
  #define MAX_MSG_LENGTH 40
  #define MAX_INPUT_LENGTH 33 // para sa ack

  char messages[MAX_MESSAGES][MAX_MSG_LENGTH + 1];
  char currentInput[MAX_INPUT_LENGTH + 1];
  uint8_t inputLength = 0;
  uint8_t scrollOffset = 0;
  bool blinkState = false;
  unsigned long lastBlink = 0;
  const int blinkInterval = 500;
  unsigned long RREQinterval = 0;
 
public:
  
  Message () {}

  void init();

  void select_addr(uint32_t destAddr);

  void sendMessage();   // ito papalitan pa basta

  void handleKeyboard();
  
  void handlePayloadReceive(const char* payload, uint32_t destAddr);
  char getKey();
  void addMessage(const char* msg);
  void drawChat();
  void saveMessageToSD(const char* msg, uint32_t destAddr);
  void loadMessagesFromSD(uint32_t destAddr);
    // loading animation lang
  void draw_animation(const unsigned char** frames,const int frame_Num);
};





#endif