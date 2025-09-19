#ifndef global_H
#define global_H

#define SD_CS 15  // or whatever pin you're using for SD
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define KB_ADDR 0x5F

// aodv variables
extern uint32_t myAddress;  // first node

extern uint32_t mySeqNo;
extern uint32_t lastBroadcastID;
extern uint16_t lastMsgID;

extern uint16_t lastAckID;
extern uint32_t lastAckSource;


// interface
extern int interface_navigation; // just declare it
extern unsigned long currentT;
extern bool interface_navigation_disabler;
extern bool interface_UI_disabler;

// GPS VARS //
extern float gpsLatitude;
extern float gpsLongitude;
extern float gpsAltitude;
extern uint8_t gpsSatellites;
extern double gpsSpeed;
extern double gpsCourse;
extern double gpsHDOP;
extern uint8_t gpsHour;
extern uint8_t gpsMinute;
extern uint8_t gpsSecond;

extern char gps_value_buffer[8][15];

// LORA SETTINGS
extern const float myFreq;
extern float bandwidth;
extern uint8_t sf;
extern uint8_t cr;
extern uint8_t syncWord;
extern int8_t power;
extern uint16_t preamble;

// RADIO LIB
extern volatile bool packetFlag;

//SD CARD
extern bool SD_CARD_READY;

//battery 

extern float voltage_bat;

extern char item_buffer[5][10];



#endif