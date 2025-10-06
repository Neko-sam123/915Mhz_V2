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

extern uint32_t prevSeqNo;
extern uint32_t prevBroadcastID;
extern uint32_t prevMsgID;

extern uint16_t lastAckID;
extern uint32_t lastAckSource;


// interface
extern int interface_navigation; // just declare it
extern unsigned long currentT;
extern bool interface_navigation_disabler;
extern bool interface_UI_disabler;

extern int messageCount[5];
// GPS VARS //
extern float gpsLatitude;
extern float gpsLongitude;
extern float gpsAltitude;
extern uint8_t gpsSatellites;
extern uint8_t gpsHour;
extern uint8_t gpsMinute;
extern uint8_t gpsSecond;

extern double gpsSpeed;

extern double gpsCourse;
extern double gpsHDOP;


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
extern volatile bool rxDoneFlag;
extern volatile bool txDoneFlag;
extern volatile bool txActive;


extern unsigned long txStartTime;
//SD CARD
extern bool SD_CARD_READY;
extern bool GPS_Validity;
//battery 
extern int bat_percent;
extern float voltage_bat;
extern bool isCharging;
extern bool isLowBattery;
extern bool lowBlinkState;
// list ng node / address
extern char item_buffer[4][10];
extern int choose_ID;

// emergency setup




#endif