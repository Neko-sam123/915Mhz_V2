#ifndef Interface_H
#define Interface_H

#include <Arduino.h>
#include "global.h"
#include "Messaging.h"
#include "SnakeGame.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/TomThumb.h>
#include <math.h> 

extern Adafruit_SSD1306 display; // extern is used in .h file

///////////////////////////////////////////////////////////////////
///////////////////////////message frames /////////////////////////
///////////////////////////////////////////////////////////////////
    extern const unsigned char epd_bitmap_frame_00_delay_0[];
    extern const unsigned char epd_bitmap_frame_01_delay_0[];
    extern const unsigned char epd_bitmap_frame_02_delay_0[];
    extern const unsigned char epd_bitmap_frame_03_delay_0[];
    extern const unsigned char epd_bitmap_frame_04_delay_0[];
    extern const unsigned char epd_bitmap_frame_05_delay_0[];
    extern const unsigned char epd_bitmap_frame_06_delay_0[];
    extern const unsigned char epd_bitmap_frame_07_delay_0[];
    extern const unsigned char epd_bitmap_frame_08_delay_0[];
    extern const unsigned char epd_bitmap_frame_09_delay_0[];
    extern const unsigned char epd_bitmap_frame_10_delay_0[];
    extern const unsigned char epd_bitmap_frame_11_delay_0[];

    extern const int epd_bitmap_allArray_LEN;
    extern const unsigned char* message_animation[];


///////////////////////////////////////////////////////////////////
///////////////////////////message frames /////////////////////////
///////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// gps - frames /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

    extern const unsigned char gpsframe_0_delay_0[];
    extern const unsigned char gpsframe_1_delay_0[];
    extern const unsigned char gpsframe_2_delay_0[];
    extern const unsigned char gpsframe_3_delay_0[];
    extern const unsigned char gpsframe_4_delay_0[];
    extern const unsigned char gpsframe_5_delay_0[];
    extern const unsigned char gpsframe_6_delay_0[];
    extern const unsigned char gpsframe_7_delay_0[];
    extern const unsigned char gpsframe_8_delay_0[];

    extern const int gpsallArray_LEN;
    extern const unsigned char* gpsallArray[];
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// gps - frames /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// settings - frames ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
    extern const unsigned char settingsframe_00_delay_0[];
    extern const unsigned char settingsframe_01_delay_0[];
    extern const unsigned char settingsframe_02_delay_0[];
    extern const unsigned char settingsframe_03_delay_0[];
    extern const unsigned char settingsframe_04_delay_0[];
    extern const unsigned char settingsframe_05_delay_0[];
    extern const unsigned char settingsframe_06_delay_0[];
    extern const unsigned char settingsframe_07_delay_0[];
    extern const unsigned char settingsframe_08_delay_0[];
    extern const unsigned char settingsframe_09_delay_0[];
    extern const unsigned char settingsframe_10_delay_0[];
    extern const unsigned char settingsframe_11_delay_0[];

    extern const int settingsallArray_LEN;
    extern const unsigned char* settingsallArray[];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// settings - frames ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Memory    Card    ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

    extern const unsigned char myBitmapmemory_card[];

    extern const int myBitmapallArray_LEN;
    extern const unsigned char* myBitmapallArray[];
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Memory    Card    ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// GPS       ICON    ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

    extern const unsigned char epd_bitmap_GPS_ICON[];

    extern const int GPS_ICON_LEN;
    extern const unsigned char* epd_bitmap_allArray[];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// GPS       ICON    ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// GPS    Loading    ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

    extern const unsigned char GPS_loadingframe_00_delay_0[];
    extern const unsigned char GPS_loadingframe_01_delay_0[];
    extern const unsigned char GPS_loadingframe_02_delay_0[];
    extern const unsigned char GPS_loadingframe_03_delay_0[];
    extern const unsigned char GPS_loadingframe_04_delay_0[];
    extern const unsigned char GPS_loadingframe_05_delay_0[];
    extern const unsigned char GPS_loadingframe_06_delay_0[];
    extern const unsigned char GPS_loadingframe_07_delay_0[];
    extern const unsigned char GPS_loadingframe_08_delay_0[];
    extern const unsigned char GPS_loadingframe_09_delay_0[];
    extern const unsigned char GPS_loadingframe_10_delay_0[];
    extern const unsigned char GPS_loadingframe_11_delay_0[];
    extern const unsigned char GPS_loadingframe_12_delay_0[];
    extern const unsigned char GPS_loadingframe_13_delay_0[];
    extern const unsigned char GPS_loadingframe_14_delay_0[];
    extern const unsigned char GPS_loadingframe_15_delay_0[];
    extern const unsigned char GPS_loadingframe_16_delay_0[];
    extern const unsigned char GPS_loadingframe_17_delay_0[];
    extern const unsigned char GPS_loadingframe_18_delay_0[];
    extern const unsigned char GPS_loadingframe_19_delay_0[];
    extern const unsigned char GPS_loadingframe_20_delay_0[];
    extern const unsigned char GPS_loadingframe_21_delay_0[];
    extern const unsigned char GPS_loadingframe_22_delay_0[];
    extern const unsigned char GPS_loadingframe_23_delay_0[];
    extern const unsigned char GPS_loadingframe_24_delay_0[];
    extern const unsigned char GPS_loadingframe_25_delay_0[];
    extern const unsigned char GPS_loadingframe_26_delay_0[];
    extern const unsigned char GPS_loadingframe_27_delay_0[];
    extern const unsigned char GPS_loadingframe_28_delay_0[];
    extern const unsigned char GPS_loadingframe_29_delay_0[];

    extern const int GPS_loadingallArray_LEN;
    extern const unsigned char* GPS_loadingallArray[];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// GPS    Loading    ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////arrows.//////////////////////////////
    extern const uint8_t arrowUp_6x4[];
    extern const uint8_t arrowDown_6x4[];
//////////////////////////////arrows.//////////////////////////////


class UInterface
{
  private:
  Message message;
  Node node;
  AODV_VAR aodv;
  SnakeGame snake;

  bool message_clear = false; // refresh if out
  bool gps_clear = false;
  bool settings_clear = false;
  int index = 0; // refresh if out

  unsigned long draw_messageT = 0;
  unsigned long m5stackT = 0;

  bool blinkState = false;  
	unsigned long lastBlink = 0;
  const int blinkInterval = 500;

  //node select
  int selection_value = 0;
  bool selection_disabler = false;

  // GPS Select
  bool GPS_disabler = true;
  int GPS_node_select = sizeof(item_buffer) / sizeof(item_buffer[0]);
  #define EARTH_RADIUS 6371000.0
  //float PI = 3.14159265;
  bool GPS_node_array_bool[sizeof(item_buffer) / sizeof(item_buffer[0])] = {0};
  unsigned long GPS_loading_arrayT[sizeof(item_buffer) / sizeof(item_buffer[0])] = {0};
  
////////////////// message ////////////////////////////

  int text_cursor_height_array[5] =
      {
        27,
        39,
        51,
        63
      };

////////////////// message ////////////////////////////

//////////////////   GPS   ////////////////////////////


//////////////////   GPS   ////////////////////////////
  public:

  void draw_messageNav();
  void m5_stack();

  void node_select();
  void gps_display();
  void settings_display();

  void draw_animation(const unsigned char** frames,const int frame_Num);
  void centerText(const char* text, int y , int width, int startX);

  void drawUpArrow(int x, int y);
  void drawDownArrow(int x, int y);

  void drawUpArrowRec();
  void drawDownArrowRec();
  
  void arrow_blinking(int x,int y ,int dulo);

  void drawBatteryIcon13px(int x, int y);
  
  float computeDistance(float lat1, float lon1, float lat2, float lon2);
  float deg2rad(float deg);

  float computeBearing(float lat1, float lon1, float lat2, float lon2);
  void drawCompass(float myLat, float myLon, float targetLat, float targetLon, float heading, bool moving);
};



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// SNAKE    FRAME    ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
    extern const unsigned char SNAKEframe_00_delay_0[];
    extern const unsigned char SNAKEframe_01_delay_0[];
    extern const unsigned char SNAKEframe_02_delay_0[];
    extern const unsigned char SNAKEframe_03_delay_0[];
    extern const unsigned char SNAKEframe_04_delay_0[];
    extern const unsigned char SNAKEframe_05_delay_0[];
    extern const unsigned char SNAKEframe_06_delay_0[];
    extern const unsigned char SNAKEframe_07_delay_0[];
    extern const unsigned char SNAKEframe_08_delay_0[];
    extern const unsigned char SNAKEframe_09_delay_0[];
    extern const unsigned char SNAKEframe_10_delay_0[];
    extern const unsigned char SNAKEframe_11_delay_0[];
    extern const unsigned char SNAKEframe_12_delay_0[];
    extern const unsigned char SNAKEframe_13_delay_0[];
    extern const unsigned char SNAKEframe_14_delay_0[];
    extern const unsigned char SNAKEframe_15_delay_0[];
    extern const unsigned char SNAKEframe_16_delay_0[];

    extern const int SNAKEallArray_LEN;
    extern const unsigned char* SNAKEallArray[];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// SNAKE    FRAME    ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif