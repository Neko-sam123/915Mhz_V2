#include "System_init.h"
#include "Interface.h"
#include "global.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

System sys;
UInterface UI;

unsigned long currentT;

void setup() {
  sys.init();
  
}

void loop() {
  currentT = millis();
  sys.sys_loop();
  UI.draw_messageNav();
}
