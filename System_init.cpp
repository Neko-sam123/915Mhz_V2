#include "System_init.h"

SPIClass hspi(HSPI);
bool SD_CARD_READY = false;

SPIClass spiLoRa(VSPI);

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  


float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsAltitude = 0.0;
uint8_t gpsSatellites = 0;
double gpsSpeed      = 0.0; // km/h
double gpsCourse     = 0.0; // degrees
double gpsHDOP       = 0.0;
uint8_t gpsHour      = 0;
uint8_t gpsMinute    = 0;
uint8_t gpsSecond    = 0;
char gps_value_buffer[8][15];


/////// RADIOLIB SETUP //////////////////
#define CS_PIN   18
#define DIO1_PIN 33   // V1.1
#define RST_PIN  23
#define BUSY_PIN 32
SX1262 radio = new Module(CS_PIN, DIO1_PIN, RST_PIN, BUSY_PIN, spiLoRa);

volatile bool packetFlag = false;
void IRAM_ATTR onPacket() {
  // set flag only (keep ISR short)
  packetFlag = true;
}


const float myFreq  = 915.0;   // MHz
float bandwidth     = 125.0;   // kHz
uint8_t sf          = 9;       // spreading factor
uint8_t cr          = 7;       // coding rate
uint8_t syncWord    = 0x12;    // sync word
int8_t power        = 17;      // output power dBm
uint16_t preamble   = 8;       // preamble length

///////////// battery setup /////////////////
#define BATTERY_PIN 35
#define DIVIDER_RATIO 2.0
// ESP32 ADC reference voltage (3.3V)
#define ADC_REF 3.3
#define ADC_RES 4095.0

float voltage_bat = 4.2;

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float voltage = (raw / ADC_RES) * ADC_REF * DIVIDER_RATIO;
  return voltage;
}


void System::init() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // OLED Init once
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found!");
    while (true);
  }
  display.clearDisplay();
  display.setFont(&TomThumb);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();

  // LoRa Init
  spiLoRa.begin(5, 19, 27, CS_PIN);
  int st = radio.begin(myFreq, bandwidth, sf, cr, syncWord, power, preamble);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("radio.begin failed: "); Serial.println(st);
    while(true) delay(500);
  }
  Serial.println("TX radio init OK");
  Serial.print("Using DIO1 pin: ");
  Serial.println(DIO1_PIN);

  // Attach ISR for RX done
  radio.setDio1Action(onPacket);
  // Clear any stale IRQ flags
  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  // Start listening
  radio.startReceive();

  // SD Init
  hspi.begin(14, 25, 13, SD_CS);
  if (!SD.begin(SD_CS, hspi)) {
    Serial.println("SD Card fail");
    SD_CARD_READY = false;
  } else {
    SD_CARD_READY = true;
    Serial.println("SD Ready");
  }

  //gps init
  gpsSerial.begin(9600, SERIAL_8N1, 34, 12); // e.g., (9600, SERIAL_8N1, 34, 12)

  //loading routing table and node seq#, messageID 
  message.init();

}


void System::sys_loop(){
   node.receivePacket(); 
   node.update_Expiry_time();

  static unsigned long GPS_T = 0;

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    gpsLatitude   = gps.location.lat();
    gpsLongitude  = gps.location.lng();
    gpsAltitude   = gps.altitude.meters();
    gpsSatellites = gps.satellites.value();
    gpsHour       = gps.time.hour();
    gpsMinute     = gps.time.minute();
    gpsSecond     = gps.time.second();
    /*
    gpsSpeed      = gps.speed.kmph();
    gpsCourse     = gps.course.deg();
    gpsHDOP       = gps.hdop.hdop();
    */

  
    snprintf(gps_value_buffer[0], sizeof(gps_value_buffer[0]), "%.6f", gpsLongitude); // Long
    snprintf(gps_value_buffer[1], sizeof(gps_value_buffer[1]), "%.6f", gpsLatitude);  // Lat
    snprintf(gps_value_buffer[2], sizeof(gps_value_buffer[2]), "%.2f", gpsAltitude);  // Alt
    snprintf(gps_value_buffer[3], sizeof(gps_value_buffer[3]), "%u",   gpsSatellites); // Sat#
    snprintf(gps_value_buffer[7], sizeof(gps_value_buffer[7]), "%02u:%02u:%02u", gpsHour, gpsMinute, gpsSecond); // Time
    /*
    snprintf(gps_value_buffer[4], sizeof(gps_value_buffer[4]), "%.2f", gpsSpeed);      // Spd
    snprintf(gps_value_buffer[5], sizeof(gps_value_buffer[5]), "%.2f", gpsCourse);     // Crs
    snprintf(gps_value_buffer[6], sizeof(gps_value_buffer[6]), "%.2f", gpsHDOP);       // HDOP
    */

  } else {
  if (currentT - GPS_T >= 2000){
    Serial.println("No GPS fix yet");
    GPS_T = currentT;
    }
  }

  static unsigned long SD_TIME = 0;
  if (!SD_CARD_READY && currentT - SD_TIME >= 5000) {
    Serial.println("Trying to init SD...");
    if (SD.begin(SD_CS, hspi, 10000000)) {   // lower clock for stability
      SD_CARD_READY = true;
      Serial.println("SD Ready!");
    } else {
      Serial.println("SD init failed, retrying...");
    }
    SD_TIME = currentT;
  }

// battery
  static unsigned long Vbat_Time = 0;
  if(currentT - Vbat_Time >= 10000){
    voltage_bat = readBatteryVoltage();
    Vbat_Time = currentT;
  }
}


