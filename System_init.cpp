#include "System_init.h"

SPIClass hspi(HSPI);
bool SD_CARD_READY = false;

SPIClass spiLoRa(VSPI);

TinyGPSPlus gps;
HardwareSerial gpsSerial(2); 

XPowersAXP2101 PMU;

//////////////////////////////////////////////////////
////////////// GPS Variable SETUP ////////////////////
//////////////////////////////////////////////////////
// Global scope variable to ah
float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsAltitude = 0.0;
uint8_t gpsSatellites = 0;
double gpsSpeed      = 2.0; // km/h
double gpsCourse     = 0.0; // degrees
double gpsHDOP       = 0.0;
uint8_t gpsHour      = 12;
uint8_t gpsMinute    = 30;
uint8_t gpsSecond    = 15;
char gps_value_buffer[8][15];


bool GPS_Validity = false;
bool gpsHasFix = false;   // flag for first lock
unsigned long lastGPSCheck = 0;


//////////////////////////////////////////////////////
////////////////// RADIOLIB SETUP ////////////////////
//////////////////////////////////////////////////////

#define CS_PIN   18
#define DIO1_PIN 33   // V1.1
#define RST_PIN  23
#define BUSY_PIN 32

SX1262 radio = new Module(CS_PIN, DIO1_PIN, RST_PIN, BUSY_PIN, spiLoRa);

volatile bool rxDoneFlag  = false;

volatile bool txDoneFlag = false;
volatile bool txActive = false;

void IRAM_ATTR onRadioEvent() {
    uint16_t irq = radio.getIrqFlags();  // safer, reads register directly

    if (irq & RADIOLIB_SX126X_IRQ_TX_DONE) {
        txDoneFlag = true;
    }
    if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {
        rxDoneFlag = true;
    }
    radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);  // clear IRQ ASAP
}


const float myFreq  = 915.0;   // MHz
float bandwidth     = 125.0;   // kHz
uint8_t sf          = 11;      // spreading factor
uint8_t cr          = 7;       // coding rate
uint8_t syncWord    = 0x12;    // sync word
int8_t power        = 22;      // output power dBm
uint16_t preamble   = 8;       // preamble length

//////////////////////////////////////////////////////
/////////////// Battery Variabl Setup ////////////////
//////////////////////////////////////////////////////

float voltage_bat = 0;
int bat_percent = 0;
bool isCharging = false;
bool isLowBattery = false;

bool usbPresent = false;
bool lowBlinkState = false;

//////////////////////////////////////////////////////
//////////// PIN for Emergency Broadcast /////////////
//////////////////////////////////////////////////////
const int buttonPin = 39; 
const unsigned long longPressTime = 5000;
unsigned long buttonPressStart = 0;
bool buttonWasPressed = false;


// ======================================================= //

void System::init() {

  Serial.begin(115200);

  ////////////////////////////////////////////////////
  /////////// GPS initialization /////////////////////
  ////////////////////////////////////////////////////
  gpsSerial.begin(9600, SERIAL_8N1, 34, 12); 

  Wire.begin(21, 22);

  //////////////////////////////////////////////////
  //////////// OLED Initialization /////////////////
  //////////////////////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found!");
    while (true);
  }
  display.clearDisplay();
  display.setFont(&TomThumb);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();

  //////////////////////////////////////////////////
  //////////// LORA Initialization /////////////////
  //////////////////////////////////////////////////
  spiLoRa.begin(5, 19, 27, CS_PIN);
  int st = radio.begin(myFreq, bandwidth, sf, cr, syncWord, power, preamble);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("radio.begin failed: "); Serial.println(st);
    while(true) delay(500);
  }
  radio.setCRC(true); 
  Serial.println("TX radio init OK");
  Serial.print("Using DIO1 pin: ");
  Serial.println(DIO1_PIN);
  radio.setDio1Action(onRadioEvent);
  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  radio.startReceive();

  //////////////////////////////////////////////////
  ////////////// SD Initialization /////////////////
  //////////////////////////////////////////////////
  hspi.begin(14, 25, 13, SD_CS);
  if (!SD.begin(SD_CS, hspi)) {
    Serial.println("SD Card fail");
    SD_CARD_READY = false;
  } else {
    SD_CARD_READY = true;
    Serial.println("SD Ready");
  }

  ////////////////////////////////////////////////////
  ////////////// Battery Setup  //////////////////////
  ////////////////////////////////////////////////////

  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, 21, 22)) {
    Serial.println("AXP2101 not detected!");
  }
  Serial.println("AXP2101 detected OK!");

  PMU.enableBattVoltageMeasure();
  PMU.enableVbusVoltageMeasure();
  PMU.enableSystemVoltageMeasure();

  voltage_bat = PMU.getBattVoltage() / 1000.0;  // Convert mV → V
  ////////////////////////////////////////////////////
  /// Loading the BroadcastID, SeqNumber, MessageID // 
  ////////////////////////////////////////////////////
  message.init();
  
  ////////////////////////////////////////////////////
  ///////////// Emergency  Broadcast Setup ///////////
  ////////////////////////////////////////////////////
  //pinMode(buttonPin, INPUT_PULLUP);

}


void System::sys_loop(){

  //// AODV RECEIVE AND UPDATE FUNCTION

  node.updateRadio();
  node.receivePacket(); 
  node.update_Expiry_time();

  ///// GPS FUNCTIONS 
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
   }
   

  // Update GPS variables if location is valid
  if (gps.location.isValid()) {
    GPS_Validity = true;
    gpsLatitude   = gps.location.lat();
    gpsLongitude  = gps.location.lng();
    gpsAltitude   = gps.altitude.meters();
    gpsSatellites = gps.satellites.value();
    gpsSpeed      = gps.speed.kmph();
    gpsCourse     = gps.course.deg();

    gpsHour       = gps.time.hour();
    gpsMinute     = gps.time.minute();
    gpsSecond     = gps.time.second();
    gpsHour = (gpsHour + 24 - 16) % 24;

    snprintf(gps_value_buffer[0], sizeof(gps_value_buffer[0]), "%.6f", gpsLongitude); // Long
    snprintf(gps_value_buffer[1], sizeof(gps_value_buffer[1]), "%.6f", gpsLatitude);  // Lat
    snprintf(gps_value_buffer[2], sizeof(gps_value_buffer[2]), "%.2f", gpsAltitude);  // Alt
    snprintf(gps_value_buffer[3], sizeof(gps_value_buffer[3]), "%u",   gpsSatellites); // Sat#
    snprintf(gps_value_buffer[7], sizeof(gps_value_buffer[7]), "%02u:%02u:%02u", gpsHour, gpsMinute, gpsSecond); // Time
  } else {
    GPS_Validity = false;
    static unsigned long GPS_T = 0;
    if (millis() - GPS_T >= 5000) {
      Serial.println("No GPS fix yet...");
      GPS_T = millis();
    }
  }
  

  //////////// CHECK FOR SD CARD INTIALIZATION ////////////////
  static unsigned long SD_TIME = 0;
  if (!SD_CARD_READY && currentT - SD_TIME >= 5000) {
    Serial.println("Trying to init SD...");
    if (SD.begin(SD_CS, hspi, 10000000)) {   // lower clock for stability
      SD_CARD_READY = true;
      Serial.println("SD Ready!");
    } else {
      Serial.println("SD init failed, retrying...");
      SD_CARD_READY = false;
    }
    SD_TIME = currentT;
  }


  //////// Battery Reading function - di nagana bulok
  static unsigned long Vbat_Time = 0;
  static unsigned long Blink_Time = 0;

  if (currentT - Vbat_Time >= 1000) {
  Vbat_Time = currentT;

  long vbat_mV = PMU.getBattVoltage();
  long vbus_mV = PMU.getVbusVoltage();

  // Convert to volts
  voltage_bat = vbat_mV / 1000.0f;
  float vbusV = vbus_mV / 1000.0f;

  // Compute battery percentage
  float percent = (voltage_bat - 3.3f) / (4.2f - 3.3f);
  if (percent < 0.0f) percent = 0.0f;
  if (percent > 1.0f) percent = 1.0f;
  bat_percent = (int)(percent * 100.0f + 0.5f);

  // ⚠️ Update globals (not new local vars!)
  usbPresent = (vbusV > 4.0f);

  // Charging detection
  if (PMU.isCharging()) {
    isCharging = true;
  } else {
    isCharging = usbPresent && (bat_percent < 100);
  }

  isLowBattery = (voltage_bat <= 3.4f);
  }

  // Blink
  if (currentT - Blink_Time >= 500) {
  Blink_Time = currentT;
  lowBlinkState = !lowBlinkState;
  }

  ////////////////////
  //Emergency_Broadcast();
}


void System::sendUBX(uint8_t *msg, uint8_t len) {
  Serial2.write(0xB5); // UBX header
  Serial2.write(0x62); // UBX header
  Serial2.write(msg, len);
}

void System::checkGPSFix() {
    // Parse GPS serial (already done in sys_loop, no need to read again)
    
    // Check fix every second
    if (millis() - lastGPSCheck > 1000) {
        lastGPSCheck = millis();

        if (!gpsHasFix && gps.location.isValid()) {
            Serial.println("✅ GPS fix acquired!");
            gpsHasFix = true;

            // ONLY enable power save after first fix
            enableGPS_PSM();
        }

        if (gpsHasFix && !gps.location.isValid()) {
            Serial.println("⚠️ GPS fix lost, switching back to full power...");
            gpsHasFix = false;
            enableGPS_FullPower();
        }
    }
}

void System::enableGPS_PSM() {
  // UBX-CFG-RXM (set Power Save Mode)
  uint8_t setRXM[] = {
    0x06, 0x11, 0x02, 0x00,   // CFG-RXM, length=2
    0x08, 0x01                // LP mode: Power Save Mode
  };
  sendUBX(setRXM, sizeof(setRXM));

  // UBX-CFG-PM2 (configure cyclic tracking)
  uint8_t setPM2[] = {
    0x06, 0x3B, 0x2C, 0x00,   // CFG-PM2, length=44
    0x01, 0x00, 0x00, 0x00,   // version / reserved
    0x00, 0x00, 0x00, 0x00,   // reserved
    0x10, 0x27, 0x00, 0x00,   // update period = 10000 ms (10s)
    0x10, 0x27, 0x00, 0x00,   // search period = 10000 ms (10s)
    0x00, 0x00, 0x00, 0x00,   // grid offset
    0x00, 0x00, 0x00, 0x00,   // on time
    0x00, 0x00, 0x00, 0x00,   // min awake time
    0x00, 0x00, 0x00, 0x00,   // reserved
    0x00, 0x00, 0x00, 0x00,   // reserved
    0x00, 0x00, 0x00, 0x00,   // reserved
    0x00, 0x00, 0x00, 0x00    // reserved
  };
  sendUBX(setPM2, sizeof(setPM2));

  Serial.println("✅ GPS set to Power Save Mode (cyclic tracking)");
}

// Switch GPS back to continuous/full power mode
void System::enableGPS_FullPower() {
  // UBX-CFG-RXM (set Continuous Mode)
  uint8_t setRXM[] = {
    0x06, 0x11, 0x02, 0x00,   // CFG-RXM, length=2
    0x08, 0x00                // LP mode: Continuous
  };
  sendUBX(setRXM, sizeof(setRXM));

  Serial.println("✅ GPS set to FULL POWER (continuous tracking)");
}


void System::Emergency_Broadcast(){
  int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) { // button pressed
    if (!buttonWasPressed) {
      // start timing
      buttonPressStart = millis();
      buttonWasPressed = true;
    } else {
      // check duration

      if (millis() - buttonPressStart >= longPressTime) {
        node.transmitEmergency();
        interface_UI_disabler = true;
        interface_navigation = 3;
      }
    }
  } else {
    // button released
    buttonWasPressed = false;

  }
}


