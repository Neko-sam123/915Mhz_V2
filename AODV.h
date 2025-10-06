#ifndef AODV_H
#define AODV_H

#include <Arduino.h>

//#include <LoRa.h>
#include <RadioLib.h>
extern SX1262 radio;   

#include <Preferences.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "AODV_Var.h"
#include "global.h"


class Node {

private:
  AODV_VAR aodv;
  static Preferences prefs;
  // ================= NODE CONFIG =================

  //unsigned long rreqT[MAX_SEEN_RREQS] = {0};  // for tracking RREQ timestamps
  //unsigned long rrerT[MAX_SEEN_RRERS] = {0};  

  uint32_t lastUpdateT = 0; // for route update
  #define EMPTY_HOP 0xFFFFFFFF // for end of route
  #define BROADCAST_ADDR 0x00

  
public:

  // ================= FUNCTION DECLARATIONS =================

  bool message_sent = false; // variable para malaman kung na send ba yung message 
  bool route_established = false; // variable para kung may route na

  Node () {}
  void init();
  void receivePacket();                                                                                          // taga check nung receive packet, may switch d2


  void startTransmit(uint8_t* data, size_t len);
  void updateRadio();


  void sendRREQ(uint32_t destAddr);                                                                              // d2 ko ilalagay yung selection nung node para sa destination
  int findseqNo(uint32_t destAddr);
  void sendDataMessage(const char* text, uint32_t destAddr);                                                     // d2 nag sesend ng message, dipa ayos variables


  void saveMessageToSD(const char* msg, uint32_t destAddr);
  
  // ================ RREP & RREQ sender =====================
  void sendRRER(uint32_t unreachableDest, uint16_t msgID);


  void sendGPSRequest(uint32_t destAddr ,int req_rep);
  int findGPS(uint32_t nodeAddr);

  // ========= routing table helpers ==========================
  int findRoute(uint32_t destAddr, bool requireUsable);                                                                              // taga check kung may route na, nag rereturn ng -1 pag wala


  void updateRoutingTable(uint32_t sourceAddr, uint32_t destAddr, uint32_t lastHop,
                              uint8_t hopCount, uint32_t seqNo, uint32_t RSSI,
                              bool isReverse, uint32_t nextHop = 0);

  void updateGPSTable(uint32_t nodeAddr, float lat, float lon);

  void invalidateRoute(uint32_t destAddr);
  void update_Expiry_time();


  void transmitEmergency();
  void transmitEmergencyACK(uint32_t origNodeAddr, uint32_t broadcastID, uint32_t myNodeAddr);

  bool isDuplicate(SeenPacket* table, int tableSize, uint32_t srcOrDest, uint32_t id);
  
  void handleEmergencyACK(uint32_t ackNodeID, float lat, float lon, uint32_t timestamp);               // tiga check nung mga mag rerespond sa emergency 
  

  void saveRoutingTable();                                                                                        // nag store saka print ng route sa SD card
  void loadRoutingTableFromSD();                                                                                  // nasa init nag rerefresh nung route pag nireset

  void saveNodeStateToNVS();  
  void LoadNodeState();

  void printRoutingTable();

  void deleteAllLogs();
  
  /*
  void logRRERToSD(uint32_t dest, uint32_t origin);                                                              // Log ng error sa SD card*/

};

#endif