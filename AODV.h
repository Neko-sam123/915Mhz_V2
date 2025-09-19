#ifndef AODV_H
#define AODV_H

#include <Arduino.h>

//#include <LoRa.h>
#include <RadioLib.h>
extern SX1262 radio;   

#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "AODV_var.h"
#include "global.h"

class Node {

private:
  AODV_VAR aodv;
  // ================= NODE CONFIG =================

  unsigned long rreqT[MAX_SEEN_RREQS] = {0};  // for tracking RREQ timestamps
  //unsigned long rrerT[MAX_SEEN_RRERS] = {0};  

  uint32_t lastUpdateT = 0; // for route update
  #define EMPTY_HOP 0xFFFFFFFF // for end of route
  #define BROADCAST_ADDR 0x00
  
public:

  uint32_t mySeqNo;
  uint32_t lastBroadcastID;
  uint16_t lastMsgID;

  uint16_t lastAckID = 0;
  uint32_t lastAckSource = 0;

  char lastPayload[32];
  bool hasNewPayload = false;
  // ================= FUNCTION DECLARATIONS =================

  bool message_sent = false; // variable para malaman kung na send ba yung message 
  bool route_established = false; // variable para kung may route na

  Node () {}
  void init();
  void receivePacket();                                                                                          // taga check nung receive packet, may switch d2

  void storePayload(const char* payload);
  bool hasPayload();
  void clearLastPayload();
  const char* getLastPayload();


  void sendRREQ(uint32_t destAddr);                                                                              // d2 ko ilalagay yung selection nung node para sa destination
  int findseqNo(uint32_t destAddr);
  void sendDataMessage(const char* text, uint32_t destAddr);                                                     // d2 nag sesend ng message, dipa ayos variables


  void saveMessageToSD(const char* msg, uint32_t destAddr);
  
  // ================ RREP & RREQ sender =====================
  void sendRREP(uint32_t sourceAddr, uint32_t sourceSeqNo, uint32_t destAddr, uint32_t destSeqNo,uint8_t hopCount, uint32_t nextHop);                   // auto send na to

  void sendRRER(uint32_t unreachableDest, uint16_t msgID);
  // ========= routing table helpers ==========================
  int findRoute(uint32_t destAddr, bool requireUsable);                                                                              // taga check kung may route na, nag rereturn ng -1 pag wala
  void updateRoutingTable(uint32_t sourceAddr, uint32_t destAddr, uint32_t lastHop,
                              uint8_t hopCount, uint32_t seqNo, uint32_t RSSI,
                              bool isReverse, uint32_t nextHop = 0);

  void update_Expiry_time();

  bool isDuplicateRREQ(uint32_t src, uint32_t bID);                                                              // kung may RREQ na na kaparehas
  bool isDuplicateRRER(uint32_t destAddr, uint16_t msgID);
  bool isDuplicateACK(uint16_t msgID, uint32_t sourceAddr);

  void saveRoutingTable();                                                                                      // nag store saka print ng route sa SD card
  void loadRoutingTableFromSD();                                                                                 // nasa init nag rerefresh nung route pag nireset

  void saveNodeStateToSD();                                                                                       // d2 nalolog yung seq no., last broadcast id, saka lastmssg id
  void loadNodeStateFromSD();

  void printRoutingTable();

  void deleteAllLogs();
  
  /*
  void logRRERToSD(uint32_t dest, uint32_t origin);                                                              // Log ng error sa SD card*/

};

#endif