#ifndef AODV_Var_H
#define AODV_Var_H

#include <Arduino.h>
#include "global.h"

#define MAX_ROUTES 10  // You can increase if needed
#define MAX_SEEN_RREQS 10
#define MAX_SEEN_RRERS 10
#define ACK_NODE 5
#define MAX_PAYLOAD_LEN 32
#define ROUTE_TIMEOUT_MS 300000  // 5 minutes

struct RoutingEntry {
  uint32_t sourceAddr = 0;
  uint32_t destAddr = 0;
  uint32_t nextHop = 0;
  uint32_t lastHop = 0;
  uint8_t hopCount = 0;
  uint32_t sourceSeqNo = 0;
  uint32_t destSeqNo = 0;
  uint32_t RSSI = 0;
  bool valid = false;
  bool usable = false;
  unsigned long expiryTime = 0;

  // ✅ Default constructor
  RoutingEntry() {}

  // ✅ Overloaded constructor
  RoutingEntry(uint32_t dAddr, uint32_t lHop, uint8_t hop, uint32_t sSeq,
             uint32_t rssi, bool isUsable = true, uint32_t nHop = 0) {
    destAddr = dAddr;
    lastHop = lHop;
    nextHop = nHop;             // Optional; can stay 0 if unknown
    hopCount = hop;
    sourceSeqNo = sSeq;
    RSSI = rssi;
    valid = true;
    usable = isUsable;
    expiryTime = 0;
  }
};

struct SeenRREQ {
  uint32_t sourceAddr;
  uint32_t broadcastID;
};

// Define the struct for seen RRERs
struct SeenRRER {
  uint32_t destAddr;     // unreachable destination
  uint16_t msgID;        // message ID (or seqNo if you prefer)
  unsigned long timestamp; // time when this RRER was seen
};

struct AckTracker {
  uint32_t sourceAddr;
  uint16_t lastAckID;
};

// ================= AODV MESSAGE TYPES =================
enum MessageType {
  TYPE_RREQ = 1,
  TYPE_RREP = 2,
  TYPE_DATA = 3,
  TYPE_ACK  = 4,
  TYPE_RRER = 5
};

// ================= AODV STRUCTS =================
struct __attribute__((packed)) RREQ {
  uint8_t type = TYPE_RREQ;
  uint8_t hopCount = 0;
  uint8_t ttl = 8;  // Time to live

  uint32_t broadcastID;

  uint32_t sourceAddr;
  uint32_t sourceSeqNo;

  uint32_t destAddr;
  uint32_t destSeqNo;

  uint32_t lastHopAddr;
  // Total 30 bytes
};

struct __attribute__((packed)) RREP {
  uint8_t type = TYPE_RREP;
  uint8_t hopCount = 0;
  uint8_t ttl = 8;  // New field

  uint32_t sourceAddr;
  uint32_t sourceSeqNo;

  uint32_t destAddr;
  uint32_t destSeqNo;

  uint32_t nextHop;
  uint32_t lastHop;

  // Total 24 bytes
};


struct __attribute__((packed)) DataMessage {
  uint8_t type = TYPE_DATA;
  uint8_t ttl = 8;  // New field
  uint32_t destAddr;
  uint32_t sourceAddr;
  uint16_t msgID;
  uint32_t nextHop;
  char payload[MAX_PAYLOAD_LEN]; //32 bytes
  // Total 48 bytes
};


struct __attribute__((packed)) ACKMessage {
  uint8_t type;           // 1 byte
  uint8_t ttl;            // 1 byte
  uint32_t sourceAddr;    // 4 bytes
  uint32_t destAddr;      // 4 bytes
  uint16_t msgID;         // 2 bytes
  uint32_t nextHop;
};

struct __attribute__((packed)) RRER {
  uint8_t type = TYPE_RRER;        // 1 byte (e.g., 3 for RERR)
  uint8_t ttl = 8;            // 1 byte

  uint32_t destAddr;   // 4 bytes (unreachable destination address)
  uint32_t msgID;  // 4 bytes (last known sequence number)
};


extern RoutingEntry routingTable[MAX_ROUTES];
extern SeenRREQ seenRREQs[MAX_SEEN_RREQS];
extern SeenRRER seenRRERs[MAX_SEEN_RRERS];
extern AckTracker ackTable[ACK_NODE];  // for 5 nodes

const int Address_size = 4;

extern uint32_t Address[Address_size];
extern char lastPayload[Address_size][32];
extern bool hasNewPayload[Address_size];


class AODV_VAR{

public:

  int findAddressIndex(uint32_t addr);
  void storePayload(const char* payload, uint32_t sourceAddr);
  bool hasPayload(uint32_t sourceAddr);
  void clearLastPayload(uint32_t sourceAddr);
  const char* getLastPayload(uint32_t sourceAddr);
};






#endif