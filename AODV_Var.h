#ifndef AODV_Var_H
#define AODV_Var_H

#include <Arduino.h>
#include "global.h"

#define MAX_ROUTES 20  // You can increase if needed
#define MAX_SEEN_RREQS 10
#define MAX_SEEN_RREP 10
#define DATA_NODE 10
#define MAX_SEEN_RRERS 10
#define ACK_NODE 10
#define MAX_SEEN_GPS 10

#define Seen_Emergency 10
#define MAX_RESPONDERS 5

#define MAX_PAYLOAD_LEN 32
#define ROUTE_TIMEOUT_MS 300000  // 5 minutes
#define GPS_ENTRY_TIMEOUT 60000  // 1 minute

struct RoutingEntry {
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

// ===== Struct for Node GPS Info =====
struct NodeGPS {
  uint32_t nodeAddr;      // unique node address
  float latitude;         // latest latitude
  float longitude;        // latest longitude
  unsigned long lastSeen;
  bool valid;
};


struct SeenPacket {
  uint32_t srcOrDest;    // source or destination address
  uint32_t id;           // msgID, broadcastID, seqNo, etc.
  unsigned long lastSeen;
};




// ===== Struct for the Table of Responders of the Emergency Broadcast
struct EmergencyResponder {
  uint32_t nodeID;        // the node that replied
  float latitude;         // responder location (from ACK)
  float longitude;
  unsigned long lastSeen; // when the ACK was received
};

// ================= AODV MESSAGE TYPES =================
enum MessageType {
  TYPE_RREQ = 1,
  TYPE_RREP = 2,
  TYPE_DATA = 3,
  TYPE_ACK  = 4,
  TYPE_RRER = 5,
  TYPE_Emergency_Packet = 6,
  TYPE_ACK_Emergency_Packet = 7,
  TYPE_GPS_REQ = 8
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
  // Total 27 bytes
};

struct __attribute__((packed)) RREP {
  uint8_t type = TYPE_RREP;
  uint8_t hopCount = 0;
  uint8_t ttl = 8;  // New field

  uint32_t sourceAddr;
  uint32_t sourceSeqNo;

  uint32_t destAddr;
  uint32_t destSeqNo;

  uint32_t lastHop;
  uint32_t nextHop;

  uint32_t broadcastID;  // new field for duplicate detection

  // Total 31 bytes
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
  uint8_t type = TYPE_ACK;           // 1 byte
  uint8_t ttl = 8;            // 1 byte
  uint32_t sourceAddr;    // 4 bytes
  uint32_t destAddr;      // 4 bytes
  uint16_t msgID;         // 2 bytes
  uint32_t nextHop;

  // Total 16 bytes
};

struct __attribute__((packed)) RRER {
  uint8_t type = TYPE_RRER;        // 1 byte (e.g., 5 for RRER)
  uint8_t ttl = 8;            // 1 byte

  uint32_t destAddr;   // 4 bytes (unreachable destination address)
  uint16_t msgID;      // 2 bytes (message ID - consistent with other structs)

  // Total 8 bytes
};


struct __attribute__((packed)) EmergencyPacket {
  uint8_t type = TYPE_Emergency_Packet;      
  uint8_t ttl = 8;

  uint32_t Emergency_node_Address;     // unique ID of the T-Beam node
  uint32_t broadcastID;

  float latitude;      // GPS latitude
  float longitude;     // GPS longitude

  uint32_t timestamp;  // optional, millis() or epoch

  // Total 22 bytes
};


struct __attribute__((packed)) Emergency_MessageACK {
  uint8_t type = TYPE_ACK_Emergency_Packet;        // 2 = message ACK
  uint8_t ttl = 8;

  uint32_t Emergency_node_Address; 
  uint32_t Emergency_BroadcastID;      // unique ID of the message
  uint32_t Emergency_Sender_Ack_Address;   // node sending this ACK
  
  float latitude;        // GPS of ACK sender
  float longitude;

  uint32_t timestamp;                // optional, match the original message timestamp

  // Total 26 bytes
};

struct __attribute__((packed)) GPSMessage {
  uint8_t type = TYPE_GPS_REQ;     // Message type identifier
  uint8_t ttl = 8;             // Time-to-live
  uint32_t destAddr;           // Final destination
  uint32_t sourceAddr;         // Original sender
  uint8_t REQ_REP;                // 0 = REQ, 1 = REP
  uint16_t msgID;              // Unique message ID
  uint32_t nextHop;            // Next hop in the route

  // GPS data
  float latitude;              // 4 bytes
  float longitude;             // 4 bytes

  // Total 25 bytes
  
};

extern RoutingEntry routingTable[MAX_ROUTES];
extern NodeGPS gpsTable[MAX_ROUTES]; 

extern SeenPacket seenRREQs[MAX_SEEN_RREQS];
extern SeenPacket seenRREPs[MAX_SEEN_RREP];
extern SeenPacket seenRRERs[MAX_SEEN_RRERS];
extern SeenPacket dataTable[DATA_NODE];  // reuse too
extern SeenPacket ackTable[ACK_NODE];
 

extern SeenPacket seenEmergency[Seen_Emergency];      // for broadcast
extern SeenPacket seenEmergencyAck[Seen_Emergency]; // for ACKs

extern SeenPacket seenGPS[MAX_SEEN_GPS];

extern EmergencyResponder responderTable[MAX_RESPONDERS];

const int Address_size = 4;

extern uint32_t Address[Address_size];
extern char lastPayload[Address_size][32];
extern bool hasNewPayload[Address_size];


class AODV_VAR{

public:
  char myNodeName[10];
  
  void Init();

  void incrementNode(uint32_t addr);
  int findAddressIndex(uint32_t addr);
  void storePayload(const char* payload, uint32_t sourceAddr);
  bool hasPayload(uint32_t sourceAddr);
  void clearLastPayload(uint32_t sourceAddr);
  const char* getLastPayload(uint32_t sourceAddr);
};






#endif