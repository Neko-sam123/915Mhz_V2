#include "AODV_Var.h"

RoutingEntry routingTable[MAX_ROUTES];
NodeGPS gpsTable[MAX_ROUTES]; 

SeenPacket seenRREQs[MAX_SEEN_RREQS];
SeenPacket seenRREPs[MAX_SEEN_RREP];
SeenPacket seenRRERs[MAX_SEEN_RRERS];
SeenPacket dataTable[DATA_NODE];  // reuse too
SeenPacket ackTable[ACK_NODE];

SeenPacket seenEmergency[Seen_Emergency];      // for broadcast
SeenPacket seenEmergencyAck[Seen_Emergency]; // for ACKs

SeenPacket seenGPS[MAX_SEEN_GPS];

EmergencyResponder responderTable[MAX_RESPONDERS];

// Global node state (shared by all Node instances)
uint32_t mySeqNo = 1;
uint32_t lastBroadcastID = 0;
uint16_t lastMsgID = 0;

uint32_t prevSeqNo = 1;
uint32_t prevBroadcastID = 0;
uint32_t prevMsgID = 0;

uint16_t lastAckID = 0;
uint32_t lastAckSource = 0;

char lastPayload[Address_size][32];
bool hasNewPayload[Address_size] = { false };

// ================= NODE CONFIG =================x

int choose_ID = 1;

// All node addresses
uint32_t allAddresses[5] = {
    0xA1,   // Main node
    0xB2,   // Node 1
    0xC3,   // Node 2
    0xD4,   // Node 3
    0xE5    // Node 4
};

// All node names
char allNames[5][10] = {
    "Main Node",
    "Node 1",
    "Node 2",
    "Node 3",
    "Node 4"
};


uint32_t Address[4];
char item_buffer[4][10];
uint32_t myAddress = allAddresses[choose_ID];


void AODV_VAR::Init(){
    int nodeCount = 0;

    strcpy(myNodeName, allNames[choose_ID]);

    for (int i = 0; i < 5; i++) {
    if (i != choose_ID) { // skip my node
      Address[nodeCount] = allAddresses[i];
      strcpy(item_buffer[nodeCount], allNames[i]);
      nodeCount++;
    }
  }
}


int messageCount[5] = {0};

void AODV_VAR::incrementNode(uint32_t addr) {
    size_t numNodes = sizeof(Address) / sizeof(Address[0]);

    for (size_t i = 0; i < numNodes; i++) {
        if (Address[i] == addr) {
            messageCount[i]++;
            return; // stop once we found the node
        }
    }
}

// Find index of an address in Address[]
int AODV_VAR::findAddressIndex(uint32_t addr) {
    for (int i = 0; i < Address_size; i++) {
        if (Address[i] == addr)
            return i;
    }
    return -1;  // Not found
}

// Store payload for given address
void AODV_VAR::storePayload(const char* payload, uint32_t addr) {
    int i = findAddressIndex(addr);
    if (i == -1) return;  // Invalid address

    strncpy(lastPayload[i], payload, sizeof(lastPayload[i]) - 1);
    lastPayload[i][sizeof(lastPayload[i]) - 1] = '\0';
    hasNewPayload[i] = true;
    incrementNode(addr);
}

// Check if new payload exists for given address
bool AODV_VAR::hasPayload(uint32_t addr) {
    int i = findAddressIndex(addr);
    if (i == -1) return false;
    return hasNewPayload[i];
}

// Clear payload for given address
void AODV_VAR::clearLastPayload(uint32_t addr) {
    int i = findAddressIndex(addr);
    if (i == -1) return;
    lastPayload[i][0] = '\0';
    hasNewPayload[i] = false;
    messageCount[i] = 0;
}

// Get payload for given address
const char* AODV_VAR::getLastPayload(uint32_t addr) {
    int i = findAddressIndex(addr);
    if (i == -1) return "";
    return lastPayload[i];
}
