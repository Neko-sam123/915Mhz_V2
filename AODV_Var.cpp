#include "AODV_Var.h"

RoutingEntry routingTable[MAX_ROUTES];
SeenRREQ seenRREQs[MAX_SEEN_RREQS];
SeenRRER seenRRERs[MAX_SEEN_RRERS];
AckTracker ackTable[ACK_NODE];

uint32_t Address[Address_size] = {
    0xA1,   //Main node
    0xB2,  // First node
    0xC3,  // Second node
    //0xD4,  // Third node
    0xE5  // Fourth node
};

// ================= NODE CONFIG =================x
//uint32_t myAddress = 0xA1;  // Main node
//uint32_t myAddress = 0xB2;  //  First node
//uint32_t myAddress = 0xC3;  // Seconde node
uint32_t myAddress = 0xD4;  // Third node
//uint32_t myAddress = 0xE5;  // Fourth node

char item_buffer[5][10] = 
    {
    "Main Node",
    "Node 1",
    "Node 2",
    //"Node 3",
    "Node 4", 
    }; // list ng mga node


char lastPayload[Address_size][32];
bool hasNewPayload[Address_size] = { false };






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
}

// Get payload for given address
const char* AODV_VAR::getLastPayload(uint32_t addr) {
    int i = findAddressIndex(addr);
    if (i == -1) return "";
    return lastPayload[i];
}
