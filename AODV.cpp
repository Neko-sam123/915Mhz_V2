#include "AODV.h"



// ================= SETUP =================
void Node::init() {
  //loadRoutingTableFromSD();
  loadNodeStateFromSD();
}


// ================= RECEIVE PACKETS =================
void Node::receivePacket() {

  if (packetFlag) {
  packetFlag = false;                 // clear flag early
  //Serial.println("✅ IRQ fired: packet available!");

  uint8_t buffer[64];

  //memset(buffer, 0, sizeof(buffer));

  int len = radio.getPacketLength();
  if(len > 0 && len <= sizeof(buffer)) {
    int state = radio.readData(buffer, len);
  }

  if (len <= 0) {
    Serial.print("readData failed: ");
    Serial.println(len);
    radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
    return;
  }
  
  uint8_t type = buffer[0];

  switch (type) {
    case TYPE_RREQ: {
      if (len < sizeof(RREQ)) {
        Serial.println("Invalid RREQ packet.");
        return;
      }

      RREQ rreq;
      memcpy(&rreq, buffer, sizeof(RREQ));

      if (rreq.ttl == 0) {
        Serial.println("RREQ dropped: TTL expired");
        return;
      }

      if (rreq.sourceAddr == myAddress){
        Serial.println("RREQ dropped: This node is the source");
        return;
      }

      if (isDuplicateRREQ(rreq.sourceAddr, rreq.broadcastID)) {
        Serial.println("Duplicate RREQ ignored.");
        return;
      }
      
      rreq.ttl--;
      rreq.hopCount++;

      // NextHop is unknown in RREQ; just leave it as default
      updateRoutingTable(
        rreq.sourceAddr, 
        rreq.destAddr,
        rreq.lastHopAddr, 
        rreq.hopCount,
        rreq.destSeqNo,
        radio.getRSSI(), 
        false); // forward


      updateRoutingTable(
        rreq.sourceAddr, 
        rreq.destAddr, 
        0, 
        0, 
        rreq.sourceSeqNo, 
        radio.getRSSI(), 
        true, // reverse 
        rreq.lastHopAddr);

      if (rreq.destAddr == myAddress) {
    
        mySeqNo++;
        RREP rrep;

        rrep.type = TYPE_RREP;
        rrep.hopCount = rreq.hopCount;
        rrep.ttl = 8;  // Default TTL

        rrep.sourceAddr = rreq.sourceAddr; //node nung nag send ng RREQ
        rrep.sourceSeqNo = rreq.sourceSeqNo;

        rrep.destAddr = myAddress; // node yan ng destination ng RREQ/ yung address mismo nito
        rrep.destSeqNo = mySeqNo; // seq number ng destination node

        rrep.nextHop = rreq.lastHopAddr;
        rrep.lastHop = myAddress;

        updateRoutingTable(
          rreq.sourceAddr, 
          rreq.destAddr, 
          rreq.lastHopAddr, 
          rreq.hopCount, 
          mySeqNo, 
          radio.getRSSI(), 
          false // reverse
          ); 

        int transmit_code = radio.transmit((uint8_t*)&rrep, sizeof(RREP));
        if (transmit_code == RADIOLIB_ERR_NONE) {
          Serial.println("RREQ RECEIVE, RREP SENT");
        } else {  
          Serial.println("TX failed code "); Serial.println(transmit_code);
        }
        radio.startReceive();
        saveNodeStateToSD();

        //Serial.println("📡 Sending RREP to source.");
      } else {
        rreq.lastHopAddr = myAddress;
        int transmit_code = radio.transmit((uint8_t*)&rreq, sizeof(RREQ));
        if (transmit_code == RADIOLIB_ERR_NONE) {
          Serial.println("RREQ FORWARD");
        } else {  
          Serial.println("TX failed code "); Serial.println(transmit_code);
        }
        radio.startReceive();
        Serial.printf("Forwarded RREQ from %lu to %lu\n", rreq.sourceAddr, rreq.destAddr);
      }

      break;
    }

    case TYPE_RREP: {
      if (buffer == nullptr || len < sizeof(RREP)) {
        Serial.println("Invalid RREP packet size.");
        return;
      } 

      RREP rrep;
      memcpy(&rrep, buffer, sizeof(RREP));

      if (rrep.ttl == 0) {
        Serial.println("RREP dropped: TTL expired");
        return;
      }

      if (rrep.nextHop != myAddress) {
        Serial.printf(" Not the next hop (%lu). Packet for %lu ignored.\n", myAddress, rrep.nextHop);
        return;
      }

      rrep.ttl--;
      rrep.hopCount++;
      

      int index = findRoute(rrep.sourceAddr,false);
      updateRoutingTable(
        routingTable[index].sourceAddr,
        routingTable[index].destAddr, 
        routingTable[index].lastHop, 
        routingTable[index].hopCount, 
        routingTable[index].sourceSeqNo,
        routingTable[index].RSSI, 
        false,  // forward
        rrep.lastHop);  

      index = findRoute(rrep.destAddr,false);
      updateRoutingTable(
        routingTable[index].sourceAddr,
        routingTable[index].destAddr, 
        rrep.lastHop, rrep.hopCount, 
        rrep.sourceSeqNo,routingTable[index].RSSI,
        true, //reverse
        routingTable[index].lastHop
         ); 

      if (rrep.sourceAddr == myAddress) {
        Serial.println("Route established.");

        updateRoutingTable(
          rrep.sourceAddr,
          rrep.destAddr,
          routingTable[index].lastHop,
          rrep.hopCount,rrep.destAddr,
          radio.getRSSI(),
          false,
          rrep.lastHop
          );
          route_established = true;
      } else {
        int index = findRoute(rrep.sourceAddr,true);
        if (index != -1) {

          int transmit_code = radio.transmit((uint8_t*)&rrep, sizeof(RREP));
          if (transmit_code == RADIOLIB_ERR_NONE) {
            Serial.println("RREP SENT TO NEXT HOP");
          } else {  
            Serial.print("TX failed code "); Serial.println(transmit_code);
          }
          radio.startReceive();
          Serial.printf("Forwarded RREP to next hop. HopCount: %u, TTL: %u\n", rrep.hopCount, rrep.ttl);
          } else {
          Serial.println(" No route to source. RREP not forwarded.");
        }
      }
      break;
      }


    case TYPE_DATA: {
      
      if (buffer == nullptr || len < sizeof(DataMessage)) {
        Serial.println(" Invalid DATA packet.");
        return;
      }

      // Copy from buffer safely
      DataMessage msg;
      memcpy(&msg, buffer, sizeof(DataMessage));

      Serial.println(" DATA received:");
      Serial.printf("  Source     : 0x%lX\n", msg.sourceAddr);
      Serial.printf("  Destination: 0x%lX\n", msg.destAddr);
      Serial.printf("  Next Hop   : 0x%lX\n", msg.nextHop);
      Serial.printf("  My Address : 0x%lX\n", myAddress);

      // TTL check
      msg.ttl--;
      if (msg.ttl == 0) {
        Serial.println("DATA dropped: TTL expired");
        return;
      }

      if (msg.destAddr == myAddress) {
        // Prepare ACK
        ACKMessage ack;
        ack.type = TYPE_ACK;
        ack.sourceAddr = myAddress;
        ack.destAddr = msg.sourceAddr;
        ack.msgID = msg.msgID;
        ack.ttl = 8;

        int routeIndex = findRoute(ack.destAddr,true);
        if (routeIndex != -1) {
            aodv.storePayload(msg.payload, msg.sourceAddr);
            String formatted = "> " + String(msg.sourceAddr, HEX) + ": " + String(msg.payload);
            saveMessageToSD(formatted.c_str(), msg.sourceAddr);
            ack.nextHop = routingTable[routeIndex].nextHop;
            int transmit_code = radio.transmit((uint8_t*)&ack, sizeof(ACKMessage));
            if (transmit_code == RADIOLIB_ERR_NONE) {
              Serial.print("ACK SENT");
            } else {
              Serial.print("TX failed code "); Serial.println(transmit_code);
            }

          radio.startReceive();
          //Serial.println(" Sent ACK back to source");

        } else {
          sendRRER(ack.destAddr,ack.msgID);
          Serial.println(" No route to source for ACK");

        }
        return;
      }


      if (msg.nextHop != myAddress) {
        Serial.printf(" DATA not for me (nextHop=%lu), ignoring.\n", msg.nextHop);
        return;
      }


      if(msg.nextHop == myAddress) {
        // Forward the data
        int routeIndex = findRoute(msg.destAddr,true);
        if (routeIndex != -1) {
          msg.nextHop = routingTable[routeIndex].nextHop;

          int transmit_code = radio.transmit((uint8_t*)&msg, sizeof(DataMessage));
          if (transmit_code == RADIOLIB_ERR_NONE) {
            Serial.print("DATA SENT TO NEXT HOP");
          } else {  
            Serial.print("TX failed code "); Serial.println(transmit_code);
          }
          radio.startReceive();

          Serial.printf(" Forwarded DATA to %lu (nextHop=%lu)\n", msg.destAddr, msg.nextHop);
        } else {
          Serial.println(" No route to forward DATA");
        }
      }

      break;
    }

    case TYPE_ACK: {
        if (len < sizeof(ACKMessage)) return;  // Validate size

        ACKMessage ack;
        memcpy(&ack, buffer, sizeof(ACKMessage));
        

        if (ack.ttl == 0) {
          Serial.println(" ACK dropped: TTL expired");
          return;
        }
        ack.ttl--;

        if (isDuplicateACK(ack.msgID, ack.sourceAddr)) {
          Serial.println("⚠️ Duplicate ACK ignored.");
          break;
        }

        if (ack.destAddr == myAddress) {
          Serial.print(" ACK received for msgID: ");
          Serial.println(ack.msgID);
          message_sent = true;

          // Mark msgID as delivered here if you store sent message IDs
        } else {
          int routeIndex = findRoute(ack.destAddr,true);
          if (routeIndex != -1) {
            ack.nextHop = routingTable[routeIndex].nextHop;

              int transmit_code = radio.transmit((uint8_t*)&ack, sizeof(ACKMessage));
              if (transmit_code == RADIOLIB_ERR_NONE) {
                Serial.print("ACK RELY TO NEXT HOP");
              } else {  
                Serial.print("TX failed code "); Serial.println(transmit_code);
              }
            radio.startReceive();

            Serial.print(" Forwarded ACK toward ");
            Serial.println(ack.destAddr);
          } else {
            sendRRER(ack.destAddr, ack.msgID);
            Serial.println(" Cannot forward ACK: No route");
          }
        }
        break;
      }
    
    case TYPE_RRER: {
      if (len < sizeof(RRER)) return;  // Validate size

      RRER rrer;
      memcpy(&rrer, buffer, sizeof(RRER));

      if (rrer.ttl == 0) {
          Serial.println(" ACK dropped: TTL expired");
          return;
        }
      rrer.ttl--;

      if (isDuplicateRRER(rrer.destAddr, rrer.msgID)) {
        Serial.println("Duplicate RRER ignored.");
        return;
      }

      if (findRoute(rrer.destAddr, true) != -1) {
        // Route exists → delete it
        for (int i = 0; i < MAX_ROUTES; i++) {
            if (routingTable[i].valid && routingTable[i].destAddr == rrer.destAddr) {
                routingTable[i].valid = false;
                routingTable[i].usable = false;
                routingTable[i].expiryTime = 0;
                Serial.println("Route deleted from routing table due to RRER");
            }
          }
      }

      int transmit_code = radio.transmit((uint8_t*)&rrer, sizeof(rrer));
      if (transmit_code == RADIOLIB_ERR_NONE) {
        Serial.print("RRER retransmit");
      } else {  
        Serial.print("TX failed code "); Serial.println(transmit_code);
      }

      radio.startReceive();
      break;

      } // closing case statement
    radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
    radio.startReceive();
    }
  } // packet 
  //saveNodeStateToSD();
}


// ================= SEND RREQ =================
void Node::sendRREQ(uint32_t destAddr) {
  mySeqNo++;
  lastBroadcastID++;

  RREQ rreq;
  rreq.type = TYPE_RREQ;
  rreq.hopCount = 0;
  rreq.ttl = 8;  // Default TTL value
  rreq.broadcastID = lastBroadcastID;
  
  rreq.sourceAddr = myAddress;
  rreq.sourceSeqNo = mySeqNo;

  rreq.destAddr = destAddr;
  rreq.destSeqNo = findseqNo(destAddr);

  rreq.lastHopAddr = myAddress;

  int transmit_code = radio.transmit((uint8_t*)&rreq, sizeof(RREQ));
  if (transmit_code == RADIOLIB_ERR_NONE) {
    Serial.print("RREQ SENT");
  } else {  
    Serial.print("TX failed code "); Serial.println(transmit_code);
  }
  radio.startReceive();
  saveNodeStateToSD();

}


int Node::findseqNo(uint32_t destAddr){
  int index = findRoute(destAddr,false);
  if (index == -1){
    return 0;
  }else{
    return routingTable[index].destSeqNo;
  }
}



// ================= SEND DATA =================
void Node::sendDataMessage(const char* text, uint32_t destAddr) {
  Serial.print("To: 0x"); Serial.print(destAddr,HEX);
  int routeIndex = findRoute(destAddr,true);
  
  if (routeIndex == -1) {
    Serial.println(" No route. Sending RREQ...");
    sendRREQ(destAddr);
    message_sent = false;
    return;
  }

  lastMsgID++;

  DataMessage msg;
  msg.type = TYPE_DATA;
  msg.ttl = 8;  // Default TTL
  msg.destAddr = destAddr;
  msg.sourceAddr = myAddress;
  msg.nextHop = routingTable[routeIndex].nextHop;
  msg.msgID = lastMsgID;
  strncpy(msg.payload, text, sizeof(msg.payload));
  msg.payload[sizeof(msg.payload) - 1] = '\0';

  int transmit_code = radio.transmit((uint8_t*)&msg, sizeof(msg));
  if (transmit_code == RADIOLIB_ERR_NONE) {
    Serial.print("RREP SENT");
  } else {  
    Serial.print("TX failed code "); Serial.println(transmit_code);
  }
  radio.startReceive();

  Serial.printf(" Forwarding... type=%1X myAddress=0x%lX destAddr=0x%lX newNextHop=0x%lX\n",
              TYPE_DATA, myAddress, msg.destAddr, msg.nextHop);

  Serial.print("\n Sent: ");
  Serial.println(msg.payload);
  saveNodeStateToSD();
}


void Node::sendRRER(uint32_t unreachableDest, uint16_t msgID) {

  RRER rrer;
  rrer.type = TYPE_RRER;
  rrer.ttl  = 8;           // default TTL
  rrer.destAddr = unreachableDest;
  rrer.msgID = msgID;

  int transmit_code = radio.transmit((uint8_t*)&rrer, sizeof(rrer));
  if (transmit_code == RADIOLIB_ERR_NONE) {
    Serial.println("RRER sent!");
  } else {
    Serial.print("RRER TX failed, code ");
    Serial.println(transmit_code);
  }

  radio.startReceive();
}


void Node::saveMessageToSD(const char* msg, uint32_t destAddr) {
  // Ensure folder exists
  if (!SD.exists("/logs")) {
    if (!SD.mkdir("/logs")) {
      Serial.println("Failed to create /logs folder");
      return;
    }
  }

  // Convert destAddr to hex string for filename (e.g., "A1B2C3D4")
  char addrStr[9];  // 8 chars + null terminator
  sprintf(addrStr, "%08X", destAddr);  // uppercase hex

  // Create filename based on destAddr
  char filename[64];
  snprintf(filename, sizeof(filename), "/logs/node_%s.txt", addrStr);

  File file = SD.open(filename, FILE_APPEND);
  if (file) {
    file.println(msg);
    file.close();
    Serial.print("Saving to node: 0x");
    Serial.println(destAddr, HEX);
  } else {
    Serial.print("Failed to open ");
    Serial.print(filename);
    Serial.println(" for writing");
  }
}

bool Node::isDuplicateRREQ(uint32_t src, uint32_t bID) {
  // 1: Check if this RREQ is a duplicate
  for (int i = 0; i < MAX_SEEN_RREQS; i++) {
    if (seenRREQs[i].sourceAddr == src && seenRREQs[i].broadcastID == bID) {
      return true;
    }
  }

  // 2: Try to store in an empty slot
  for (int i = 0; i < MAX_SEEN_RREQS; i++) {
    if (seenRREQs[i].sourceAddr == 0) {
      seenRREQs[i].sourceAddr = src;
      seenRREQs[i].broadcastID = bID;
      rreqT[i] = currentT;
      return false;
    }
  }

  // 3: If no empty slot, find the oldest and replace it
  int oldestIndex = 0;
  unsigned long oldestTime = rreqT[0];
  for (int i = 1; i < MAX_SEEN_RREQS; i++) {
    if (rreqT[i] < oldestTime) {
      oldestTime = rreqT[i];
      oldestIndex = i;
    }
  }

  seenRREQs[oldestIndex] = {src, bID};
  rreqT[oldestIndex] = currentT;
  return false;
}


bool Node::isDuplicateRRER(uint32_t destAddr, uint16_t msgID) {
  for (int i = 0; i < MAX_SEEN_RRERS; i++) {
    if (seenRRERs[i].destAddr == destAddr &&
        seenRRERs[i].msgID == msgID) {
      return true; // duplicate
    }
  }

  // If not found, store it
  for (int i = 0; i < MAX_SEEN_RRERS; i++) {
    if (seenRRERs[i].destAddr == 0) {
      seenRRERs[i].destAddr  = destAddr;
      seenRRERs[i].msgID     = msgID;
      seenRRERs[i].timestamp = millis();
      return false;
    }
  }

  // Replace the oldest entry if full
  int oldestIndex = 0;
  unsigned long oldestTime = seenRRERs[0].timestamp;
  for (int i = 1; i < MAX_SEEN_RRERS; i++) {
    if (seenRRERs[i].timestamp < oldestTime) {
      oldestIndex = i;
      oldestTime = seenRRERs[i].timestamp;
    }
  }

  seenRRERs[oldestIndex].destAddr  = destAddr;
  seenRRERs[oldestIndex].msgID     = msgID;
  seenRRERs[oldestIndex].timestamp = millis();

  return false;
}


bool Node::isDuplicateACK(uint16_t msgID, uint32_t sourceAddr) {
  for (int i = 0; i < 5; i++) {
    if (ackTable[i].sourceAddr == sourceAddr) {
      // already seen this node
      if (ackTable[i].lastAckID == msgID) {
        return true;  // duplicate
      } else {
        ackTable[i].lastAckID = msgID;  // update
        return false;
      }
    }
  }

  // If source not found, add it to the table
  for (int i = 0; i < 5; i++) {
    if (ackTable[i].sourceAddr == 0) { // empty slot
      ackTable[i].sourceAddr = sourceAddr;
      ackTable[i].lastAckID = msgID;
      return false;
    }
  }

  // table full: you might replace oldest entry or ignore
  return false;
}


// ================= ROUTING TABLE HELPERS =================
int Node::findRoute(uint32_t destAddr, bool allowUnusable) {
    uint32_t now = millis();
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routingTable[i].valid && routingTable[i].destAddr == destAddr) {
            if (now < routingTable[i].expiryTime) {
                if (routingTable[i].usable || allowUnusable) {
                    routingTable[i].expiryTime = currentT + ROUTE_TIMEOUT_MS;
                    route_established = true;
                    return i;
                } else {
                    route_established = false;
                    return -1;
                }
            } else {
                routingTable[i].valid = false;
                routingTable[i].usable = false;
                Serial.print("FIND ROUTE: Route expired for 0x");
                Serial.println(routingTable[i].destAddr, HEX);
            }
        }
    }
    route_established = false;
    return -1;
}

void Node::updateRoutingTable(uint32_t sourceAddr, uint32_t destAddr, uint32_t lastHop,
                              uint8_t hopCount, uint32_t seqNo, uint32_t RSSI,
                              bool isReverse, uint32_t nextHop) {
  int updatedIndex = -1;

  // Direction handling
  uint32_t from = isReverse ? destAddr : sourceAddr;
  uint32_t to   = isReverse ? sourceAddr : destAddr;

  int index = findRoute(to, false);
  if(to == BROADCAST_ADDR){
    return;
  }

  if(to == myAddress){
    return;
  }

  bool isUsable = (nextHop != 0 && nextHop != BROADCAST_ADDR);

  if (index == -1) {
    for (int i = 0; i < MAX_ROUTES; i++) {
      if (!routingTable[i].valid) {
        routingTable[i] = RoutingEntry(to, lastHop, hopCount, seqNo, RSSI, isUsable, nextHop);
        routingTable[i].expiryTime = currentT + ROUTE_TIMEOUT_MS;
        Serial.print(" New ");
        Serial.print(isReverse ? "reverse" : "forward");
        Serial.print(" route added to 0x"); Serial.println(to, HEX);
        updatedIndex = i;
        break;
      }
    }
  } else {    
    if (seqNo > routingTable[index].sourceSeqNo ||
        (seqNo == routingTable[index].sourceSeqNo && hopCount < routingTable[index].hopCount)) {
      routingTable[index] = RoutingEntry(to, lastHop, hopCount, seqNo, RSSI, isUsable, nextHop);
      routingTable[index].expiryTime = currentT + ROUTE_TIMEOUT_MS;
      Serial.print(isReverse ? "Reverse" : "Forward");
      Serial.print(" route updated to 0x"); Serial.println(to, HEX);
      updatedIndex = index;
    }
  }

  if (updatedIndex != -1) {
    printRoutingTable();
    saveRoutingTable();
    }
  }

void Node::update_Expiry_time() {
    static unsigned long TableT = 0;
    if (currentT - TableT >= 2000) {
        printRoutingTable();
        TableT = currentT;
    }

    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routingTable[i].valid) {
            if (millis() > routingTable[i].expiryTime) {
                routingTable[i].valid = false;
                routingTable[i].usable = false;
                route_established = false;
                Serial.print("Route expired for 0x");
                Serial.println(routingTable[i].destAddr, HEX);
            }
        }
    }
}

void Node::printRoutingTable() {
  Serial.println("----------------------------------------------------------------------------------------");
  Serial.println("|| Dest ||Next Hop||Last Hop|| HopCount  ||  SeqNo  ||    RSSI   || Expiry ||");

  unsigned long now = millis();

  for (int i = 0; i < MAX_ROUTES; i++) {
    if (routingTable[i].valid) {
      unsigned long remaining = 0;

      if (routingTable[i].expiryTime > now) {
        remaining = routingTable[i].expiryTime - now;
      }

      unsigned long mins = (remaining / 1000) / 60;
      unsigned long secs = (remaining / 1000) % 60;

      char buffer[160];
      snprintf(buffer, sizeof(buffer),
        "|| 0x%02X ||  0x%02X  ||  0x%02X  ||    %2d     || %5lu   || %4lu ||  %02lu:%02lu ||",
        routingTable[i].destAddr,
        routingTable[i].nextHop,
        routingTable[i].lastHop,
        routingTable[i].hopCount,
        routingTable[i].sourceSeqNo,
        routingTable[i].RSSI,
        mins,
        secs
      );
      Serial.println(buffer);
    }
  }

  Serial.println("----------------------------------------------------------------------------------------");
}




void Node::saveRoutingTable() {
  if (!SD.exists("/RoutingTable")) {
    if (!SD.mkdir("/RoutingTable")) {
      Serial.println(" Failed to create RoutingTable folder");
      return;
    }
  }

  File file = SD.open("/RoutingTable/route_table.txt", FILE_WRITE);
  if (!file) {
    Serial.println(" Failed to open route_table.txt for writing");
    return;
  }

  file.println("----- Routing Table -----");

  for (int i = 0; i < MAX_ROUTES; i++) {
    if (routingTable[i].valid) {
      file.printf(
        "Src: 0x%08lX | Dest: 0x%08lX | NextHop: 0x%08lX | LastHop: 0x%08lX | HopCount: %u | "
        "SrcSeqNo: %lu | DestSeqNo: %lu | RSSI: %lu | Usable: %u | Expiry: %lu\n",
        routingTable[i].sourceAddr,
        routingTable[i].destAddr,
        routingTable[i].nextHop,
        routingTable[i].lastHop,
        routingTable[i].hopCount,
        routingTable[i].sourceSeqNo,
        routingTable[i].destSeqNo,
        routingTable[i].RSSI,
        routingTable[i].usable ? 1 : 0,
        routingTable[i].expiryTime
      );
    }
  }

  file.close();
}


void Node::loadRoutingTableFromSD() {
  File file = SD.open("/RoutingTable/route_table.txt", FILE_READ);
  if (!file) {
    Serial.println(" No routing table found on SD");
    return;
  }
  for (int index = 0; index < MAX_ROUTES; index++) {
    routingTable[index].sourceAddr = 0;
    routingTable[index].destAddr = 0;
    routingTable[index].nextHop = 0;
    routingTable[index].lastHop = 0;
    routingTable[index].hopCount = 0;
    routingTable[index].sourceSeqNo = 0;
    routingTable[index].destSeqNo = 0;
    routingTable[index].RSSI = 0;
    routingTable[index].usable = 0;
    routingTable[index].expiryTime = 0;
    routingTable[index].valid = 0;
  }

  int index = 0;
  while (file.available() && index < MAX_ROUTES) {
    String line = file.readStringUntil('\n');

    if (line.startsWith("Src: ")) {
      uint32_t src, dest, nextHop, lastHop;
      uint8_t hopCount;
      uint32_t srcSeqNo, destSeqNo, RSSI;
      uint8_t usableFlag;
      unsigned long expiry;

      int matched = sscanf(line.c_str(),
        "Src: 0x%lx | Dest: 0x%lx | NextHop: 0x%lx | LastHop: 0x%lx | HopCount: %hhu | "
        "SrcSeqNo: %lu | DestSeqNo: %lu | RSSI: %lu | Usable: %hhu | Expiry: %lu",
        &src, &dest, &nextHop, &lastHop, &hopCount,
        &srcSeqNo, &destSeqNo, &RSSI, &usableFlag, &expiry);

      if (matched == 10) {
        routingTable[index].sourceAddr = src;
        routingTable[index].destAddr = dest;
        routingTable[index].nextHop = nextHop;
        routingTable[index].lastHop = lastHop;
        routingTable[index].hopCount = hopCount;
        routingTable[index].sourceSeqNo = srcSeqNo;
        routingTable[index].destSeqNo = destSeqNo;
        routingTable[index].RSSI = RSSI;
        routingTable[index].usable = (usableFlag == 1);
        routingTable[index].expiryTime = expiry;
        routingTable[index].valid = true;
        index++;
      }
    }
  }

  file.close();
  Serial.println(" Routing table restored from SD");
}


void Node::saveNodeStateToSD() {
  if (!SD.exists("/RoutingTable")) {
    if (!SD.mkdir("/RoutingTable")) {
      static unsigned long printT = 0;
      if(currentT - printT >= 3000){
      Serial.println(" Failed to create /RoutingTable directory");
      printT = currentT;
      }
      return;
    }
  }

  File file = SD.open("/RoutingTable/state.txt", FILE_WRITE);
  if (!file) {
    Serial.println(" Failed to open state.txt for writing");
    return;
  }

  file.print("MySeqNo: "); file.println(mySeqNo);
  file.print("LastBroadcastID: "); file.println(lastBroadcastID);
  file.print("LastMsgID: "); file.println(lastMsgID);

  file.flush(); // Ensure data is written to SD
  file.close();
  //Serial.println(" Node state saved to SD");
}

void Node::loadNodeStateFromSD() {

  File file = SD.open("/RoutingTable/state.txt", FILE_READ);
  if (!file) {
    Serial.println(" No saved node state found");
    return;
  }
  if(!file.available()){
    uint32_t mySeqNo = 1;
    uint32_t lastBroadcastID = 0;
    uint16_t lastMsgID = 0;
  }
  while (file.available()) {
    String line = file.readStringUntil('\n');

    if (line.startsWith("MySeqNo: ")) {
      sscanf(line.c_str(), "MySeqNo: %lu", &mySeqNo);
    } else if (line.startsWith("LastBroadcastID: ")) {
      sscanf(line.c_str(), "LastBroadcastID: %lu", &lastBroadcastID);
    } else if (line.startsWith("LastMsgID: ")) {
      sscanf(line.c_str(), "LastMsgID: %lu", &lastMsgID);
    }
  }

  file.close();
  Serial.printf(" Node state restored: MySeqNo=%lu | LastBroadcastID=%lu | LastMsgID=%lu\n",
                mySeqNo, lastBroadcastID, lastMsgID);

}


void Node::deleteAllLogs() {
  const char* logsDir = "/logs";

  // Check if logs directory exists
  if (!SD.exists(logsDir)) {
    Serial.println(" No /logs directory found.");
    return;
  }

  File dir = SD.open(logsDir);
  if (!dir || !dir.isDirectory()) {
    Serial.println(" Failed to open /logs directory");
    return;
  }

  Serial.println("🗑️ Deleting all log files in /logs...");

  File entry;
  while ((entry = dir.openNextFile())) {
    String name = entry.name();
    if (!entry.isDirectory()) {
      if (SD.remove(name)) {
        Serial.print(" Deleted: ");
        Serial.println(name);
      } else {
        Serial.print(" Failed to delete: ");
        Serial.println(name);
      }
    }
    entry.close();
  }

  dir.close();
  Serial.println(" All log files deleted.");
}



/*
void Node::logRRERToSD(uint32_t dest, uint32_t origin) {
  if (!SD.exists("/logs")) {
    if (!SD.mkdir("/logs")) {
      Serial.println(" Failed to create /logs folder");
      return;
    }
  }

  File file = SD.open("/logs/rrer_log.txt", FILE_APPEND);
  if (file) {
    file.print("RRER | Unreachable: 0x");
    file.print(dest, HEX);
    file.print(" | Origin: 0x");
    file.print(origin, HEX);
    file.print(" | Time: ");
    file.println(millis());
    file.close();
    Serial.println(" RRER logged to SD");
  } else {
    Serial.println(" Failed to open rrer_log.txt");
  }
}

*/


