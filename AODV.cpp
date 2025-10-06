#include "AODV.h"
// Node.cpp
Preferences Node::prefs;  // allocate storage for static


unsigned long txStartTime = 0;

// ================= SETUP =================
void Node::init() {
  LoadNodeState();
  aodv.Init();
  loadRoutingTableFromSD();     
}


// ================= RECEIVE PACKETS =================
void Node::receivePacket() {
  if (rxDoneFlag && !txActive) {
    rxDoneFlag  = false;  // clear flag early
    //Serial.println("✅ IRQ fired: packet available!");

    uint8_t buffer[64];
    int len = radio.getPacketLength();

    if (len > 0 && len <= sizeof(buffer)) {
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
      case TYPE_RREQ:
        {
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

          if (rreq.sourceAddr == myAddress) {
            Serial.println("RREQ dropped: This node is the source");
            return;
          }

          if (isDuplicate(seenRREQs, MAX_SEEN_RREQS, rreq.sourceAddr, rreq.broadcastID)){
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
            0,
            rreq.destSeqNo,
            radio.getRSSI(),
            false
            );  // forward routing


          updateRoutingTable(
            rreq.sourceAddr,
            rreq.destAddr,
            0,
            0,
            rreq.sourceSeqNo,
            radio.getRSSI(),
            true,  // reverse routing
            rreq.lastHopAddr);

          if (rreq.destAddr == myAddress) {
            mySeqNo++;
            RREP rrep;

            rrep.type = TYPE_RREP;
            rrep.hopCount = rreq.hopCount;
            rrep.ttl = 8;  // Default TTL

            rrep.sourceAddr = rreq.sourceAddr;  //node nung nag send ng RREQ
            rrep.sourceSeqNo = rreq.sourceSeqNo;

            rrep.destAddr = myAddress;  // node yan ng destination ng RREQ/ yung address mismo nito
            rrep.destSeqNo = mySeqNo;   // seq number ng destination node

            rrep.lastHop = myAddress;
            rrep.nextHop = rreq.lastHopAddr;

            rrep.broadcastID = rreq.broadcastID;


            updateRoutingTable(
              rreq.sourceAddr,
              rreq.destAddr,
              0,
              rreq.hopCount,
              rreq.sourceSeqNo,
              radio.getRSSI(),
              true,  // reverse
              rreq.lastHopAddr
              );

            /*
            int transmit_code = radio.transmit((uint8_t*)&rrep, sizeof(RREP));
            if (transmit_code == RADIOLIB_ERR_NONE) {
              Serial.println("RREQ RECEIVE, RREP SENT");
            } else {
              Serial.println("TX failed code ");
              Serial.println(transmit_code);
            }
            radio.startReceive();
            */

            startTransmit((uint8_t*)&rrep, sizeof(RREP));


            saveNodeStateToNVS();
            //Serial.println("📡 Sending RREP to source.");
          } else {
            rreq.lastHopAddr = myAddress;

            /*
            int transmit_code = radio.transmit((uint8_t*)&rreq, sizeof(RREQ));
            if (transmit_code == RADIOLIB_ERR_NONE) {
              Serial.println("RREQ FORWARD");
            } else {
              Serial.println("TX failed code ");
              Serial.println(transmit_code);
            }
            radio.startReceive();
            */

            startTransmit((uint8_t*)&rreq, sizeof(RREQ));

            Serial.printf("Forwarded RREQ from %lu to %lu\n", rreq.sourceAddr, rreq.destAddr);
          }
          break;
        }


      case TYPE_RREP:
        {
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

          if (isDuplicate(seenRREPs, MAX_SEEN_RREP, rrep.destAddr, rrep.broadcastID)) {
            Serial.println("Duplicate RREP ignored.");
            return;
          }

          if(rrep.nextHop != myAddress){
            Serial.println("Not the Next Hop");
            return;
          }

          rrep.ttl--;
          rrep.hopCount++;

          int routeIndex = findRoute(rrep.destAddr, false);
          // Update route to destination (forward route)
          updateRoutingTable(
            rrep.sourceAddr,
            rrep.destAddr,
            routingTable[routeIndex].lastHop,
            rrep.hopCount,
            rrep.destSeqNo,
            radio.getRSSI(),
            false,  // forward
            rrep.lastHop
            );

          routeIndex = findRoute(rrep.sourceAddr, false);
          // Update reverse route to source
          updateRoutingTable(
            rrep.sourceAddr,
            rrep.destAddr,
            rrep.lastHop,
            routingTable[routeIndex].hopCount,
            rrep.sourceSeqNo,
            radio.getRSSI(),
            true,  // reverse
            routingTable[routeIndex].nextHop
            );

          if (rrep.sourceAddr == myAddress) {
            Serial.println("Route established.");
            route_established = true;
          } else {

            rrep.nextHop = routingTable[routeIndex].nextHop;

            /*
            int transmit_code = radio.transmit((uint8_t*)&rrep, sizeof(RREP));
            if (transmit_code == RADIOLIB_ERR_NONE) {
              Serial.println("RREP SENT TO NEXT HOP");
            } else {
              Serial.print("TX failed code ");
              Serial.println(transmit_code);
            }
            radio.startReceive();
            */

            startTransmit((uint8_t*)&rrep, sizeof(RREP));
          }
          break;
        }


      case TYPE_DATA:
        {
          if (buffer == nullptr || len < sizeof(DataMessage)) {
            Serial.println(" Invalid DATA packet.");
            return;
          }

          // Copy from buffer safely
          DataMessage msg;
          memcpy(&msg, buffer, sizeof(DataMessage));

          // TTL check
          msg.ttl--;
          if (msg.ttl == 0) {
            Serial.println("DATA dropped: TTL expired");
            return;
          }

          if (isDuplicate(dataTable, DATA_NODE, msg.sourceAddr, msg.msgID)){
             Serial.println("Duplicate DATA ignored.");
            return;
          }

          if (msg.nextHop != myAddress) {
            Serial.printf(" DATA not for me (nextHop=%lu), ignoring.\n", msg.nextHop);
            return;
            break;
          }

          Serial.println(" DATA received:");
          Serial.printf("  Source     : 0x%lX\n", msg.sourceAddr);
          Serial.printf("  Destination: 0x%lX\n", msg.destAddr);
          Serial.printf("  Next Hop   : 0x%lX\n", msg.nextHop);
          Serial.printf("  My Address : 0x%lX\n", myAddress);


          if (msg.destAddr == myAddress) {
            // Prepare ACK
            ACKMessage ack;
            ack.type = TYPE_ACK;
            ack.sourceAddr = myAddress;
            ack.destAddr = msg.sourceAddr;
            ack.msgID = msg.msgID;
            ack.ttl = 8;

            int routeIndex = findRoute(ack.destAddr, true);
            routingTable[routeIndex].expiryTime = currentT + ROUTE_TIMEOUT_MS;
            if (routeIndex != -1) {
              aodv.storePayload(msg.payload, msg.sourceAddr);
              String formatted = "> " + String(msg.sourceAddr, HEX) + ": " + String(msg.payload);
              saveMessageToSD(formatted.c_str(), msg.sourceAddr);

              ack.nextHop = routingTable[routeIndex].nextHop;

              /*
              int transmit_code = radio.transmit((uint8_t*)&ack, sizeof(ACKMessage));
              if (transmit_code == RADIOLIB_ERR_NONE) {
                Serial.print("ACK SENT");
              } else {
                Serial.print("TX failed code ");
                Serial.println(transmit_code);
              }

              radio.startReceive();
              */

              startTransmit((uint8_t*)&ack, sizeof(ACKMessage));
              //Serial.println(" Sent ACK back to source");

            } else {
              //sendRRER(myAddress, ack.msgID);
              //sendRREQ(msg.sourceAddr);

              // ======= walang nangyayari d2 =============== //

              Serial.println(" No route to source for ACK");
            }
            return;
          }
          if (msg.nextHop == myAddress) {
            // Forward the data
            int routeIndex = findRoute(msg.destAddr, true);
            if (routeIndex != -1) {
              // Validate route entry before using
              if (routingTable[routeIndex].valid && routingTable[routeIndex].usable) {
                msg.nextHop = routingTable[routeIndex].nextHop;

                /*
                int transmit_code = radio.transmit((uint8_t*)&msg, sizeof(DataMessage));
                if (transmit_code == RADIOLIB_ERR_NONE) {
                  Serial.print("DATA SENT TO NEXT HOP");
                } else {
                  Serial.print("TX failed code ");
                  Serial.println(transmit_code);
                }
                radio.startReceive();
                */

                startTransmit((uint8_t*)&msg, sizeof(DataMessage));

                Serial.printf(" Forwarded DATA to %lu (nextHop=%lu)\n", msg.destAddr, msg.nextHop);
              } else {

                // ======= wlang nangyayari d2 ============== //
                Serial.println(" Route found but not usable. Sending RREQ...");
                //sendRREQ(msg.destAddr);
                //sendRRER(msg.destAddr, msg.msgID);
              }
            }
          }

          break;
        }

      case TYPE_ACK:
        {
          if (len < sizeof(ACKMessage)) return;  // Validate size

          ACKMessage ack;
          memcpy(&ack, buffer, sizeof(ACKMessage));

          if (ack.ttl == 0) {
            Serial.println(" ACK dropped: TTL expired");
            return;
          }
          ack.ttl--;

          if (ack.nextHop != myAddress) {
            Serial.printf(" Ack not for me (nextHop=%lu), ignoring.\n", ack.nextHop);
            return;
            break;
          }


          if(isDuplicate(ackTable, ACK_NODE, ack.sourceAddr, ack.msgID)){
            Serial.println("⚠️ Duplicate ACK ignored.");
            break;
          }

          int routeIndex = findRoute(ack.sourceAddr,true);

          routingTable[routeIndex].expiryTime = currentT + ROUTE_TIMEOUT_MS;

          if (ack.destAddr == myAddress) {
            Serial.print(" ACK received for msgID: ");
            Serial.println(ack.msgID);
            message_sent = true;

            // Mark msgID as delivered here if you store sent message IDs
          } else {
            int routeIndex = findRoute(ack.destAddr, true);
            if (routeIndex != -1) {
              ack.nextHop = routingTable[routeIndex].nextHop;

              /*
              int transmit_code = radio.transmit((uint8_t*)&ack, sizeof(ACKMessage));
              if (transmit_code == RADIOLIB_ERR_NONE) {
                Serial.print("ACK RELY TO NEXT HOP");
              } else {
                Serial.print("TX failed code ");
                Serial.println(transmit_code);
              }
              radio.startReceive();
              */

              startTransmit((uint8_t*)&ack, sizeof(ACKMessage));

              Serial.print(" Forwarded ACK toward ");
              Serial.println(ack.destAddr);
            } else {
              //sendRREQ(ack.destAddr);
              //sendRRER(ack.destAddr, ack.msgID);
              Serial.println(" Cannot forward ACK: No route");
            }
          }
          break;
        }


      case TYPE_RRER:
        {
          if (len < sizeof(RRER)) return;  // Validate size
          RRER rrer;
          memcpy(&rrer, buffer, sizeof(RRER));

          if (rrer.ttl == 0) {
            Serial.println(" RRER dropped: TTL expired");
            return;
          }
          rrer.ttl--;

          if (isDuplicate(seenRRERs, MAX_SEEN_RRERS, rrer.destAddr, rrer.msgID)) {
            Serial.println("Duplicate RRER ignored.");
            return;
          }
          // Invalidate the unreachable destination route(s)
          invalidateRoute(rrer.destAddr);

          /*
          int transmit_code = radio.transmit((uint8_t*)&rrer, sizeof(RRER));
          if (transmit_code == RADIOLIB_ERR_NONE) {
            Serial.print("RRER retransmit");
          } else {
            Serial.print("TX failed code ");
            Serial.println(transmit_code);
          }
          radio.startReceive();
          */

          startTransmit((uint8_t*)&rrer, sizeof(RRER));
          break;
        }

      case TYPE_Emergency_Packet:
        {
          if (len < sizeof(EmergencyPacket)) return;  // Validate size

          EmergencyPacket emergency_packet;
          memcpy(&emergency_packet, buffer, sizeof(EmergencyPacket));

          if (emergency_packet.Emergency_node_Address == myAddress) {
            Serial.println("This node send the Emergency signal");
            return;
          }

          if (emergency_packet.ttl == 0) {
            Serial.println(" Emergency Packet dropped: TTL expired");
            return;
          }

          emergency_packet.ttl--;
          if (isDuplicate(seenEmergency, Seen_Emergency, emergency_packet.Emergency_node_Address, emergency_packet.broadcastID)){
            Serial.println("Duplicate Emergency Packet ignored.");
            return;
          }

          ///// may variable na mag cacause ng pag notif sa oled screen for acknowledgment, yung function na send emergencyack

          /*
          int transmit_code = radio.transmit((uint8_t*)&emergency_packet, sizeof(EmergencyPacket));
          if (transmit_code == RADIOLIB_ERR_NONE) {
            Serial.println("Emergency Packet rebroadcast");
          } else {
            Serial.print("TX failed code ");
            Serial.println(transmit_code);
          }
          radio.startReceive();
          */

          startTransmit((uint8_t*)&emergency_packet, sizeof(EmergencyPacket));

          break;
        }

      case TYPE_ACK_Emergency_Packet:
        {
          if (len < sizeof(Emergency_MessageACK)) return;

          Emergency_MessageACK emergency_messageAck;
          memcpy(&emergency_messageAck, buffer, sizeof(Emergency_MessageACK));

          // TTL check
          if (emergency_messageAck.ttl == 0) {
            Serial.println("Emergency Message_ACK dropped: TTL expired");
            return;
          }
          emergency_messageAck.ttl--;

          // Duplicate check
          if (isDuplicate(seenEmergencyAck, Seen_Emergency, 
                          emergency_messageAck.Emergency_Sender_Ack_Address, 
                          emergency_messageAck.Emergency_BroadcastID)){
            Serial.println("Duplicate Emergency Ack ignored.");
            return;
          }

          // If this node is the originator of the emergency
          if (emergency_messageAck.Emergency_node_Address == myAddress) {
            handleEmergencyACK(emergency_messageAck.Emergency_BroadcastID,
                               emergency_messageAck.latitude,
                               emergency_messageAck.longitude,
                               emergency_messageAck.timestamp);
          } else {
            // Rebroadcast with random backoff (if you want flood ACKs)
            delay(random(50, 200));

            /*
            int transmit_code = radio.transmit((uint8_t*)&emergency_messageAck, sizeof(Emergency_MessageACK));
            if (transmit_code == RADIOLIB_ERR_NONE) {
              Serial.println("Emergency Ack rebroadcast");
            } else {
              Serial.print("TX failed code ");
              Serial.println(transmit_code);
            }
            radio.startReceive();
            */

            startTransmit((uint8_t*)&emergency_messageAck, sizeof(Emergency_MessageACK));
          }
          break;
        }

      case TYPE_GPS_REQ:
        {
      
        if (len < sizeof(GPSMessage)) return;  // Validate size

          GPSMessage gps_message;
          memcpy(&gps_message, buffer, sizeof(GPSMessage));

          if (gps_message.sourceAddr == myAddress) {
            Serial.println("This node send the GPS Request");
            return;
          }

          if (gps_message.ttl == 0) {
            Serial.println(" Emergency Packet dropped: TTL expired");
            return;
          }

          gps_message.ttl--;
          if (isDuplicate(seenGPS, MAX_ROUTES, gps_message.sourceAddr , gps_message.msgID)){
            Serial.println("Duplicate GPS REQUEST Packet ignored.");
            return;
          }

          

          if(gps_message.nextHop != myAddress){
            Serial.println("Not the next hop");
            return;
          }

          int i = findRoute(gps_message.sourceAddr, true);
          if (i != -1) {
            routingTable[i].expiryTime = currentT + ROUTE_TIMEOUT_MS;
          }

          i = findRoute(gps_message.destAddr, true);
          if (i != -1) {
            routingTable[i].expiryTime = currentT + ROUTE_TIMEOUT_MS;
          }

          updateGPSTable(gps_message.sourceAddr,gps_message.latitude,gps_message.longitude);

          if(gps_message.REQ_REP == 1){
            Serial.println("This is a reply from the GPS REQ");
            return;
          }

          if(gps_message.destAddr == myAddress && gps_message.REQ_REP == 0){
            sendGPSRequest(gps_message.sourceAddr, 1);
          }else{
            
            if(i != -1){
            gps_message.nextHop = routingTable[i].nextHop;

            /*
            int transmit_code = radio.transmit((uint8_t*)&gps_message, sizeof(GPSMessage));
            if (transmit_code == RADIOLIB_ERR_NONE) {
              Serial.println("GPS REQ Retransmit");
            } else {
              Serial.print("TX failed code ");
              Serial.println(transmit_code);
            }
            radio.startReceive();
            */

            startTransmit((uint8_t*)&gps_message, sizeof(GPSMessage));

            }else{
              Serial.println("No Route for GPS REQ retransmit");
            }
          }

        }


        //radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
        //radio.startReceive();

    }  // closing nung switch/case statment
  }    // closing nung if statement sa packet
}  // closing ng function


// ================= TRANSMIT CODE TO AH =====================
void Node::startTransmit(uint8_t* data, size_t len) {
    if (txActive) return;

    txActive = true;
    txDoneFlag = false;

    int state = radio.startTransmit(data, len); // non-blocking

    if (state == RADIOLIB_ERR_NONE) {
        txStartTime = millis();
        Serial.println("📡 TX started");  // now prints on success
    } else {
        txActive = false;
        Serial.print("❌ TX start failed: "); Serial.println(state);
        radio.startReceive();
    }
}


void Node::updateRadio() {
   if (txActive && txDoneFlag) {
        txDoneFlag = false;
        txActive = false;
        Serial.println("TX complete, back to RX");
        radio.startReceive(); // switch back to RX
    } else if (txActive && millis() - txStartTime > 3000) {
        // TX timeout
        txActive = false;
        Serial.println("TX failed, back to RX");
        radio.startReceive();
    }
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
  
  /*
  int transmit_code = radio.transmit((uint8_t*)&rreq, sizeof(RREQ));
  if (transmit_code == RADIOLIB_ERR_NONE) {
    Serial.println("RREQ SENT");
  } else {
    Serial.println("TX failed code ");
    Serial.println(transmit_code);
  }
  Serial.print("Broadcast ID: ");
  Serial.println(rreq.broadcastID);
  radio.startReceive();
  */

  startTransmit((uint8_t*)&rreq, sizeof(RREQ));

  saveNodeStateToNVS();
}


int Node::findseqNo(uint32_t destAddr) {
  int index = findRoute(destAddr, false);
  if (index == -1) {
    return 0;
  } else {
    return routingTable[index].destSeqNo;
  }
}



// ================= SEND DATA =================
void Node::sendDataMessage(const char* text, uint32_t destAddr) {
  Serial.print("To: 0x");
  Serial.print(destAddr, HEX);
  int routeIndex = findRoute(destAddr, true);

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

  /*
  int transmit_code = radio.transmit((uint8_t*)&msg, sizeof(DataMessage));
  if (transmit_code == RADIOLIB_ERR_NONE) {
    Serial.print("DATA SENT");
  } else {
    Serial.print("TX failed code ");
    Serial.println(transmit_code);
  }
  radio.startReceive();
  */
  startTransmit((uint8_t*)&msg, sizeof(DataMessage));

  Serial.printf(" Forwarding... type=%1X myAddress=0x%lX destAddr=0x%lX newNextHop=0x%lX\n",
                TYPE_DATA, myAddress, msg.destAddr, msg.nextHop);

  Serial.print("\n Sent: ");
  Serial.println(msg.payload);
  //static uint32_t lastSavedSeqNo = 0;
  saveNodeStateToNVS();
}


void Node::sendRRER(uint32_t unreachableDest, uint16_t msgID) {

  RRER rrer;
  rrer.type = TYPE_RRER;
  rrer.ttl = 8;  // default TTL
  rrer.destAddr = unreachableDest;
  rrer.msgID = msgID;

  /*
  int transmit_code = radio.transmit((uint8_t*)&rrer, sizeof(RRER));
  if (transmit_code == RADIOLIB_ERR_NONE) {
    Serial.println("RRER sent!");
  } else {
    Serial.print("RRER TX failed, code ");
    Serial.println(transmit_code);
  }
  radio.startReceive();
  */

  startTransmit((uint8_t*)&rrer, sizeof(RRER));
}




void Node::sendGPSRequest(uint32_t destAddr, int req_rep) {
  GPSMessage gps;
  
  int i = findRoute(destAddr,true);
  if(i != -1){
    lastMsgID++;
    gps.type = TYPE_GPS_REQ;
    gps.ttl = 8;                          // default TTL
    gps.destAddr = destAddr;
    gps.sourceAddr = myAddress;       // this node's ID
    gps.msgID = lastMsgID;           // increment msg counter
    gps.REQ_REP = req_rep; // request pag 0
    gps.nextHop = routingTable[i].nextHop;   // from routing table

    gps.latitude = gpsLatitude;
    gps.longitude = gpsLongitude;

    /*
    int transmit_code = radio.transmit((uint8_t*)&gps, sizeof(GPSMessage));
    if (transmit_code == RADIOLIB_ERR_NONE) {
      Serial.println("GPS request sent!");
    } else {
      Serial.print("GPS request TX failed, code ");
      Serial.println(transmit_code);
    }
    radio.startReceive();
    */

    startTransmit((uint8_t*)&gps, sizeof(GPSMessage));

    saveNodeStateToNVS();
  }else{
    Serial.println("No Route for GPS REQUEST");
  }
}


bool Node::isDuplicate(SeenPacket* table, int tableSize, uint32_t srcOrDest, uint32_t id) {
  unsigned long now = millis();

  // 1. Check if already exists
  for (int i = 0; i < tableSize; i++) {
    if (table[i].srcOrDest == srcOrDest && table[i].id == id) {
      return true; // duplicate
    }
  }

  // 2. Empty slot → store it
  for (int i = 0; i < tableSize; i++) {
    if (table[i].srcOrDest == 0) {
      table[i].srcOrDest = srcOrDest;
      table[i].id = id;
      table[i].lastSeen = now;
      return false;
    }
  }

  // 3. Table full → replace oldest
  int oldestIndex = 0;
  unsigned long oldestTime = table[0].lastSeen;
  for (int i = 1; i < tableSize; i++) {
    if (table[i].lastSeen < oldestTime) {
      oldestTime = table[i].lastSeen;
      oldestIndex = i;
    }
  }

  table[oldestIndex].srcOrDest = srcOrDest;
  table[oldestIndex].id = id;
  table[oldestIndex].lastSeen = now;

  return false;
}



void Node::handleEmergencyACK(uint32_t ackNodeID, float lat, float lon, uint32_t timestamp) {
  unsigned long now = millis();

  // 1. Update if already exists
  for (int i = 0; i < MAX_RESPONDERS; i++) {
    if (responderTable[i].nodeID == ackNodeID) {
      responderTable[i].latitude = lat;
      responderTable[i].longitude = lon;
      responderTable[i].lastSeen = now;
      return;
    }
  }

  // 2. If not found, add in empty slot
  for (int i = 0; i < MAX_RESPONDERS; i++) {
    if (responderTable[i].nodeID == 0) {
      responderTable[i].nodeID = ackNodeID;
      responderTable[i].latitude = lat;
      responderTable[i].longitude = lon;
      responderTable[i].lastSeen = now;
      return;
    }
  }

  // 3. If table full, replace oldest entry
  int oldestIndex = 0;
  for (int i = 1; i < MAX_RESPONDERS; i++) {
    if (responderTable[i].lastSeen < responderTable[oldestIndex].lastSeen) {
      oldestIndex = i;
    }
  }

  responderTable[oldestIndex].nodeID = ackNodeID;
  responderTable[oldestIndex].latitude = lat;
  responderTable[oldestIndex].longitude = lon;
  responderTable[oldestIndex].lastSeen = now;
}



void Node::updateGPSTable(uint32_t nodeAddr, float lat, float lon) {
  unsigned long now = millis();


  // Update existing entry
  for (int i = 0; i < MAX_ROUTES; i++) {
    if (gpsTable[i].valid && gpsTable[i].nodeAddr == nodeAddr) {
      gpsTable[i].latitude = lat;
      gpsTable[i].longitude = lon;
      gpsTable[i].lastSeen = now;
      gpsTable[i].valid = true;
      return;
    }
  }

  // Insert into empty slot
  for (int i = 0; i < MAX_ROUTES; i++) {
    if (!gpsTable[i].valid) {
      gpsTable[i].nodeAddr = nodeAddr;
      gpsTable[i].latitude = lat;
      gpsTable[i].longitude = lon;
      gpsTable[i].lastSeen = now;
      gpsTable[i].valid = true;
      return;
    }
  }

  // If no empty slot, replace the oldest entry
  int oldestIndex = 0;
  unsigned long oldestTime = gpsTable[0].lastSeen;
  for (int i = 1; i < MAX_ROUTES; i++) {
    if (!gpsTable[i].valid || gpsTable[i].lastSeen < oldestTime) {
      oldestTime = gpsTable[i].lastSeen;
      oldestIndex = i;
    }
  }
  gpsTable[oldestIndex].nodeAddr = nodeAddr;
  gpsTable[oldestIndex].latitude = lat;
  gpsTable[oldestIndex].longitude = lon;
  gpsTable[oldestIndex].lastSeen = now;
  gpsTable[oldestIndex].valid = true;
}


int Node::findGPS(uint32_t nodeAddr) {
  unsigned long now = millis();

  for (int i = 0; i < MAX_ROUTES; i++) {
    if (gpsTable[i].valid && gpsTable[i].nodeAddr == nodeAddr) {
      if (now - gpsTable[i].lastSeen > GPS_ENTRY_TIMEOUT) {
        // expired → clear it
        gpsTable[i].valid = false;
        gpsTable[i].nodeAddr = 0;
        gpsTable[i].latitude = 0.0;
        gpsTable[i].longitude = 0.0;
        gpsTable[i].lastSeen = 0;
        return -1;
      }
      return i; // still valid
    }
  }
  return -1;
}








// ================= ROUTING TABLE HELPERS =================
int Node::findRoute(uint32_t destAddr, bool requireUsable) {
  uint32_t now = millis();
  for (int i = 0; i < MAX_ROUTES; i++) {
    if (routingTable[i].valid && routingTable[i].destAddr == destAddr) {
      if (now < routingTable[i].expiryTime) {
        if (requireUsable) {
          if (routingTable[i].usable) {
            route_established = true;
            return i;
          } else {
            route_established = false;
            return -1;
          }
        } else {
          return i;
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
  uint32_t to = isReverse ? sourceAddr : destAddr;

  int index = findRoute(to, false);
  if (to == BROADCAST_ADDR) {
    return;
  }

  if (to == myAddress) {
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
        Serial.print(" route added to 0x");
        Serial.println(to, HEX);
        updatedIndex = i;
        break;
      }
    }
  } else {
    if (seqNo >= routingTable[index].sourceSeqNo || (seqNo == routingTable[index].sourceSeqNo && hopCount < routingTable[index].hopCount)) {
      routingTable[index] = RoutingEntry(to, lastHop, hopCount, seqNo, RSSI, isUsable, nextHop);
      routingTable[index].expiryTime = currentT + ROUTE_TIMEOUT_MS;
      Serial.print(isReverse ? "Reverse" : "Forward");
      Serial.print(" route updated to 0x");
      Serial.println(to, HEX);
      updatedIndex = index;
    }
  }

  if (updatedIndex != -1) {
    printRoutingTable();
    saveRoutingTable();
  }
}

void Node::invalidateRoute(uint32_t destAddr) {
  bool deleted = false;

  int i = findRoute(destAddr,true);
  if(i != -1){
    if (routingTable[i].valid) {
      routingTable[i].valid = false;
      routingTable[i].usable = false;
      routingTable[i].expiryTime = 0;
      routingTable[i].nextHop = 0;
      routingTable[i].lastHop = 0;
      routingTable[i].hopCount = 0;
      routingTable[i].sourceSeqNo = 0;
      routingTable[i].destSeqNo = 0;
      routingTable[i].RSSI = 0;
      deleted = true;
      Serial.print("Route invalidated for dest 0x");
      Serial.println(destAddr, HEX);
    }
  }
  if (deleted) {
    Serial.println("A route has been invalidated because of RRER");
    saveRoutingTable();
    printRoutingTable();
  }
}

void Node::update_Expiry_time() {
  static unsigned long TableT = 0;
  if (currentT - TableT >= 5000) {
    printRoutingTable();
    TableT = currentT;
  }

  for (int i = 0; i < MAX_ROUTES; i++) {
    if (routingTable[i].valid) {
      if (millis() > routingTable[i].expiryTime) {
        invalidateRoute(routingTable[i].destAddr);
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
               secs);
      Serial.println(buffer);
    }
  }

  Serial.println("----------------------------------------------------------------------------------------");
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
  char addrStr[9];                     // 8 chars + null terminator
  sprintf(addrStr, "%08X", destAddr);  // uppercase hex

  // Create filename based on destAddr
  char filename[64];
  snprintf(filename, sizeof(filename), "/logs/node_%s.txt", addrStr);

  char timestamp[64];
  snprintf(timestamp, sizeof(timestamp), "/logs/timestamp%s.txt", addrStr); 

  File file = SD.open(filename, FILE_APPEND);
  File file_timestamp = SD.open(timestamp, FILE_APPEND);
  if (file) {
    file.println(msg);
    /// for timestamp ,data record
    file_timestamp.print(msg);
    file_timestamp.print(" ");
    file_timestamp.print(gpsHour);

    file.close();
    Serial.print("Saving to node: 0x");
    Serial.println(destAddr, HEX);
  } else {
    Serial.print("Failed to open ");
    Serial.print(filename);
    Serial.println(" for writing");
  }
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
        "Dest: 0x%08lX | NextHop: 0x%08lX | LastHop: 0x%08lX | HopCount: %u | "
        "SrcSeqNo: %lu | DestSeqNo: %lu | RSSI: %lu | Usable: %u | Expiry: %lu\n",
        routingTable[i].destAddr,
        routingTable[i].nextHop,
        routingTable[i].lastHop, 
        routingTable[i].hopCount,
        routingTable[i].sourceSeqNo,
        routingTable[i].destSeqNo,
        routingTable[i].RSSI,
        routingTable[i].usable ? 1 : 0,
        routingTable[i].expiryTime);
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
                           "Dest: 0x%lx | NextHop: 0x%lx | LastHop: 0x%lx | HopCount: %hhu | "
                           "SrcSeqNo: %lu | DestSeqNo: %lu | RSSI: %lu | Usable: %hhu | Expiry: %lu",
                           &src, &dest, &nextHop, &lastHop, &hopCount,
                           &srcSeqNo, &destSeqNo, &RSSI, &usableFlag, &expiry);

      if (matched == 10) {
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


void Node::LoadNodeState() {
  if (!prefs.begin("Node_state", false)) {
    Serial.println("Failed to open NVS namespace!");
    return;
  }

  mySeqNo = prefs.getUInt("seqNo", 1);
  lastBroadcastID = prefs.getUInt("broadcastID", 0);
  lastMsgID = prefs.getUInt("msgID", 0);

  Serial.printf("Restored: seqNo=%u, broadcastID=%u, msgID=%u\n",
                mySeqNo, lastBroadcastID, lastMsgID);

  prevSeqNo = mySeqNo;
  prevBroadcastID = lastBroadcastID;
  prevMsgID = lastMsgID;
}

void Node::saveNodeStateToNVS() {
  bool changed = false;

  if (mySeqNo != prevSeqNo) {
    prefs.putUInt("seqNo", mySeqNo);
    prevSeqNo = mySeqNo;
    changed = true;
  }
  if (lastBroadcastID != prevBroadcastID) {
    prefs.putUInt("broadcastID", lastBroadcastID);
    prevBroadcastID = lastBroadcastID;
    changed = true;
  }
  if (lastMsgID != prevMsgID) {
    prefs.putUInt("msgID", lastMsgID);
    prevMsgID = lastMsgID;
    changed = true;
  }

  if (changed) {
    Serial.println("Node state updated in NVS.");
    Serial.printf("[SAVE] seqNo=%u, broadcastID=%u, msgID=%u\n",
                  mySeqNo, lastBroadcastID, lastMsgID);
  }
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

