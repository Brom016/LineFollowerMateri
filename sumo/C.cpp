void handleWebSocket(AsyncWebSocket* server,
                     AsyncWebSocketClient* client,
                     AwsEventType type,
                     void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {                           // C0
    Serial.printf("Client #%u terhubung dari %s\n",
                  client->id(),
                  client->remoteIP().toString().c_str());
  }
  else if (type == WS_EVT_DISCONNECT) {                   // C6
    Serial.printf("Client #%u terputus\n", client->id());
    stopAll();
  }
  else if (type == WS_EVT_DATA) {
    String msg = String((char*)data).substring(0, len);   // C1

    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, msg)) return;                // C2

    float lx   = doc["lx"]   | 0.0f;                     // C3
    float ry   = doc["ry"]   | 0.0f;
    bool  stop = doc["stop"] | false;

    if (stop) { stopAll(); return; }

    int fwd  = (int)(ry / 100.0f * 255.0f);              // C4
    int turn = (int)(lx / 100.0f * 255.0f);

    setMotorLeft (constrain(fwd + turn, -255, 255));      // C5
    setMotorRight(constrain(fwd - turn, -255, 255));
  }
  else if (type == WS_EVT_ERROR) {                        // C7
    Serial.printf("WebSocket error client #%u\n", client->id());
    stopAll();
  }
}