void setup() {
  Serial.begin(115200);   // D1
  delay(500);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);   // D2
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopAll();

  ledcAttach(ENA, PWM_FREQ, PWM_RES);           // D3
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);                 // D4

  Serial.println("\n=== Access Point Aktif ===");
  Serial.print("IP       : ");
  Serial.println(WiFi.softAPIP());             // D5

  ws.onEvent(handleWebSocket);                 // D6
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", HTML_PAGE);  // D7
  });

  server.begin();
  Serial.println("Server siap! Buka: http://192.168.4.1");
}

void loop() {
  ws.cleanupClients(2);   // D8
}