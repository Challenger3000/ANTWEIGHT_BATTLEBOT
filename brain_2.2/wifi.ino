
// void send_update() {
//   String message = ":";
//   message += "1";
//   message += ":";
//   message += value1;
//   message += ":";
//   message += value2;
//   message += ":";
//   message += value3;
//   message += ":";
//   message += value4;
//   message += ":";
//   message += value5;
//   message += ":";

//   // Send the constructed message to all connected WebSocket clients
//   ws.textAll(message);
// }

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String message = String((char*)data);
    
    int firstColon = message.indexOf(':', 1);
    int secondColon = message.indexOf(':', firstColon + 1);
    int thirdColon = message.indexOf(':', secondColon + 1);

    num1 = message.substring(1, firstColon).toDouble();
    num2 = message.substring(firstColon + 1, secondColon).toDouble();
    num3 = message.substring(secondColon + 1, thirdColon).toDouble();
    Kp = num1;
    Ki = num2;
    Kd = num3;
    led_color(255,255,255);
    delay(200);
    led_color(10,0,10);
    new_pid_values = true;
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      // Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      // Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

String processor(const String& var){

  return "meh";
}

void init_WifiWebServer(){
  WiFi.softAP(ssid, password);
  // Serial.println(WiFi.localIP());
  initWebSocket();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  server.begin();
}

void run_web_server(){
  ws.cleanupClients();  
}