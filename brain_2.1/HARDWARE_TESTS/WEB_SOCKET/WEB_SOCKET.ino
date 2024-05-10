

// wifi website code start
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "TEST_WIFI";
const char* password = "12345678";
double num1, num2, num3;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ROBOT PID TUNER</title>
    <style>
        body, html {
            height: 100%;
            margin: 0;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            font-family: Arial, sans-serif;
        }
        .container {
            width: 100vw; /* Take up full viewport width */
            padding: 20px; /* Padding around the elements */
            box-sizing: border-box;
        }
        input, button {
            display: block;
            width: 100%; /* Full container width */
            padding: 20px; /* Larger touch targets */
            margin: 10px 0; /* Spacing between elements */
            font-size: 2em; /* Larger font size for visibility */
            box-sizing: border-box;
        }
    </style>
</head>
<body>
    <h1>ROBOT PID TUNER</h1>
    <div class="container">
        <div class="label">P</div>
        <input type="number" id="number1" placeholder="P value" maxlength="5">

        <div class="label">I</div>
        <input type="number" id="number2" placeholder="I value" maxlength="5">

        <div class="label">D</div>
        <input type="number" id="number3" placeholder="D value" maxlength="5">

        <button id="button" class="button">Send</button>
    </div>

    <script>
        var gateway = `ws://${window.location.hostname}/ws`;
        var websocket;
        window.addEventListener('load', onLoad);
        function initWebSocket() {
            console.log('Trying to open a WebSocket connection...');
            websocket = new WebSocket(gateway);
            websocket.onopen    = onOpen;
            websocket.onclose   = onClose;
            websocket.onmessage = onMessage;
        }
        function onOpen(event) {
            console.log('Connection opened');
        }
        function onClose(event) {
            console.log('Connection closed');
            setTimeout(initWebSocket, 2000);
        }
        function onMessage(event) {
            var state;
            if (event.data == "1"){
            state = "ON";
            }
            else{
            state = "OFF";
            }
            document.getElementById('state').innerHTML = state;
        }
        function onLoad(event) {
            initWebSocket();
            initButton();
        }
        function initButton() {
            document.getElementById('button').addEventListener('click', send);
        }
        function send(){
            var num1 = document.getElementById('number1').value;
            var num2 = document.getElementById('number2').value;
            var num3 = document.getElementById('number3').value;
            
            // Formatting the message in the ":10:20:30.5:" format
            var message = ":" + num1 + ":" + num2 + ":" + num3 + ":";
            websocket.send(message);
            console.log('Sent: ' + message);
        }

        function request_update(){
            websocket.send("update");
        }
    </script>
</body>
</html>

)rawliteral";

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

    Serial.print("Number 1: ");
    Serial.println(num1, 5);
    Serial.print("Number 2: ");
    Serial.println(num2, 5);
    Serial.print("Number 3: ");
    Serial.println(num3, 5);
    // if (strcmp((char*)data, "update") == 0) {
    //   // send_update();
    // }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
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
  Serial.println(var);

  return "meh";
}

void initWifiWebserver(){
  WiFi.softAP(ssid, password);
  Serial.println(WiFi.localIP());
  initWebSocket();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  server.begin();
}

void run_web_server(){
  ws.cleanupClients();  
}
// wifi website code end 


void setup(){
  Serial.begin(115200);
  while(!Serial){}
  initWifiWebserver();
}

void loop() {
  run_web_server();
}