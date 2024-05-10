#include "WiFi.h"
 
void setup(){
  Serial.begin(115200);
  while(!Serial){
    delay(10);
  }
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}