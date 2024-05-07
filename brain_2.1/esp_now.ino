
void init_esp_now_rx(){
  if(!digitalRead(BUTTON)){
    binding_mode = true;
    init_esp_now();
    led_color(0,10,10);
    while(!binding()){
      ;
    }
  } else {
    if(EEPROM_DATA.binding_status == 1){
      memcpy(peerInfo.peer_addr, EEPROM_DATA.bound_mac, 6);
      peerInfo.channel = EEPROM_DATA.bound_ch;  
      peerInfo.encrypt = true;      
      memcpy(peerInfo.lmk, EEPROM_DATA.encryption_key, 16);
    	init_esp_now();
    }
  }
}

void change_channel(uint8_t channel){
  esp_wifi_set_channel(channel,WIFI_SECOND_CHAN_NONE);
  current_ch = channel;
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}

bool binding(){
  if(!binding_mode){return true;}
  if(current_ch != binding_ch)change_channel(binding_ch);
  if(millis() % 500 < 250){
    led_color(0, 0, 10);
  }else{
    led_color(0, 10, 20);
  }
  if(millis()-last_sendtime > 200){
    last_sendtime = millis();
    myData.mode   = 42;
    myData.id     = 42;
    myData.x_axis = 42;
    myData.y_axis = 42;
    myData.pot_1  = 42;
    myData.sw_1   = 42;
    myData.sw_2   = 42;
    myData.ch06   = 42;
    myData.ch07   = 42;
    myData.ch08   = 42;
    myData.ch09   = 42;
    myData.ch10   = 42;
    myData.ch11   = 42;
    myData.ch12   = 42;
    myData.ch13   = 42;
    myData.ch14   = 42;
    myData.ch15   = 42;
    myData.ch16   = 42;
    // Serial.println("sending binding");
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
  return false;
}

bool isMacAddressEqual(const uint8_t* receivedMac, const uint8_t* currentMac) {
	for (int i = 0; i < 6; i++) {
		if (receivedMac[i] != currentMac[i]) {
			return false;
		}
	}
	return true;
}

bool received_binding_confirmed_packet(){
	
	uint8_t mymac[6];
	WiFi.macAddress(mymac);

  if(myData.mode    == 43
  && myData.id      == 43
  && myData.x_axis  == 43
  && myData.y_axis  == 43
  && myData.pot_1   == 43
  && myData.sw_1    == 43
  && myData.sw_2    == 43
  && isMacAddressEqual(myData.mac, mymac)
	){
    return true;
  } else {
    return false;
  }
}

void print_MAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if(binding_mode){
    memcpy(&myData, incomingData, sizeof(myData));

    if(received_binding_confirmed_packet()){
      binding_mode = false;
      sending_ch = myData.ch10;
      change_channel(sending_ch);
      memcpy(peerInfo.peer_addr, mac, 6);
      peerInfo.channel = sending_ch;  
      peerInfo.encrypt = true;      
      memcpy(peerInfo.lmk, myData.string, 16);
      if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
      }

      EEPROM_DATA.binding_status = 1;
      EEPROM_DATA.bound_ch = sending_ch;
      memcpy(EEPROM_DATA.bound_mac, peerInfo.peer_addr, sizeof(EEPROM_DATA.bound_mac));
			memcpy(EEPROM_DATA.encryption_key, myData.string, sizeof(EEPROM_DATA.encryption_key));

      // Save the updated EEPROM data
      EEPROM.put(EEPROM_ADDRES, EEPROM_DATA);
      EEPROM.commit();
      return;
    }
    // recieved binding confirmed packet, set password/channel
    // add pear to list
    return;
  }else{
  memcpy(&myData, incomingData, sizeof(myData));
  last_receive = millis();
  new_rx_data = true;
  led_state = RX_RECEIVING;
  int temp_setpoint = map(myData.x_axis,0,4095,600,-600);
  if(temp_setpoint > 6 || temp_setpoint < -6){
    Setpoint = temp_setpoint;
  }else{
    Setpoint = 0;
  }

  // Serial.print(myData.x_axis);
  // Serial.print("\t");
  // Serial.print(myData.y_axis);
  // Serial.print("\t");
  // Serial.print(myData.pot_1);
  // Serial.print("\t");
  // Serial.print(myData.sw_1);
  // Serial.print("\t");
  // Serial.println(myData.sw_2);
  }
}

void init_esp_now(){
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  if(binding_mode){
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = binding_ch;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    change_channel(binding_ch);
    // Serial.println("setting binding channel");
  }else{
		
		// Serial.print("\n\n\nPEER INFO FROM EEPROM:");
		// Serial.print("Peer MAC Address: ");
		// print_MAC(peerInfo.peer_addr);
		// Serial.print("Channel: ");
		// Serial.println(peerInfo.channel);
		// Serial.print("Encryption: ");
		// Serial.println(peerInfo.encrypt ? "Enabled" : "Disabled");
		// Serial.print("Password as hex: ");
		// for (int i = 0; i < 16; i++) {
		// 	Serial.print(peerInfo.lmk[i], HEX);
		// 	Serial.print(" ");
		// }
		// Serial.println();
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    change_channel(sending_ch);
  }  
  esp_now_register_recv_cb(OnDataRecv);
}

void switch_wireles_mode(){
  
  // Serial.print("motor status: ");
  read_drv8908_status();
  
  // init_drv8908(MOTOR_LAYOUT);
  write_register_drv8908(CONFIG_CTRL, 0b00000001); // clear faults

  // if(wireles_mode == 0){
  //   wireles_mode = 1;
  //   esp_now_deinit();
  //   init_WifiWebServer();
  //   led_color(10,0,10);
    
  //   drive_motor_A(COAST, 0);
  //   drive_motor_B(COAST, 0);
  // }else{
  //   wireles_mode = 0;
  //   server.end();
  //   init_esp_now();
  //   myPID.SetTunings(Kp,Ki,Kd);

  //   EEPROM_DATA.PID_P = Kp;
  //   EEPROM_DATA.PID_I = Ki;
  //   EEPROM_DATA.PID_D = Kd;
  //   EEPROM.put(EEPROM_ADDRES, EEPROM_DATA);
  //   EEPROM.commit();
  //   led_color(0,10,0);
  // }
}