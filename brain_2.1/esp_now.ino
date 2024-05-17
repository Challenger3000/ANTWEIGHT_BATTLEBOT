void send_voltage_telemety(){
  if(!wireles_mode && millis() - last_voltage_send > 1000){
    last_voltage_send = millis();
    float v_bat = 0.0067441860 * (float)analogRead(VSENSE);
    txData.volatage = v_bat/cell_count;
    esp_now_send(EEPROM_DATA.bound_mac, (uint8_t *) &txData, sizeof(txData));
  }
}

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
    rxData.mode   = 42;
    rxData.id     = 42;
    rxData.x_axis = 42;
    rxData.y_axis = 42;
    rxData.pot_1  = 42;
    rxData.sw_1   = 42;
    rxData.sw_2   = 42;
    rxData.sw_3   = 42;
    rxData.btn_A  = 42;
    rxData.btn_B  = 42;
    rxData.ch09   = 42;
    rxData.ch10   = 42;
    rxData.ch11   = 42;
    rxData.ch12   = 42;
    rxData.ch13   = 42;
    rxData.ch14   = 42;
    rxData.ch15   = 42;
    rxData.ch16   = 42;
    // Serial.println("sending binding");
    esp_now_send(broadcastAddress, (uint8_t *) &rxData, sizeof(rxData));
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

  if(rxData.mode    == 43
  && rxData.id      == 43
  && rxData.x_axis  == 43
  && rxData.y_axis  == 43
  && rxData.pot_1   == 43
  && rxData.sw_1    == 43
  && rxData.sw_2    == 43
  && isMacAddressEqual(rxData.mac, mymac)
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

bool rx_packet_ok() {
  if (rxData_early_check.mode == 1 &&
      rxData_early_check.id == 1 &&
      rxData_early_check.x_axis > -2 &&
      rxData_early_check.x_axis < 5000 &&
      rxData_early_check.y_axis > -2 &&
      rxData_early_check.y_axis < 5000) {
    return true;
  } else {
    return false;
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if(binding_mode){
    memcpy(&rxData, incomingData, sizeof(rxData));

    if(received_binding_confirmed_packet()){
      binding_mode = false;
      sending_ch = rxData.ch10;
      change_channel(sending_ch);
      memcpy(peerInfo.peer_addr, mac, 6);
      peerInfo.channel = sending_ch;  
      peerInfo.encrypt = true;      
      memcpy(peerInfo.lmk, rxData.string, 16);
      if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
      }

      EEPROM_DATA.binding_status = 1;
      EEPROM_DATA.bound_ch = sending_ch;
      memcpy(EEPROM_DATA.bound_mac, peerInfo.peer_addr, sizeof(EEPROM_DATA.bound_mac));
			memcpy(EEPROM_DATA.encryption_key, rxData.string, sizeof(EEPROM_DATA.encryption_key));

      // Save the updated EEPROM data
      EEPROM.put(EEPROM_ADDRES, EEPROM_DATA);
      EEPROM.commit();
      return;
    }
    // recieved binding confirmed packet, set password/channel
    // add pear to list
    return;
  }else{
    memcpy(&rxData_early_check, incomingData, sizeof(rxData_early_check));
    if(rx_packet_ok()){
      memcpy(&rxData, incomingData, sizeof(rxData));
      last_receive = millis();
      new_rx_data = true;
      led_state = RX_RECEIVING;
      int temp_setpoint = map(rxData.x_axis,0,4095,max_yaw_rate,-max_yaw_rate);
      if(temp_setpoint > 6 || temp_setpoint < -6){
        Setpoint = temp_setpoint;
      }else{
        Setpoint = 0;
      }
    }
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
  }else{
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    change_channel(sending_ch);
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void switch_wireles_mode(){
  if(wireles_mode == 0){
    led_state = WIFI_MODE;
    led_update();
    wireles_mode = 1;
    esp_now_deinit();
    init_WifiWebServer();
    drive_motor_A(COAST, 0);
    drive_motor_B(COAST, 0);
  }else{
    led_state = RX_LOST;
    led_update();
    wireles_mode = 0;
    server.end();
    init_esp_now();
    if(new_pid_values){
      myPID.SetTunings(Kp,Ki,Kd);
      EEPROM_DATA.PID_P = Kp;
      EEPROM_DATA.PID_I = Ki;
      EEPROM_DATA.PID_D = Kd;
      EEPROM.put(EEPROM_ADDRES, EEPROM_DATA);
      EEPROM.commit();
      new_pid_values = false;
    }
  }
  delay(500);
}