
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  int rssi = ppkt->rx_ctrl.rssi;
  rssi_last = rssi;
}

void init_data_structures(){
  txData.mode   = 0;
  txData.id     = 0;
  txData.x_axis = 0;
  txData.y_axis = 0;
  txData.pot_1  = 0;
  txData.sw_1   = 0;
  txData.sw_2   = 0;
  txData.sw_3   = 0;
  txData.btn_A  = 0;
  txData.btn_B  = 0;
  txData.ch09   = 0;
  txData.ch10   = 0;
  txData.ch11   = 0;
  txData.ch12   = 0;
  txData.ch13   = 0;
  txData.ch14   = 0;
  txData.ch15   = 0;
  txData.ch16   = 0;
}

void print_rssi_list(){
  for (int i = 0; i < rssi_list_index; i++) {
    Serial.print("=======================");
    Serial.print("INDEX: ");
    Serial.println(i);
    Serial.print("MAC: ");
    print_MAC(rssi_list[i].mac_address);
    Serial.print("RSSI: ");
    Serial.println(rssi_list[i].rssi);
    Serial.print("RECEIVED PACKETS: ");
    Serial.println(rssi_list[i].received_packets);
  }
}


void binding(){
  if(current_ch != binding_ch)change_channel(binding_ch);
  int strongest_rssi_index_local = 0;
  int strongest_rssi_local = -200;
  for (int i = 0; i < rssi_list_index; i++) {
    if(rssi_list[i].rssi > -50 && rssi_list[i].rssi > strongest_rssi_local){
      strongest_rssi_local = i;
    }
  }  
  if(strongest_rssi_local != -200 && strongest_rssi_local != strongest_rssi_index){
    strongest_rssi_index = strongest_rssi_local;
    last_strongest_receive = millis();
  }

  if(millis()-last_strongest_receive > 1000 && rssi_list[strongest_rssi_index].received_packets > 10 && rssi_list[strongest_rssi_index].rssi > -50 ){
    randomSeed((unsigned long)esp_random());
    pmk_key_str[0] = '\0';  
    // Serial.println("Generating a random 16-byte PMK:");

    // Append each random byte as a two-character hex value to the string
    for (int i = 0; i < 16; i++) {
      uint8_t randomByte = random(0, 256);  // Generate a random byte
      pmk_key_str[i] = randomByte;  // Format byte as hex and append
    }

    sending_ch = random(0, 13);
    txData.mode   = 43;
    txData.id     = 43;
    txData.x_axis = 43;
    txData.y_axis = 43;
    txData.pot_1  = 43;
    txData.sw_1   = 43;
    txData.sw_2   = 43;
    txData.ch10   = sending_ch;
    strcpy(txData.string, pmk_key_str);
    memcpy(txData.mac, rssi_list[strongest_rssi_index].mac_address, 6);
    EEPROM_DATA.binding_status = 1;
    EEPROM_DATA.bound_ch = sending_ch;
    memcpy(EEPROM_DATA.bound_mac, rssi_list[strongest_rssi_index].mac_address, sizeof(EEPROM_DATA.bound_mac));
    memcpy(EEPROM_DATA.encryption_key, txData.string, sizeof(txData.string));
    EEPROM.put(EEPROM_ADDRES, EEPROM_DATA);
    EEPROM.commit();
    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
    delay(100);
    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
    delay(100);
    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
    delay(100);
    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
    esp_wifi_set_promiscuous(false);

    change_channel(sending_ch);
    // Serial.println("binding confirmed");
    memcpy(peerInfo.peer_addr, rssi_list[strongest_rssi_index].mac_address, 6);
    peerInfo.channel = sending_ch;
    peerInfo.encrypt = true;
    memcpy(peerInfo.lmk, txData.string, 16);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }   
    esp_now_register_send_cb(OnDataSent);  
    state = SENDING;
    return;
  }

  if(millis()-last_sendtime > 200){
    last_sendtime = millis();
    txData.mode   = 42;
    txData.id     = 42;
    txData.x_axis = 42;
    txData.y_axis = 42;
    txData.pot_1  = 42;
    txData.sw_1   = 42;
    txData.sw_2   = 42;
    txData.sw_3   = 42;
    txData.btn_A  = 42;
    txData.btn_B  = 42;
    txData.ch09   = 42;
    txData.ch10   = 42;
    txData.ch11   = 42;
    txData.ch12   = 42;
    txData.ch13   = 42;
    txData.ch14   = 42;
    txData.ch15   = 42;
    txData.ch16   = 42;
    // Serial.println("SENDING BINDING");
    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
  }
}

void change_channel(uint8_t channel){
  esp_wifi_set_channel(channel,WIFI_SECOND_CHAN_NONE);
  current_ch = channel;
}

void print_MAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}

bool received_binding_packet(){
  if(txData.mode == 42
  && txData.id == 42
  && txData.x_axis == 42
  && txData.y_axis == 42
  && txData.pot_1 == 42
  && txData.sw_1 == 42
  && txData.sw_2 == 42){
    return true;
  } else {
    return false;
  }  
}

int findMacAddress(const uint8_t* mac_address) {
  for (int i = 0; i < rssi_list_index; i++) {
    if (memcmp(rssi_list[i].mac_address, mac_address, sizeof(rssi_list[i].mac_address)) == 0) {
      return i;
    }
  }
  return -1;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){
    failed_packet_count = 0;
    rx_lost = false;
  }else{
    failed_packet_count++;
    if(failed_packet_count > 5){
      rx_lost = true;
    }
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&txData, incomingData, sizeof(txData));
  last_receive = millis();
  
  if(state == BINDING){
    if(received_binding_packet()){
      int index = findMacAddress(mac);
      
      if(index == -1){
        if(rssi_list_index < rssi_list_size){
          for (int i = 0; i < 6; i++) {
            rssi_list[rssi_list_index].mac_address[i] = mac[i];
          }
          // memcpy(rssi_list[rssi_list_index].mac_address, mac, sizeof(mac));
          rssi_list[rssi_list_index].rssi = rssi_last;
          rssi_list[rssi_list_index].received_packets = 1;
          rssi_list_index++;
        }else{
          Serial.println("RSSI LIST FULL");
          while (true)
          {
            delay(1000);
          }
          
        }
      }else{
        for (int i = 0; i < 6; i++) {
          rssi_list[index].mac_address[i] = mac[i];
        }
        rssi_list[index].rssi = rssi_last;
        rssi_list[index].received_packets++;
      }
    }
  }else if(state == SENDING){
    new_rx_data = true;
    memcpy(&rxData, incomingData, sizeof(rxData));
    // Serial.print("Bytes received: ");
    // Serial.println(len);
    // Serial.print("v: ");
    // Serial.println(rxData.volatage);
    // Serial.println();

  }
}

void init_esp_now(){
  WiFi.mode(WIFI_STA);
  
  esp_wifi_set_channel(sending_ch,WIFI_SECOND_CHAN_NONE);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  if(state == BINDING){
    Serial.println("Binding mode");
    // Register binding peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = binding_ch;  
    peerInfo.encrypt = false;    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    change_channel(binding_ch);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
  }else if(state == SENDING){
    sending_ch = EEPROM_DATA.bound_ch;
    memcpy(peerInfo.peer_addr, EEPROM_DATA.bound_mac, 6);
    peerInfo.channel = EEPROM_DATA.bound_ch;  
    peerInfo.encrypt = true;      
    memcpy(peerInfo.lmk, EEPROM_DATA.encryption_key, 16);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    // Serial.println("Sending mode");
    change_channel(sending_ch);
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
  }
}


void send_data(){
  calculate_expo_12_Bit(mapWithMidpoint(constrain(analogRead(g_y),EEPROM_DATA.calib_y_low,EEPROM_DATA.calib_y_high), EEPROM_DATA.calib_y_low, ch2_offset, EEPROM_DATA.calib_y_high, 0, 4095), 0.3);

  txData.mode   = 1;
  txData.id     = 1;
  // if(get_switch_2()==1){
  if(false){
    txData.x_axis = 4095 -  calculate_expo_12_Bit(mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095),0.5);
  // }else if(get_switch_2()==2){
  }else if(true){
    txData.x_axis = 4095 -  calculate_expo_12_Bit(mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095),0.25);
  // }else if(get_switch_2()==3){
  }else if(false){
    txData.x_axis = 4095 - mapWithMidpoint(constrain(analogRead(g_x),EEPROM_DATA.calib_x_low ,EEPROM_DATA.calib_x_high), EEPROM_DATA.calib_x_low, ch1_offset, EEPROM_DATA.calib_x_high, 0, 4095);
  }
  txData.y_axis =mapWithMidpoint(constrain(analogRead(g_y),EEPROM_DATA.calib_y_low ,EEPROM_DATA.calib_y_high), EEPROM_DATA.calib_y_low, ch2_offset, EEPROM_DATA.calib_y_high, 0, 4095);
  txData.pot_1  = analogRead(pot);
  txData.sw_1   = get_switch_1();
  txData.sw_2   = get_switch_2();
  txData.sw_3   = get_switch_3();
  txData.btn_A  = get_button_A();
  txData.btn_B  = get_button_B();
  txData.ch09   = 50;
  txData.ch10   = 50;
  txData.ch11   = 50;
  txData.ch12   = 50;
  txData.ch13   = 50;
  txData.ch14   = 50;
  txData.ch15   = 50;
  txData.ch16   = 130;
  memcpy(txData.mac, peerInfo.peer_addr, 6);
  esp_now_send(peerInfo.peer_addr, (uint8_t *) &txData, sizeof(txData));
}
