
void init_eeprom(){  
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDRES, EEPROM_DATA);

	Serial.println();
	Serial.println();
	Serial.print("EEPROM PID PARAMS P: ");
	Serial.println(EEPROM_DATA.PID_P);
	Serial.print("EEPROM PID PARAMS I: ");
	Serial.println(EEPROM_DATA.PID_I);
	Serial.print("EEPROM PID PARAMS D: ");
	Serial.println(EEPROM_DATA.PID_D);
	Serial.print("EEPROM Binding Status: ");
	Serial.println(EEPROM_DATA.binding_status);
	Serial.print("EEPROM Bound Channel: ");
	Serial.println(EEPROM_DATA.bound_ch);
	Serial.print("EEPROM Bound MAC Address: ");
	for (int i = 0; i < 6; i++) {
			Serial.print(EEPROM_DATA.bound_mac[i], HEX);
			if (i < 5) {
					Serial.print(":");
			}
	}
	Serial.println();
	Serial.print("EEPROM Encryption Key Size: ");
	Serial.println(sizeof(EEPROM_DATA.encryption_key));
	Serial.print("EEPROM Encryption Key: ");
	for (int i = 0; i < 16; i++) {
		Serial.print(EEPROM_DATA.encryption_key[i], HEX);
		if (i < 15) {
			Serial.print(":");
		}
	}
	Serial.println();
	Serial.println();
	Serial.println();

  if(EEPROM_DATA.eeprom_structure_version != 1){
    EEPROM_DATA.eeprom_structure_version = 1;
    EEPROM_DATA.PID_P = Kp;
    EEPROM_DATA.PID_I = Ki;
    EEPROM_DATA.PID_D = Kd;
    EEPROM.put(EEPROM_ADDRES, EEPROM_DATA);
    EEPROM.commit();
  }  
}