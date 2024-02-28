#include <EEPROM.h>

float length = 1550.89;

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  delay(200);

   int address = 0;

  float address1 = EEPROM.readFloat(address);
  address += sizeof(float);
  float address2 = EEPROM.readFloat(address);
  address += sizeof(float);
  float address3 = EEPROM.readFloat(address);
  address += sizeof(float);
  float address4 = EEPROM.readFloat(address);
  address += sizeof(float);

  address = 0;

  EEPROM.writeFloat(address, length);
  address += sizeof(float);
  EEPROM.writeFloat(address,address1);
  address += sizeof(float);
  EEPROM.writeFloat(address,address2);
  address += sizeof(float);
  EEPROM.writeFloat(address,address3);
  address += sizeof(float);

  Serial.println(address);
  EEPROM.commit();
  address = 0;

  Serial.println(EEPROM.readFloat(0), 2);
  address += sizeof(float);
  Serial.println(EEPROM.readFloat(4), 2);
  address += sizeof(float);
  Serial.println(EEPROM.readFloat(8), 2);
  address += sizeof(float);
  Serial.println(EEPROM.readFloat(12), 2);
  address += sizeof(float);

  Serial.println(address);
}

void loop() {
}
