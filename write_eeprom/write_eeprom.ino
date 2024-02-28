#include <EEPROM.h>


void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  delay(200);

  int address = 16;


  // EEPROM.writeFloat(address, length);
  // address += sizeof(float);
  // EEPROM.writeFloat(address, address1);
  // address += sizeof(float);
  // EEPROM.writeFloat(address, address2);
  // address += sizeof(float);
  // EEPROM.writeFloat(address, address3);
  // address += sizeof(float);
  //address 16

  //profile
  // String sentence1 = "I love1.";  // limit ที่10 // 1
  // EEPROM.writeString(address, sentence1);
  // address += 10 + 1;
  // EEPROM.writeInt(address, 190);  // -2^31
  // address += sizeof(int);
  // EEPROM.writeInt(address, 80);  // -2^31
  // address += sizeof(int);
  // String sentence2 = "I love2.";  // limit ที่10 // 2
  // EEPROM.writeString(address, sentence2);
  // address += 10 + 1;
  // EEPROM.writeInt(address, 200);  // -2^31
  // address += sizeof(int);
  // EEPROM.writeInt(address, 70);  // -2^31
  // address += sizeof(int);
  // String sentence3 = "I love3.";  // limit ที่10 // 3
  // EEPROM.writeString(address, sentence3);
  // address += 10 + 1;
  // EEPROM.writeInt(address, 210);  // -2^31
  // address += sizeof(int);
  // EEPROM.writeInt(address, 60);  // -2^31
  // address += sizeof(int);
  // String sentence4 = "I love4.";  // limit ที่10 // 4
  // EEPROM.writeString(address, sentence4);
  // address += 10 + 1;
  // EEPROM.writeInt(address, 220);  // -2^31
  // address += sizeof(int);
  // EEPROM.writeInt(address, 50);  // -2^31
  // address += sizeof(int);
  //address 92


  // Serial.println(address);
  // EEPROM.commit();
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
  Serial.println(EEPROM.readString(address));  //16
  address += 10 + 1;
  Serial.println(address);
  Serial.println(EEPROM.readInt(address));  //27
  address += sizeof(int);
  Serial.println(address);
  Serial.println(EEPROM.readInt(address));  //31
  address += sizeof(int);
  Serial.println(address);
  Serial.println(EEPROM.readString(address));  //35
  address += 10 + 1;
  Serial.println(address);
  Serial.println(EEPROM.readInt(address));  //46
  address += sizeof(int);
  Serial.println(address);
  Serial.println(EEPROM.readInt(address));  //50
  address += sizeof(int);
  Serial.println(address);
  Serial.println(EEPROM.readString(address));  //54
  address += 10 + 1;
  Serial.println(address);
  Serial.println(EEPROM.readInt(address));  //65
  address += sizeof(int);
  Serial.println(address);
  Serial.println(EEPROM.readInt(address));  //69
  address += sizeof(int);
  Serial.println(address);
  Serial.println(EEPROM.readString(address));  //73
  address += 10 + 1;
  Serial.println(address);
  Serial.println(EEPROM.readInt(address));  //84
  address += sizeof(int);
  Serial.println(address);
  Serial.println(EEPROM.readInt(address));  //88
  address += sizeof(int);
  //92

  Serial.println(address);
}

void loop() {
}
