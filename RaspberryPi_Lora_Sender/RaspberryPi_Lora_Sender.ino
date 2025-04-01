//LORA code for Transmitting Side
#include <SPI.h>
#include <LoRa.h>
void setup() {

  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readString();
    data.trim();
    Serial.print("Sending packet: ");
    Serial.println(data);
    
    // send packet
    LoRa.beginPacket();
    LoRa.print(data);
    LoRa.endPacket();
  }
}
