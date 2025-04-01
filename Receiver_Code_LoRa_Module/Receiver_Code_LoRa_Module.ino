#include <SPI.h>
#include <LoRa.h>
#include <LiquidCrystal.h>

const int rs = 3, en = 4, d4 = 5, d5 = 6, d6 = 7, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define BUZZ A0
long lastUpdate;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  lcd.begin(20, 4);
  pinMode(BUZZ, OUTPUT);

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Automatic Traffic");
  lcd.setCursor(9, 1);
  lcd.print("And");
  lcd.setCursor(3, 2);
  lcd.print("Road Monitoring");
  lcd.setCursor(7, 3);
  lcd.print("System");

  digitalWrite(BUZZ, HIGH);
  delay(1000);
  digitalWrite(BUZZ, LOW);

  delay(2000);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    String receivedData;

    // Read packet
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }
    Serial.println(receivedData);

    if (receivedData == "A") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Accident Detected");
      Serial.println("Accident Detected");
      beep();
      delay(2000);
    } else if (receivedData == "B") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Dim Light Detected");
      Serial.println("Dim Light Detected");
      beep();
      delay(2000);
    } else if (receivedData == "C") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Rush Detected");
      Serial.println("Rush Detected");
      beep();
      delay(2000);
    } else if (receivedData == "D") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Dim Light Detected");
      Serial.print("Dim Light Detected ");
      lcd.setCursor(0, 1);
      lcd.print("Rush Detected");
      Serial.print("Rush Detected");
      beep();
      delay(2000);
    } else if (receivedData == "E") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Accident Detected");
      Serial.print("Accident Detected ");
      lcd.setCursor(0, 1);
      lcd.print("Dim Light Detected");
      Serial.print("Dim Light Detected");
      beep();
      delay(2000);
    } else if (receivedData == "F") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Accident Detected");
      Serial.println("Accident Detected");
      beep();
      delay(2000);
    } else if (receivedData == "G") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Accident Detected");
      Serial.print("Accident Detected ");
      lcd.setCursor(0, 1);
      lcd.print("Rush Detected ");
      Serial.print("Rush Detected ");
      lcd.setCursor(0, 2);
      lcd.print("Dim Light Detected");
      Serial.println("Dim Light Detected");
      beep();
      delay(2000);
    } else if (receivedData == "H") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Fog Detected");
      Serial.print("Fog Detected ");
      beep();
      delay(2000);
    } else {
      // Unknown data
    }
  }
  else{
    if(millis() - lastUpdate > 3000){
     lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Monitoring......."); 
    lastUpdate = millis();
    }
  }

  delay(200);
}

void beep() {
  digitalWrite(BUZZ, HIGH);
  delay(1000);
  digitalWrite(BUZZ, LOW);
  delay(1000);
}
