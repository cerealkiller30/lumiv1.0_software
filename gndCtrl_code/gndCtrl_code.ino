#include <Arduino.h>
#include <LoRa.h>

#define ss 17
#define rst 21
#define dio0 20

const int sw1 = 2;
const int sw2 = 3;
const int sw3 = 4;
const int sw4 = 5;

const int numButtons = 4;
int pins[numButtons] = {sw1, sw2, sw3, sw4};

// Store last known states
int lastState[numButtons] = {HIGH, HIGH, HIGH, HIGH};

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Receiver Start");

  LoRa.setPins(ss, rst, dio0);
  if(!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed");
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setTxPower(20);

  Serial.println("LoRa Receiver Ready");
  // Configure all button pins as INPUT_PULLUP
  for (int i = 0; i < numButtons; i++) {
    pinMode(pins[i], INPUT_PULLUP);
  }
  // Wait briefly to let pins and caps stabilize
  delay(100);

  // Read initial stable state after delay
  for (int i = 0; i < numButtons; i++) {
    lastState[i] = digitalRead(pins[i]);
  }
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String LoRaData = "";
    while (LoRa.available()) {
      LoRaData += (char)LoRa.read();
    }
    LoRaData.trim();
    Serial.print("RX: RSSI = ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" ");
    Serial.println(LoRaData);
  }
  for (int i = 0; i < numButtons; i++) {
    int reading = digitalRead(pins[i]);
    //code to send data to testPad
    // Detect falling edge: button just pressed
    if (reading == LOW && lastState[i] == HIGH) {
      switch (i) {
        case 0:
          LoRa.beginPacket();
          LoRa.print("Initialization");
          LoRa.endPacket();
          Serial.println("TestPad on Initialisation ");
          break;
        case 1:
           LoRa.beginPacket();
          LoRa.print("ARM");
          LoRa.endPacket();
          Serial.println("LUMI ARMED");
          break;
        case 2:
           LoRa.beginPacket();
          LoRa.print("IGNITE");
          LoRa.endPacket();
          Serial.println("TestPad IGNITED");
          break;
        case 3:
           LoRa.beginPacket();
          LoRa.print("ABORT");
          LoRa.endPacket();
          Serial.println("LUMI ABORTED!!");
          break;
      }
    }

    // Update last known state
    lastState[i] = reading;
  }
}
