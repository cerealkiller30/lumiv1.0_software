// #include <Arduino.h>
// #include "../lib/headers.h"
#include <LoRa.h>
#include <SPI.h>
#define CS2 17
#define RfRst 20
#define DIO0 21
#define CS1 13
#define SCK1 10
#define MISO1 12
#define MOSI1 11
#define DT 2
#define CLK 3
#define ledb 6
#define ledg 7
#define ledr 8

enum TestPadState { IDLE, IGNITE };
TestPadState currentState = IDLE;
// State machine for rocket control         `1
void setup(){
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(7);
  LoRa.enableCrc();
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  analogWrite(ledg, 0);
  LoRa.setTxPower(20);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String LoRaData = "";
    while (LoRa.available()) {
      LoRaData += (char)LoRa.read();
    }
    LoRaData.trim(); 
    Serial.print("Received: ");
    Serial.println(LoRaData);


    switch (currentState) {
      case IDLE:
        if (LoRaData == "IGNITE") {
          currentState = IGNITE;
        }
        break;

      case IGNITE:{
        // Shouldn't be here due to delay handling in ARM case
        Serial.println("Ignition sequence started: T-10s countdown");

        // Countdown from 10s to 0s
        for (int t = 10; t >= 0; t--) {
          LoRa.beginPacket();
          LoRa.print("T minus ");
          LoRa.print(t);
          LoRa.print(" seconds");
          LoRa.endPacket();

          Serial.print("T-");
          Serial.print(t);
          Serial.println("s");

          delay(1000);
        }

        // At T=0 → fire pyro
        Serial.println("Firing pyro...");
        digitalWrite(4, HIGH);
        digitalWrite(5, HIGH);
        delay(100);   // 100ms pulse
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        Serial.println("Pyro ignition complete.");

        currentState = IDLE;
        break;
      }
    }
  }
  // Optionally, add a small delay to avoid busy-waiting
  delay(10);
}

