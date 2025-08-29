/*
DJS IMPULSE - LUMI V1.0 (Software) PROJECT NIMBUS
LUMI board software.
Needs better structuring.
yet to add :
            1) Indicator codes (LEDS and buzzers)
            2) RunCam toggling on and off logic
            3) Testing the RF settings and revamping them (Also rf packet sending)
            4) Review Pyro Arm logic

includes :
            1) apogee detection logic (tested - handheld. need to change condition values to accomodate flight conditions instead)
            2) sd card logging (tested)
            3) initialization scripts of all sensors (tested with the pcb connections)
            red - lora, purple - bno, yellow - imu, light blue - gps, blue - sd card, white - all.
*/
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP3XX.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

//Sensor Objects
Adafruit_BMP3XX bmp;
sh2_SensorValue_t sensorValue;
TinyGPSPlus gps;
SoftwareSerial ss(28,29);

#define PYRO1 16
#define PYRO2 17
#define PYRO1_CS 40
#define PYRO2_CS 41
#define BUZZER_PIN 3
#define SD_CS_PIN BUILTIN_SDCARD
#define LED_R 4
#define LED_G 5
#define LED_B 6
#define LED_R2 39
#define LED_G2 38
#define LED_B2 32
#define CAM_PWM 22
#define BUZZER 3

#define LORA_CS 0
#define LORA_RST 23
#define LORA_DIO0  34
//gps define
static const uint32_t GPSBaud = 4800;
//bno define
#define BNO08X_CS 10
#define BNO08X_RST 14
#define BNO08X_INT 2
Adafruit_BNO08x bno08x(BNO08X_RST);
File dataFile;

// Kalman filter variables
float kalman_alt = 0.0;
float kalman_gain = 0.2;  // adjust if too sensitive or sluggish
float acc_y=-5.0;
float maxAcc_y=-5.0;

// Apogee detection
float maxAltitude = -10000;
bool apogeeDetected = false;
const float DROP_THRESHOLD = 0.35;
int startupIgnoreCount = 30; // Ignore first few readings
unsigned long apogeeTime = 0;
bool pyro2Fired = false;
bool pyro1Fired = false;
bool ascent = false;
String LoRaData = "";

float gyroX, gyroY, gyroZ; 
float angVelMag;   // angular velocity magnitude

enum FlightState { PRELAUNCH, ASCENT, APOGEE, DESCENT, LANDED };
FlightState currentState = PRELAUNCH;

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  pinMode(PYRO1, OUTPUT);
  pinMode(PYRO2, OUTPUT);
  pinMode(PYRO1_CS, OUTPUT);
  pinMode(PYRO2_CS, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_B2, OUTPUT);
  pinMode(LED_G2, OUTPUT);
  pinMode(LED_R2, OUTPUT);
  pinMode(BUZZER,OUTPUT);
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_B,HIGH);
  digitalWrite(LED_R,HIGH);
  pinMode(LED_B2, LOW);
  pinMode(LED_G2, LOW);
  pinMode(LED_R2, LOW);

  digitalWrite(PYRO1, LOW);
  digitalWrite(PYRO2, LOW);
  digitalWrite(PYRO1_CS, LOW);
  digitalWrite(PYRO2_CS, LOW);
  digitalWrite(LORA_CS, HIGH); //pulling rf cs high to ensure clean init

  LoRa.setSPI(SPI1);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if(!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed");
    digitalWrite(LED_R2,LOW);
    delay(1500);
    digitalWrite(LED_R2,HIGH);
  } else {
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(8);
    LoRa.setTxPower(20); 
    Serial.println("Lora init done");
    digitalWrite(LED_R,HIGH);
    delay(1500);
    digitalWrite(LED_R,LOW);
    LoRa.beginPacket(); LoRa.printf("Lora init done"); LoRa.endPacket();
  }
  // BMP390 INIT
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    digitalWrite(LED_R2,HIGH);
    delay(1500);
    digitalWrite(LED_R2,LOW);
  }
  else {
    Serial.println("BMP detected");
    digitalWrite(LED_R,LOW);
    digitalWrite(LED_B,LOW);
    delay(1500);
      digitalWrite(LED_R,HIGH);
    digitalWrite(LED_B,HIGH);
  }
  
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // BNO08x INIT (SPI)
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("BNO08x not detected via SPI!");
    LoRa.beginPacket(); LoRa.printf("Failed to find BNO08X chip"); LoRa.endPacket();
    digitalWrite(LED_R2,HIGH);
    delay(1500);
    digitalWrite(LED_R2,LOW);
  }
  Serial.println("BNO detected"); LoRa.beginPacket(); LoRa.print("BNO085 found!"); LoRa.endPacket();
  digitalWrite(LED_R,LOW);
  digitalWrite(LED_G,LOW);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION);
  bno08x.enableReport(SH2_RAW_GYROSCOPE);
  delay(1500);
  digitalWrite(LED_R,HIGH);
  digitalWrite(LED_G,HIGH);

  //gps init over urart
  if(millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("Failed to find GPS");
    LoRa.beginPacket(); LoRa.printf("Failed to find GPS"); LoRa.endPacket();
    digitalWrite(LED_R2,HIGH);
    delay(1500);
    digitalWrite(LED_R2,LOW);
  }
  digitalWrite(LED_B,LOW);
  digitalWrite(LED_G,LOW);
  delay(1500);
  digitalWrite(LED_B,HIGH);
  digitalWrite(LED_G,HIGH);
  LoRa.beginPacket(); LoRa.printf("GPS found"); LoRa.endPacket();
  //sd init
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
    LoRa.beginPacket(); LoRa.printf("SD init failed!"); LoRa.endPacket();
    digitalWrite(LED_R2,HIGH);
    delay(1500);
    digitalWrite(LED_R2,LOW);
  }
  Serial.println("SDCard init successful!");
  LoRa.beginPacket(); LoRa.printf("SDCard init successful!"); LoRa.endPacket();
  digitalWrite(LED_B,LOW);
  delay(1500);
  digitalWrite(LED_B,HIGH);

  dataFile = SD.open("flight.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("time,kalmanAlt,lat,lon,accY,apogeeDetected");
    dataFile.close();
  }
  digitalWrite(LED_G,LOW);
  digitalWrite(LED_R,LOW);
  digitalWrite(LED_B,LOW);
  delay(1500);
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R,HIGH);
  digitalWrite(LED_B,HIGH);

  //buzzer added
  tone(BUZZER, 1000);   // play 1 kHz
  delay(500);
  tone(BUZZER, 2000);   // play 2 kHz
  delay(500);
  noTone(BUZZER);       // stop
}

void loop() {
  static unsigned long lastLogTime = 0;
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  lastTime = now;

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    LoRaData = "";
    while(LoRa.available()) {
      LoRaData += (char)LoRa.read();
    }
    LoRaData.trim();
    Serial.print("Received: ");
    Serial.println(LoRaData);
  }
  if(LoRaData == "ARM") {
    while (ss.available() > 0) {
      gps.encode(ss.read());
    }
    if (!bmp.performReading()) {
      Serial.println("Failed to read BMP!");
      return;
    }
    float rawAlt = bmp.readAltitude(1008);

    // Apply Kalman filter
    kalman_alt = kalman_gain * rawAlt + (1 - kalman_gain) * kalman_alt;
    // Read BNO08x data
    sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        acc_y = sensorValue.un.linearAcceleration.y;
        Serial.print("Altitude=");
        Serial.print(kalman_alt);
        Serial.print(" m, AccY=");
        Serial.println(acc_y, 3);
      }
    }
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_RAW_GYROSCOPE) {
        gyroX = sensorValue.un.gyroscope.x;  // rad/s
        gyroY = sensorValue.un.gyroscope.y;
        gyroZ = sensorValue.un.gyroscope.z;

        // total angular velocity magnitude
        angVelMag = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
      }
    }

    if (startupIgnoreCount > 0) {
      startupIgnoreCount--;
      return;
    }
    else {
      if(acc_y>maxAcc_y) {
        maxAcc_y = acc_y;
      }
      if (kalman_alt > maxAltitude) {
        maxAltitude = kalman_alt;
      } else if (ascent && !apogeeDetected && (maxAltitude - kalman_alt) >= DROP_THRESHOLD && acc_y<=1.5 && maxAcc_y>=50 && angVelMag < 2.5){ //(change to value during flight. this handheld test value)) 
        Serial.println("-------------Apogee Detected!!!-------------");
        apogeeDetected = true;
        digitalWrite(LED_G,LOW);
      }
      switch (currentState) {
        case PRELAUNCH:
          if (acc_y > 40) {
            currentState = ASCENT;
            Serial.println("Launch successful -> ASCENT");
            ascent = true;
            LoRa.beginPacket(); LoRa.printf("Launch successful -> ASCENT"); LoRa.endPacket();
          }
          break;
        
        case ASCENT:
          if (apogeeDetected) {
            apogeeTime = now;
            digitalWrite(PYRO1_CS, HIGH);
            digitalWrite(PYRO1, HIGH);
            pyro1Fired = true;
            digitalWrite(LED_B2,LOW);
            digitalWrite(LED_G2,LOW);
            delay(1000);
            digitalWrite(LED_B2,HIGH);
            digitalWrite(LED_G2,HIGH);
            currentState = APOGEE;
            Serial.println("Apogee Detected -> APOGEE");
            LoRa.beginPacket(); LoRa.printf("Apogee Detected -> APOGEE"); LoRa.endPacket();
          }
          break;

        case APOGEE:
          if(now - apogeeTime >= 1000) {
            digitalWrite(PYRO2_CS, HIGH);
            digitalWrite(PYRO2, HIGH);
            pyro2Fired = true;
            digitalWrite(LED_R2,LOW);
            digitalWrite(LED_G2,LOW);
            delay(1000);
            digitalWrite(LED_R2,HIGH);
            digitalWrite(LED_G2,HIGH);
            currentState = DESCENT;
            Serial.println("Second pyro fired -> DESCENT");
            LoRa.beginPacket(); LoRa.printf("Second pyro fired -> DESCENT"); LoRa.endPacket();
          }
          break;
        
        case DESCENT:
          if (kalman_alt < 10.0 && now > 20000) {
            currentState = LANDED;
            Serial.print("LANDED");
            LoRa.beginPacket(); LoRa.print("LANDED"); LoRa.endPacket();
          }
          break;
        
        case LANDED:
          LoRa.beginPacket(); LoRa.print("LANDED"); LoRa.endPacket();
          Serial.println("Landed");
          break;
      }
    }
  }
  else {
    Serial.println("Waiting for ARM command from groundstation...");
  }

    if (now - lastLogTime > 100) {
      lastLogTime = now;

      dataFile = SD.open("flight.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.printf("%lu,%.2f,%.6f,%.6f,%.2f,%d\n", 
                        now,kalman_alt,gps.location.lat(),gps.location.lng(),acc_y,apogeeDetected);
        dataFile.flush();
      }

      LoRa.beginPacket();
      LoRa.printf("%lu,%.2f,%.6f,%.6f,%.2f,%d\n", 
                  now,kalman_alt,gps.location.lat(),gps.location.lng(),acc_y, apogeeDetected);
      LoRa.endPacket();
    }
}