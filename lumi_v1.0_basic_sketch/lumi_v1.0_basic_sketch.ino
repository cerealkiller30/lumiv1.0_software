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
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
TinyGPSPlus gps;
SoftwareSerial ss(28,29);

#define PYRO1 15
#define PYRO2 17
#define PYRO1_CS 40
#define PYRO2_CS 41
#define BUZZER_PIN 3
#define SD_CS_PIN BUILTIN_SDCARD
#define LED_R 4
#define LED_G 5
#define LED_B 6

#define LORA_CS 0
#define LORA_RST 23
#define LORA_DIO0  34
//gps define
static const uint32_t GPSBaud = 4800;
//bno define
#define BNO08X_CS 10
#define BNO08X_RST 14
#define BNO08X_INT 2

File dataFile;

// Kalman filter variables
float kalman_alt = 0.0;
float kalman_gain = 0.6;  // adjust if too sensitive or sluggish
float acc_y;
float maxAcc_y=0.0;

// Apogee detection
float maxAltitude = -10000;
bool apogeeDetected = false;
const float DROP_THRESHOLD = 0.3;
int startupIgnoreCount = 30; // Ignore first few readings
unsigned long apogeeTime = 0;
bool pyro2Fired = false;

enum FlightState { PRELAUNCH, ASCENT, APOGEE, DESCENT, LANDED };
FlightState currentState = PRELAUNCH;

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  while (!Serial && millis() < 3000);
  pinMode(PYRO1, OUTPUT);
  pinMode(PYRO2, OUTPUT);
  pinMode(PYRO1_CS, OUTPUT);
  pinMode(PYRO2_CS, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_B,HIGH);
  digitalWrite(LED_R,HIGH);

  digitalWrite(PYRO1, LOW);
  digitalWrite(PYRO2, LOW);
  digitalWrite(PYRO1_CS, LOW);
  digitalWrite(PYRO2_CS, LOW);
  digitalWrite(LORA_CS, HIGH); //pulling rf cs high to ensure clean init

  LoRa.setSPI(SPI1);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if(!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed");
  } else {
    LoRa.setTxPower(2); 
    Serial.println("Lora init done");
    LoRa.beginPacket(); LoRa.print("Lora init done"); LoRa.endPacket();
  }
  // BMP390 INIT
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 not detected!");
    LoRa.beginPacket(); LoRa.print("Failed to find BMP3XX chip"); LoRa.endPacket();
    while (1);
  }
  Serial.println("BMP detected");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // BNO08x INIT (SPI)
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("BNO08x not detected via SPI!");
    LoRa.beginPacket(); LoRa.print("Failed to find BNO08X chip"); LoRa.endPacket();
    while (1);
  }
  Serial.println("BNO detected"); LoRa.beginPacket(); LoRa.print("BNO085 found!"); LoRa.endPacket();
  bno08x.enableReport(SH2_LINEAR_ACCELERATION);

  //gps init over urart
  if(millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("Failed to find GPS");
    LoRa.beginPacket(); LoRa.print("Failed to find GPS"); LoRa.endPacket();
  }
  LoRa.beginPacket(); LoRa.print("GPS found"); LoRa.endPacket();
  //sd init
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
    LoRa.beginPacket(); LoRa.print("SD init failed!"); LoRa.endPacket();
    while (1);
  }
  Serial.println("SDCard init successful!");
  LoRa.beginPacket(); LoRa.print("SDCard init successful!"); LoRa.endPacket();

  dataFile = SD.open("flight.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("time,kalmanAlt,lat,lon,acc_z");
    dataFile.close();
  }
}

void loop() {
  static unsigned long lastLogTime = 0;
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  lastTime = now;

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
  if (startupIgnoreCount > 0) {
    startupIgnoreCount--;
    return;
  }
  if(acc_y>maxAcc_y) {
    maxAcc_y = acc_y;
  }
  if (kalman_alt > maxAltitude) {
    maxAltitude = kalman_alt;
  } else if (!apogeeDetected && (maxAltitude - kalman_alt) >= DROP_THRESHOLD && acc_y<=0.0 && maxAcc_y>=3.35){ //(change to value during flight. this handheld test value)) 
    Serial.println("-------------Apogee Detected!!!-------------");
    apogeeDetected = true;
    digitalWrite(LED_G,LOW);
  }
  switch (currentState) {
    case PRELAUNCH:
      if (acc_y > 5.0) {
        currentState = ASCENT;
        Serial.println("Launch successful -> ASCENT");
        LoRa.beginPacket(); LoRa.print("Launch successful -> ASCENT"); LoRa.endPacket();
      }
      break;
    
    case ASCENT:
      if (apogeeDetected) {
        apogeeTime = now;
        digitalWrite(PYRO1_CS, HIGH);
        digitalWrite(PYRO1, HIGH);
        currentState = APOGEE;
        Serial.println("Apogee Detected -> APOGEE");
        LoRa.beginPacket(); LoRa.print("Apogee Detected -> APOGEE"); LoRa.endPacket();
      }
      break;

    case APOGEE:
      if(now - apogeeTime >= 1000) {
        digitalWrite(PYRO2_CS, HIGH);
        digitalWrite(PYRO2, HIGH);
        pyro2Fired = true;
        currentState = DESCENT;
        Serial.println("Second pyro fired -> DESCENT");
        LoRa.beginPacket(); LoRa.print("Second pyro fired -> DESCENT"); LoRa.endPacket();
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
      break;
  }

  if (now - lastLogTime > 100) {
    lastLogTime = now;

    dataFile = SD.open("flight.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.printf("%lu,%.2f,%.6f,%.6f,%.2f\n", 
                      now,kalman_alt,gps.location.lat(),gps.location.lng(),acc_y);
      dataFile.close();
    }

    LoRa.beginPacket();
    LoRa.printf("%lu,%.2f,%.6f,%.6f,%.2f\n", 
                now,kalman_alt,gps.location.lat(),gps.location.lng(),acc_y);
    LoRa.endPacket();
  }
}