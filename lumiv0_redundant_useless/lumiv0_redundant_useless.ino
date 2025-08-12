// Include all the sensor libraries
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

#define RUNCAM_SERIAL Serial2
#define SEALEVELPRESSURE_HPA (1013.25)

//Pins
#define PYRO1 15
#define PYRO2 17
#define PYRO1_CS 40
#define PYRO2_CS 41
#define BUZZER_PIN 3
#define SD_CS_PIN BUILTIN_SDCARD
#define LED_R 4
#define LED_G 5
#define LED_B 6

//lora define
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
float maxAltitude = 0;
bool apogeeDetected = false;
unsigned long apogeeTime = 0;
bool pyro2Fired = false;

//kalman filter variables
float kalmanAlt = 0;
float kalmanVel = 0;
float kalmanUncertainty = 1;
float accelBias = 0;
float prevKalmanAlt = 0;
int apogeeStableCount = 0;
const int loopTime = 10;
const int windowMs = 200;
const int stableThreshold = windowMs / loopTime;

//flightstates
enum FlightState { PRELAUNCH, ASCENT, APOGEE, DESCENT, LANDED };
FlightState currentState = PRELAUNCH;

//led color and buzzer function
void setLEDColor(int r, int g, int b) {
  analogWrite(LED_R, r);
  analogWrite(LED_G, g);
  analogWrite(LED_B, b);
}
void beep(int times, int duration = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration);
  }
}

//RUNCam recording function
void startRunCamRecording() {
  RUNCAM_SERIAL.write(0xC0);
  RUNCAM_SERIAL.write(0xC0);
  Serial.println("RunCam recording triggered");
  LoRa.beginPacket(); Serial.println("RunCam recording triggered"); LoRa.endPacket();
}

float getLinearAccelZ() {
  if(bno08x.getSensorEvent(&sensorValue)) {
    if(sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      return sensorValue.un.linearAcceleration.z;
    }
  }
  return 0;
}

void updateKalman(float accelZ, float baroAlt, float dt) {
  //Prediction
  kalmanVel += (accelZ - accelBias) * dt;
  kalmanAlt += kalmanVel * dt;
  kalmanUncertainty += 0.1;

  //update with barometer
  float kalmanGain = kalmanUncertainty / (kalmanUncertainty + 0.5);
  kalmanAlt += kalmanGain * (baroAlt - kalmanAlt);
  kalmanUncertainty *= (1 - kalmanGain);
}

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  RUNCAM_SERIAL.begin(9600);
  
  pinMode(PYRO1, OUTPUT);
  pinMode(PYRO2, OUTPUT);
  pinMode(PYRO1_CS, OUTPUT);
  pinMode(PYRO2_CS, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LORA_CS, OUTPUT);
  
  //pyro pins to low
  digitalWrite(PYRO1, LOW);
  digitalWrite(PYRO2, LOW);
  digitalWrite(PYRO1_CS, LOW);
  digitalWrite(PYRO2_CS, LOW);
  digitalWrite(LORA_CS, HIGH); //pulling rf cs to high to ensure clean init

  setLEDColor(0,0,255); //Blue - prelaunch initialization
  beep(2);
  
  //lora init on spi1
  LoRa.setSPI(SPI1);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if(!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed");
    setLEDColor(255, 0, 0);
    beep(5);
  } else {
    LoRa.setTxPower(2); 
    Serial.println("Lora init done");
    LoRa.beginPacket(); LoRa.print("Lora init done"); LoRa.endPacket();
  }
  //bno init on spi0
  if(!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08X chip");
    setLEDColor(255, 0, 0);
    beep(5);
    LoRa.beginPacket(); LoRa.print("Failed to find BNO08X chip"); LoRa.endPacket();
  }
  Serial.println("BNO085 found!");
  LoRa.beginPacket(); LoRa.print("BNO085 found!"); LoRa.endPacket();
  bno08x.enableReport(SH2_LINEAR_ACCELERATION);
  //bmp init on i2c
  if(!bmp.begin_I2C()) {
    Serial.println("Failed to find BMP3XX chip");
    setLEDColor(255, 0, 0);
    beep(5);
    LoRa.beginPacket(); LoRa.print("Failed to find BMP3XX chip"); LoRa.endPacket();
  }
  Serial.println("BMP3XX found!");
  LoRa.beginPacket(); LoRa.print("BMP3XX found!"); LoRa.endPacket();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  //gps init over urart
  if(millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("Failed to find GPS");
    setLEDColor(255, 0, 0);
    beep(5);
    LoRa.beginPacket(); LoRa.print("Failed to find GPS"); LoRa.endPacket();
  }
  //sd init
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
    LoRa.beginPacket(); LoRa.print("SD init failed!"); LoRa.endPacket();
    setLEDColor(255, 0, 0);
    beep(5);
    while (1);
  }
  Serial.println("SDCard init successful!");
  LoRa.beginPacket(); LoRa.print("SDCard init successful!"); LoRa.endPacket();

  dataFile = SD.open("flight.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("time,kalmanAlt,lat,lon,acc_z");
    dataFile.close();
  }

  startRunCamRecording();
}

void loop() {
  static unsigned long lastLogTime = 0;
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  bmp.performReading();
  float baroAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  float accZ = getLinearAccelZ();
  
  updateKalman(accZ, baroAlt, dt);

  if(!apogeeDetected && kalmanAlt < prevKalmanAlt) {
    apogeeStableCount++;
    if(apogeeStableCount > stableThreshold) apogeeDetected = true;
  } else {
    apogeeStableCount = 0;
  }
  prevKalmanAlt = kalmanAlt;

  switch (currentState) {
    case PRELAUNCH:
      if (accZ > 5.0) {
        currentState = ASCENT;
        Serial.println("Launch successful -> ASCENT");
        LoRa.beginPacket(); LoRa.print("Launch successful -> ASCENT"); LoRa.endPacket();
        setLEDColor(0,255,0); //Green to indicate launch
        beep(1);
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
        setLEDColor(255,165,0); // orange
        beep(2);
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
        setLEDColor(128,0,128); //purple
        beep(3);
      }
      break;
    
    case DESCENT:
      if (baroAlt < 10.0 && now > 20000) {
        currentState = LANDED;
        Serial.print("LANDED");
        LoRa.beginPacket(); LoRa.print("LANDED"); LoRa.endPacket();
        setLEDColor(255,255,0); //yellow
        digitalWrite(BUZZER_PIN, HIGH);
      }
      break;
    
    case LANDED:
      break;
  }

  if (now - lastLogTime > 100) {
    lastLogTime = now;
    setLEDColor(128,0,128); // purple

    dataFile = SD.open("flight.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.printf("%lu,%.2f,%.6f,%.6f,%.2f\n", 
                      now,kalmanAlt,gps.location.lat(),gps.location.lng(),accZ);
      dataFile.close();
    }

    LoRa.beginPacket();
    LoRa.printf("%lu,%.2f,%.6f,%.6f,%.2f\n", 
                now,kalmanAlt,gps.location.lat(),gps.location.lng(),accZ);
    LoRa.endPacket();
  }
}
