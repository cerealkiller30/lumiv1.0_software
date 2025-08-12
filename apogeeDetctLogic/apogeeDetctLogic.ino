#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>

// BMP sensor object
Adafruit_BMP3XX bmp;

// BNO08x over SPI
#define BNO08X_CS 10
#define BNO08X_INT 2
#define BNO08X_RESET 14
Adafruit_BNO08x bno08x(BNO08X_RESET);
#define LED_B 6
#define LED_G 5
#define LED_R 4
#define BUZZER 3

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
void buzzerLoop() {
  digitalWrite(BUZZER,HIGH);
  delay(1000);
  digitalWrite(BUZZER,LOW);
}
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_B,HIGH);
  digitalWrite(LED_R,HIGH);

  // BMP390 INIT
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 not detected!");
    while (1);
  }
  Serial.println("BMP detected");
  digitalWrite(LED_B,LOW);
  buzzerLoop();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // BNO08x INIT (SPI)
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("BNO08x not detected via SPI!");
    while (1);
  }
  Serial.println("BNO detected");
  digitalWrite(LED_R,LOW);
  buzzerLoop();
  buzzerLoop();
  bno08x.enableReport(SH2_LINEAR_ACCELERATION);
  delay(100);
}

void loop() {
  // Read BMP sensor
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

  // Skip initial unstable readings
  if (startupIgnoreCount > 0) {
    startupIgnoreCount--;
    return;
  }
  if(acc_y>maxAcc_y) {
    maxAcc_y = acc_y;
  }
  if (kalman_alt > maxAltitude) {
    maxAltitude = kalman_alt;
  } else if (!apogeeDetected && (maxAltitude - kalman_alt) >= DROP_THRESHOLD && acc_y<=0.0 && maxAcc_y>=3.35) {
    Serial.println("-------------Apogee Detected!!!-------------");
    apogeeDetected = true;
    digitalWrite(LED_G,LOW);
    buzzerLoop();
    buzzerLoop();
    buzzerLoop();
    buzzerLoop();
  }
  delay(100);  // Adjust as needed (100ms = 10Hz loop)
}
