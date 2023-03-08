#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define HOST_ADDRESS 0

#define RF69_FREQ 915.0
#define RFM69_CS 9
#define RFM69_INT 3
#define RFM69_RST 8

#define ANGLE_DEADZONE 3
#define ANGLE_OFFSET_X 0
#define ANGLE_OFFSET_Y 0
#define MAX_ANGLE 30

#define INVERT false

RH_RF69 RF69(RFM69_CS, RFM69_INT);
RHReliableDatagram RF69Mgr(RF69, HOST_ADDRESS);
uint8_t destAddr[] = {1, 2, 3, 4};
int minVal = 265, maxVal = 402;

void setup() {
  Serial.begin(115200);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // Manual reset.
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  while (!RF69Mgr.init()) {
    Serial.println("RFM69 radio init failed. Trying again....");
    delay(250);
  }

  Serial.println("RFM69 radio init OK!");
  if (!RF69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed; using default");
  }

  RF69.setTxPower(20, true); // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

  RF69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");
  Serial.print((int)RF69_FREQ);
  Serial.println(" MHz");

  // IMU Initialization
  Serial.println("Initializing IMU...");
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("Done.");
}

void loop() {
  // IMU Readings
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  int16_t AcX = Wire.read() << 8 | Wire.read();
  int16_t AcY = Wire.read() << 8 | Wire.read();
  int16_t AcZ = Wire.read() << 8 | Wire.read();

  int16_t xAng = map(AcX, minVal, maxVal,-90 ,90);
  int16_t yAng = map(AcY, minVal, maxVal,-90 ,90);
  int16_t zAng = map(AcZ, minVal, maxVal,-90 ,90);
  
  int x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI) + ANGLE_OFFSET_X;
  int y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI) + ANGLE_OFFSET_Y;
  // int z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  x = rotationWrap(x);
  y = rotationWrap(y);
  sendData(true, calculateVibrationPercentage(x));
  sendData(false, calculateVibrationPercentage(y));
  // char radiopacket[10] = "Hello";
  // RF69Mgr.sendto((uint8_t *)radiopacket, strlen(radiopacket), 1);
}

bool sendData(boolean x, int data) {
  Serial.print(x ? "X: " : "Y: ");
  Serial.println(data);
  if (data == 0) return;
  byte buff[1];
  buff[0] = abs(data);
  uint8_t addr = x ? (data > 0 ? destAddr[0] : destAddr[1]) : (data > 0 ? destAddr[2] : destAddr[3]) ;  
  RF69Mgr.sendto(buff, 1, addr);
}

int calculateVibrationPercentage(int deg) {
  if (abs(deg) <= ANGLE_DEADZONE) {
    return 0;
  }

  int k = 1;
  if (deg < 0) k = -1;
  if(INVERT) k *= -1;

  return k * min(abs(deg) - ANGLE_DEADZONE, MAX_ANGLE - ANGLE_DEADZONE) * 255/(MAX_ANGLE - ANGLE_DEADZONE);
}

int rotationWrap(int deg) {
  if (deg > 180) {
    return -1 * (360 - deg);
  }
  return deg;
}