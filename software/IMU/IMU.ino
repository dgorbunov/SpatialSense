#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define HOST_ADDRESS 4

#define RF69_FREQ 915.0
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4

// TO BE ADJUSTED
#define ANGLE_DEADZONE 3.50
#define ANGLE_CORRECTION_ROLL 0.88
#define ANGLE_CORRECTION_PITCH -0.81
#define SAMPLERATE_DELAY_MS 30

RH_RF69 RF69(RFM69_CS, RFM69_INT);
RHReliableDatagram RF69Mgr(RF69, HOST_ADDRESS);
uint8_t destAddr[4] = {
  0, // FL
  1, // BL
  2, // FR
  3  //BR
};

uint8_t effects[1] = {
  12 // 10 is lower frequency
};

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool imuIdle = false;

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

  RF69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  // uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
  //                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

  // RF69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");
  Serial.print((int)RF69_FREQ);
  Serial.println(" MHz");

  if(!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);

  float roll = (float) -1 * event.orientation.y + ANGLE_CORRECTION_ROLL;
  float pitch = wrapPitch((float) -1 * event.orientation.z) + ANGLE_CORRECTION_PITCH;

  Serial.print(F("Roll: "));
  Serial.print(roll);
  Serial.print(F(" Pitch :"));
  Serial.println(pitch);

  float bat_voltage = (analogRead(9) / 1023.0) * 6.6;
  if (bat_voltage < 3.5) blinkAsync(300);

  sendData(deadzone(roll), deadzone(pitch));
  delay(SAMPLERATE_DELAY_MS);
}

unsigned long ledTimer = 0;

void blinkAsync(int delay) {
  if (millis() - ledTimer > delay) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.println("Batt low!");
    ledTimer = millis();
  }  
}

void sendData(float roll, float pitch) {
  // Serial.print(F("Roll: "));
  // Serial.print(roll);
  // Serial.print(F(" Pitch :"));
  // Serial.println(pitch);

  // Simplified X logic (FL: 0, BL: 1, FR: 2, BR: 3), 1 or 2 vibrations at a time
  if (roll != 0 && pitch != 0){
    uint8_t addr[1];
    if (roll > 0){
      if (pitch > 0){
        addr[0] = destAddr[1];
      } else {
        addr[0] = destAddr[0];
      }
    } else {
      if (pitch > 0){
        addr[0] = destAddr[3];
      } else {
        addr[0] = destAddr[2];
      }
    }

    writeMotors(addr, 1);

  } else if (roll != 0){
      uint8_t addr[2]={0, 0};
      if (roll > 0){
        addr[0] = destAddr[0], addr[1] = destAddr[1];
      } else {
        addr[0] = destAddr[2], addr[1] = destAddr[3];
      }

      writeMotors(addr, 2);

  } else if (pitch != 0) {
      uint8_t addr[2]={0, 0};

      if (pitch > 0){
        addr[0] = destAddr[1], addr[1] = destAddr[3];
      } else {
        addr[0] = destAddr[0], addr[1] = destAddr[2];      
      }

      writeMotors(addr, 2);

  } else if (!imuIdle) {
    imuIdle = true;
    // writeMotors(destAddr, 4);
  }
}

void writeMotors(uint8_t activeMotors[], int num) {
  imuIdle = false;
  byte buff[1] = {effects[0]};

  for (int i = 0; i < num; i++) {
    RF69Mgr.sendto(buff, 1, activeMotors[i]);
    // Serial.println("Writing to " + (String)activeMotors[i]);
  }
}

float deadzone(float input) {
  return abs(input) <= ANGLE_DEADZONE ? 0 : input;
}

float wrapPitch(float pitch) {
  if (pitch > 0) {
    return mapFloat(pitch, 180, 0, 0, 180);
  } else return mapFloat(pitch, -180, 0, 0, -180);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}