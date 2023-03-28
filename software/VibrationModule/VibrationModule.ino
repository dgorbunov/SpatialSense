#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <Wire.h>
#include "Adafruit_DRV2605.h"

#define HOST_ADDRESS 3

#define RF69_FREQ 915.0
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4

#define VIBRATION_TIMEOUT 25 // ms with no effect signal to turn off

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram RF69Manager(rf69, HOST_ADDRESS);

Adafruit_DRV2605 drv;

uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t currentEffect = 0;
unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  delay(3000);

  pinMode(LED_BUILTIN, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // Manual reset.
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  while (!RF69Manager.init()) {
    Serial.println("RFM69 radio init failed. Trying again....");
    delay(250);
  }

  Serial.println("RFM69 radio init OK!");
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  // uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
  //                   0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  // rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");
  Serial.print((int)RF69_FREQ);
  Serial.println(" MHz");

  if (! drv.begin()) {
    Serial.println("Could not find DRV2605");
    while (1) delay(10);
  }

  autocal();
  // nocal();
  
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG);
  currentEffect = 0;
  drv.setWaveform(0, currentEffect);
  drv.go();
}

void autocal() {
    /**
    a. ERM_LRA—selectionwilldependondesiredactuator.
    b. FB_BRAKE_FACTOR[2:0] — A value of 2 is valid for most actuators.
    c. LOOP_GAIN[1:0] — A value of 2 is valid for most actuators.
    d. RATED_VOLTAGE[7:0] — See the Rated Voltage Programming section for calculating the correct register value.
    e. OD_CLAMP[7:0] — See the Overdrive Voltage-Clamp Programming section for calculating the correct register value.
    f. AUTO_CAL_TIME[1:0] — A value of 3 is valid for most actuators.
    g. DRIVE_TIME[3:0]—SeetheDrive-TimeProgrammingforcalculatingthecorrectregistervalue.
    h. SAMPLE_TIME[1:0] — A value of 3 is valid for most actuators.
    i. BLANKING_TIME[1:0] — A value of 1 is valid for most actuators.
    j. IDISS_TIME[1:0] — A value of 1 is valid for most actuators

    Online values:
    uiBrakeFactor = 2 
    uiLoopGain = 2 
    uiBEMFGain = 3 
    uiRatedVoltage = 61 
    uiClampVoltage = 117 
    uiAutocalTime = 2 
    uiDriveTime = 13 
    uiSampleTime = 1 
    uiBlankingTime = 2 
    uiIDissTime = 3 
    uiZCDetTime = 0    
  **/

  // Set auto-calibration mode
  drv.setMode(DRV2605_MODE_AUTOCAL);

  // Set LRA, FB_BRAKE_FACTOR, LOOP_GAIN
  uint8_t feedback_val = 0;
  const uint8_t FB_BRAKE_FACTOR = 4;
  const uint8_t LOOP_GAIN = 3;
  feedback_val |= (1 << 7);  // Set LRA mode
  feedback_val |= (FB_BRAKE_FACTOR << 4);  // Set Brake Factor to 2
  feedback_val |= (LOOP_GAIN << 2);  // Set Loop Gain to 2
  drv.writeRegister8(0x1A, feedback_val);

  // Set RATED_VOLTAGE
  // Equation: (1.85*sqrt(1-(4*300*10^-6+ 300 * 10^-6)*235))/(20.58*10^-3) = 72
  const uint8_t RATED_VOLTAGE = 78;
  drv.writeRegister8(0x16, RATED_VOLTAGE);

  // Set OD_CLAMP
  // Equation: 1.85/(21.22*10^-3) = 87
  const uint8_t OD_CLAMP = 90;
  drv.writeRegister8(0x17, OD_CLAMP);

  // Set AUTO_CAL_TIME, ZC_DET_TIME (DEFAULTS)
  uint8_t ctrl4_val = 0;
  const uint8_t ZC_DET_TIME = 0;
  const uint8_t AUTO_CAL_TIME = 3;
  ctrl4_val |= (ZC_DET_TIME << 6); // ZC_DET_TIME (do we need this?)
  ctrl4_val |= (AUTO_CAL_TIME << 4);  // Set auto cal time to 1000ms (min) to 1200ms (max)
  drv.writeRegister8(0x1E, ctrl4_val);

  // Set DRIVE_TIME
  // (((1/235)*0.5*1000)-0.5)/0.1
  const uint8_t DRIVE_TIME = 16; 
  uint8_t ctrl1_val = 0;
  ctrl1_val |= (DRIVE_TIME & 0x1f);
  drv.writeRegister8(0x1B, ctrl1_val);

  // Set SAMPLE_TIME, BLANKING_TIME, IDISS_TIME (DEFAULTS)
  uint8_t ctrl2_val = 0;
  const uint8_t SAMPLE_TIME = 3;
  const uint8_t BLANKING_TIME = 1;
  const uint8_t IDISS_TIME = 1;
  ctrl2_val |= (SAMPLE_TIME << 4);  // Set sample to 300 us
  ctrl2_val |= (BLANKING_TIME << 2);  // Set blanking time to 25 us
  ctrl2_val |= (IDISS_TIME << 0);  // Set idiss time to 25 us
  drv.writeRegister8(0x1C, ctrl2_val);

  // Start auto-calibration
  drv.go();

  uint8_t go = 1;
  uint8_t i = 0;
  const uint8_t loop_delay = 100;
  const uint8_t timeout = 4000;

  // Wait for autocalibration to finish
  do {
    go = drv.readRegister8(DRV2605_REG_GO);
    delay(loop_delay);
    ++i;
  } while (go == 1 && i < timeout / loop_delay);

  Serial.print("Autocal Result: ");
  uint8_t DIAG_RESULT = (drv.readRegister8(0x00) & ( 1 << 3 )) >> 3;
  if (!DIAG_RESULT) Serial.println("SUCCESS");
  else Serial.println("FAILED");
}

void nocal() {
  drv.useLRA();
}

void loop() {
  // Wait for a message addressed to us from the client
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (RF69Manager.recvfrom(buf, &len, &from)) {
    buf[len] = 0; // zero out remaining string
    
    // Serial.print("Got packet from #"); Serial.print(from);
    // Serial.print(" [RSSI :");
    // Serial.print(rf69.lastRssi());
    // Serial.print("] : ");
    // Serial.print((char*)buf);
    // Serial.print(" ");
    // Serial.println((int)buf[0]);

    if (currentEffect != (int)buf[0]) {
      currentEffect = (int)buf[0];
      drv.setWaveform(0, currentEffect);
      Serial.println("Set Waveform to " + (String)currentEffect);      
    }

    timer = millis();
  }

  // Serial.println(currentEffect);

  if (currentEffect != 0 && millis() - timer >= VIBRATION_TIMEOUT) {
    currentEffect = 0;
    drv.setWaveform(0, 0);
    Serial.println("Zero Waveform");
  }

  float bat_voltage = (analogRead(9) / 1023.0) * 6.6;
  if (bat_voltage < 3.5) blinkAsync(300);

  drv.go();
}

unsigned long ledTimer = 0;

void blinkAsync(int delay) {
  if (millis() - ledTimer > delay) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.println("Batt low!");
    ledTimer = millis();
  }  
}