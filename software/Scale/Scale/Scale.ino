#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <HX711_ADC.h>

#define HOST_ADDRESS 5

#define RF69_FREQ 915.0
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4

#define HX711_DOUT 2
#define HX711_SCK 5

#define FULL_READING 445 // full glass reading, 427 originally
#define CUP_READING 200 // empty cup mass, actual 225

uint8_t effects[] = {
  27
};

uint8_t destAddr[] = {
  0,
  1,
  2,
  3,
};

RH_RF69 RF69(RFM69_CS, RFM69_INT);
RHReliableDatagram RF69Manager(RF69, HOST_ADDRESS);

HX711_ADC LoadCell(HX711_DOUT, HX711_SCK);
unsigned long t = 0;
volatile boolean newDataReady;
bool stopSending = false;

// ISR for load cell
void dataReadyISR() {
  if (LoadCell.update()) {
    newDataReady = 1;
  }
}

void setup() {
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

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
  if (!RF69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed; using default");
  }

  RF69.setTxPower(20, true); // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  // uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
  //                  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  // RF69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");
  Serial.print((int)RF69_FREQ);
  Serial.println(" MHz");

  float calibrationValue = 696.0;

  LoadCell.begin();
  //LoadCell.setReverseOutput();
  unsigned long stabilizingtime = 200; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Ccheck MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
  }

  attachInterrupt(digitalPinToInterrupt(HX711_DOUT), dataReadyISR, FALLING);
}

void loop() {
  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t) {
      float i = LoadCell.getData();
      newDataReady = 0;
      // Serial.print("Load_cell output val: ");
      Serial.println(i);

      processScaleReading(i);
      t = millis();
    }
  }

  tare();
}

void tare() {
  // receive command from serial terminal, send 't' to zero scale
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  //check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Zeroed");
  }
}

void processScaleReading(int reading) {
  if (reading > CUP_READING && reading < FULL_READING && !stopSending) {
    sendData(effects[0]);
  } else if (reading >= FULL_READING) {
    stopSending = true;
  }
}

void sendData(int data) {
  byte buff[1] = {data};
  for (int i = 0; i < 4; i++) {
    RF69Manager.sendto(buff, 1, i);
  }

  Serial.print("Sent ");
  Serial.println(data);
}