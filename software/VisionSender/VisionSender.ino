#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define HOST_ADDRESS 8

#define RF69_FREQ 915.0
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram RF69Manager(rf69, HOST_ADDRESS);

uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t currentEffect = 0;
bool buttonPressed = false;

void setup() {
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

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
}

void loop() {
  // logic is inverted with pullup
  bool left = digitalRead(BUTTON_LEFT_PIN);
  bool right = digitalRead(BUTTON_RIGHT_PIN);
  if (!left && !buttonPressed) {
    buttonPressed = true;
    sendData(LEFT_DATA);
  } else if (!right && !buttonPressed) {
    buttonPressed = true;
    sendData(RIGHT_DATA);  
  } else {
    buttonPressed = false;
  }
}

void sendData(int data) {
  byte buff[1] = {data};
  RF69Manager.sendto(buff, 1, DEST_ADDRESS);
}