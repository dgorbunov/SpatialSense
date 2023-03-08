// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// who am i? (server address)
#define MY_ADDRESS     4

#define RFM69_CS      9
#define RFM69_INT     3
#define RFM69_RST     8
#define LED           13
#define VIBRATE_PIN   A5

#define INVERT_VIBRATE true

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0;  // packet counter, we increment per xmission
unsigned long onTime = 0;
bool isOn = false;

void setup() {
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(VIBRATE_PIN, OUTPUT);
  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  digitalWrite(VIBRATE_PIN, INVERT_VIBRATE == true ?  LOW : HIGH);

  Serial.println("Feather Addressed RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  // TODO: Remove encryption for packet latency
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}
// Dont put this on the stack:
uint8_t data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop() {
  // Wait for a message addressed to us from the client
  uint8_t len = sizeof(buf);
  uint8_t from, to;
  if (rf69_manager.recvfrom(buf, &len, &from, &to)) {
    buf[len] = 0; // zero out remaining string
    
    Serial.print("Got packet from #"); Serial.print(from);
    Serial.print(" [RSSI :");
    Serial.print(rf69.lastRssi());
    Serial.print("] : ");
    Serial.print((char*)buf);
    Serial.print(" ");
    Serial.println((int)buf[0]);
  
    if (!isOn) {
      digitalWrite(VIBRATE_PIN, INVERT_VIBRATE == true?  HIGH : LOW);
      isOn = true;
    }
    onTime = millis();
  } else {
    checkTimeOut();
  }
}

void checkTimeOut() {
    if (millis() - onTime > 25 && isOn) {
      digitalWrite(VIBRATE_PIN, INVERT_VIBRATE == true?  LOW : HIGH);
      onTime = millis();
      isOn = false;      
    }
}
