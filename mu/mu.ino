// Mu - MotherCan Microcontroller software

#include <SPI.h>
#include <RH_RF95.h>

// RF connection vars.
#define RFM95_RST   2
#define RFM95_CS    4
#define RFM95_INT   3
#define RFM95_FREQ 915.0
int16_t packetnum = 0;

// Create RF instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM_RST, HIGH);

  // Start serial connection
  Serial.begin(9600);
  delay(100);
  Serial.println("MU is in startup");

  // RF reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("[RF] Init Error");
    while (1);
  }
  Serial.println("[RF] Init Succeded");

  // RF set frequency
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("[RF] Frequency set failed");
    while (1);
  }
  Serial.print("[RF] Frequency set to: ");
  Serial.println(RF95_FREQ);

  // RF set power
  rf95.setTxPower(23, false);
}

void loop() {
  // Wait 1 second between transmissions.
  delay(1000);
  Serial.println("Start transmission...");

  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("MSG: "); Serial.println(radiopacket);
  radiopacket[19] = 0;

  Serial.println("Sending msg");
  delay(5);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for transmission to complete");
  delay(5);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000))
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("[RSSI] -  ");
      Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is groundstation live?");
  }


}
