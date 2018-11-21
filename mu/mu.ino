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
  // put your main code here, to run repeatedly:

}
