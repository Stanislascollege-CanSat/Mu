// LIBRARIES
#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

// CONSTANTS
const unsigned short int RH_ADDRESS_ALPHA = 1;
const unsigned short int RH_ADDRESS_MU = 2;
const unsigned short int RH_ADDRESS_BETA = 3;
const unsigned short int RH_ADDRESS_RHO = 4;
const unsigned short int RH_ADDRESS_DELTA = 5;

const unsigned short int RH_RST = 2;
const unsigned short int RH_CS = 4;
const unsigned short int RH_INT = 3;
const float RH_FREQ = 868.0;


// OBJECT DECLARATION
RH_RF95 RH_Driver(RH_CS, RH_INT);
RHReliableDatagram RH_Datagram(RH_Driver, RH_ADDRESS_ALPHA);


// BUFFERS
String reader;






// MAIN PROGRAMME FUNCTIONS
void setup(){
  // --------------- Starting serial @ 115200 -------------------- //
  Serial.begin(115200);
  delay(1000);

  // --------------- Initializing RH_Datagram -------------------- //
  if(!RH_Datagram.init()){
    Serial.print("RH_Datagram INIT failed (11)");
    while(1);
    //exit(11);
  }

  // --------------- Setting RH_Driver frequency -------------------- //

  if(!RH_Driver.setFrequency(RH_FREQ)){
    Serial.print("RH_Driver setFrequency failed (12)");
    while(1);
    //exit(12);
  }

  // --------------- Setting RH_Driver TxPower to 23 (maximum) -------------------- //

  RH_Driver.setTxPower(23, false);

  // --------------- Setting #retries for RH_Datagram -------------------- //

  RH_Datagram.setRetries(3);

  // --------------- Setting duration timeout for RH_Datagram -------------------- //

  RH_Datagram.setTimeout(200);

  Serial.println("SETUP PASSED");


  reader = "";
  
}

void loop(){

  
  uint8_t BUF[RH_RF95_MAX_MESSAGE_LEN] = "";
  uint8_t LEN = sizeof(BUF);
  uint8_t FROM_ADDRESS;
  uint8_t TO_ADDRESS;


  if(RH_Datagram.recvfromAck(BUF, &LEN, &FROM_ADDRESS, &TO_ADDRESS)){
    // valid message received
    Serial.print("0x");
    Serial.print(FROM_ADDRESS, HEX);
    Serial.print(": ");
    Serial.println((char*) BUF);
    
  }

  if(Serial.available()){
    reader = Serial.readString();
    RH_Datagram.sendtoWait(reader.c_str(), reader.length(), RH_ADDRESS_ALPHA);
    RH_Datagram.sendtoWait(reader.c_str(), reader.length(), RH_ADDRESS_BETA);
    RH_Datagram.sendtoWait(reader.c_str(), reader.length(), RH_ADDRESS_RHO);
    RH_Datagram.sendtoWait(reader.c_str(), reader.length(), RH_ADDRESS_MU);
    RH_Datagram.sendtoWait(reader.c_str(), reader.length(), RH_ADDRESS_DELTA);

    Serial.print("Sent: " + reader + " !sizeof=");
    Serial.println(reader.length());
  }
  
  
    
  
}
