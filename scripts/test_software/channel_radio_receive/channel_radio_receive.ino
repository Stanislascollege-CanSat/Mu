// LIBRARIES
#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

// CONSTANTS
const unsigned short int RH_LOCAL_ADDRESS = 3;
const unsigned short int RH_REMOTE_ADDRESS_ALPHA = 1;
const unsigned short int RH_REMOTE_ADDRESS_MU = 2;
const unsigned short int RH_REMOTE_ADDRESS_RHO = 4;
const unsigned short int RH_REMOTE_ADDRESS_DELTA = 5;

const unsigned short int RH_RST = 10;
const unsigned short int RH_CS = 12;
const unsigned short int RH_INT = 6;
const float RH_FREQ = 868.0;


// OBJECT DECLARATION
RH_RF95 RH_Driver(RH_CS, RH_INT);
RHReliableDatagram RH_Datagram(RH_Driver, RH_LOCAL_ADDRESS);






// MAIN PROGRAMME FUNCTIONS
void setup(){
  // --------------- Starting serial @ 115200 -------------------- //
  Serial.begin(115200);
  while(!Serial){
    delay(1);
  }

  // --------------- Initializing RH_Datagram -------------------- //
  if(!RH_Datagram.init()){
    Serial.print("RH_Datagram INIT failed (11)");
    exit(11);
  }

  // --------------- Setting RH_Driver frequency -------------------- //

  if(!RH_Driver.setFrequency(RH_FREQ)){
    Serial.print("RH_Driver setFrequency failed (12)");
    exit(12);
  }

  // --------------- Setting RH_Driver TxPower to 23 (maximum) -------------------- //

  RH_Driver.setTxPower(23, false);

  // --------------- Setting #retries for RH_Datagram -------------------- //

  RH_Datagram.setRetries(3);

  // --------------- Setting duration timeout for RH_Datagram -------------------- //

  RH_Datagram.setTimeout(200);
  
}

void loop(){

  uint8_t BUF[10];
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
  
  
    
  
}

