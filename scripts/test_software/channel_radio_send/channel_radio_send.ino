// LIBRARIES
#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

// CONSTANTS
const unsigned short int RH_LOCAL_ADDRESS = 1;
const unsigned short int RH_REMOTE_ADDRESS_MU = 2;
const unsigned short int RH_REMOTE_ADDRESS_BETA = 3;
const unsigned short int RH_REMOTE_ADDRESS_RHO = 4;
const unsigned short int RH_REMOTE_ADDRESS_DELTA = 5;

const unsigned short int RH_RST = 2;
const unsigned short int RH_CS = 4;
const unsigned short int RH_INT = 3;
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
  
}

void loop(){

  
  uint8_t BUF[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t LEN;
  uint8_t FROM_ADDRESS;
  uint8_t TO_ADDRESS;

  if(Serial.available()){
    const uint8_t* data = reinterpret_cast<const uint8_t*>(Serial.readString().c_str());
    RH_Datagram.sendtoWait(data, sizeof(data), RH_REMOTE_ADDRESS_BETA);
  }
  
  
    
  
}

