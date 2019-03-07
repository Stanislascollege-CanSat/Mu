// INCLUDES
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_Si7021.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GPS.h>
#include <MPU9250.h>



// RADIO CHANNELS
const unsigned short int RH_CHANNEL_GS_ALPHA = 1;   //
const unsigned short int RH_CHANNEL_GS_DELTA = 2;   //
const unsigned short int RH_CHANNEL_MU = 3;         // Available radio-network-channels
const unsigned short int RH_CHANNEL_BETA = 4;       //
const unsigned short int RH_CHANNEL_RHO = 5;        //

const unsigned short int RH_CHANNEL_LOCAL = RH_CHANNEL_BETA; // Set local channel, used by the programme

// PIN DEFINITIONS
const unsigned short int PIN_RH_RST = 10;    //
const unsigned short int PIN_RH_CS = 12;     // Setting: RHDriver pins
const unsigned short int PIN_RH_INT = 6;    //

const float RHDriverFreq = 868.0;   // RHDriver Frequency

// RADIO DECLARATION
RH_RF95 RHDriver(PIN_RH_CS, PIN_RH_INT);
RHReliableDatagram RHNetwork(RHDriver, RH_CHANNEL_LOCAL);






//
// SETUP FUNCTION
//

void setup(){
  // --------------- Starting serial @ 115200 -------------------- //
  Serial.begin(115200);
  delay(1000);

  Serial.println("Started setup @RHChannel_" + String(RH_CHANNEL_LOCAL));

  // --------------- Initializing RH_Datagram -------------------- //
  if(!RHNetwork.init()){
    Serial.println("ERR: 11 -> RHNetwork INIT failed. Did you assign the right pins?");
    while(1);
  }

  // --------------- Setting RH_Driver frequency -------------------- //

  if(!RHDriver.setFrequency(RHDriverFreq)){
    Serial.println("ERR: 12 -> RHDriver setFrequency failed. Check the connection with the radio chip.");
    while(1);
    //exit(12);
  }

  // --------------- Setting RH_Driver TxPower to 23 (maximum) -------------------- //

  RHDriver.setTxPower(23, false);

  // --------------- Setting #retries for RH_Datagram -------------------- //

  RHNetwork.setRetries(0);

  // --------------- Setting duration timeout for RH_Datagram -------------------- //

  RHNetwork.setTimeout(200);


  Serial.println("Setup finished in " + String(millis()) + " milliseconds.");
}



//
// LOOP FUNCTION
//



void loop(){

  uint8_t BUF[RH_RF95_MAX_MESSAGE_LEN] = "";
  uint8_t LEN = sizeof(BUF);
  uint8_t FROM_ADDRESS;
  uint8_t TO_ADDRESS;


  if(RHNetwork.recvfromAck(BUF, &LEN, &FROM_ADDRESS, &TO_ADDRESS)){
    // valid message received
    Serial.print("0x");
    Serial.print(FROM_ADDRESS, HEX);
    Serial.print(": ");
    Serial.println((char*) BUF);
    delay(1000);
    RHNetwork.sendtoWait(BUF, LEN, FROM_ADDRESS);
  }
}