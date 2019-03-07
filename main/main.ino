// INCLUDES
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_Si7021.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GPS.h>
#include <MPU9250.h>

// PIN DEFINITIONS
const unsigned short int PIN_RH_RST = 10;    //
const unsigned short int PIN_RH_CS = 12;     // Setting: RHDriver pins
const unsigned short int PIN_RH_INT = 6;    //
const unsigned short int PIN_BUZZ = 11;
const unsigned short int PIN_A_BAT = 9;

const string PIN_MCU_LED = LED_BUILTIN;

// RADIO CHANNELS
const unsigned short int RH_CHANNEL_GS_ALPHA = 1;   //
const unsigned short int RH_CHANNEL_GS_DELTA = 2;   //
const unsigned short int RH_CHANNEL_MU = 3;         // Available radio-network-channels
const unsigned short int RH_CHANNEL_BETA = 4;       //
const unsigned short int RH_CHANNEL_RHO = 5;        //

const unsigned short int RH_CHANNEL_LOCAL = RH_CHANNEL_MU; // Set local channel, used by the programme

const float RH_DRIVER_FREQ = 868.0;   // RHDriver Frequency

// SERVO VARIABLES
const int SERVOMIN 150 // this is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX 600 // this is the 'maximum' pulse length count (out of 4096)
const unsigned short int SERVO_PINS = 0;
const unsigned short int SERVO_HULL = 1;

// RADIO DECLARATION
RH_RF95 RHDriver(PIN_RH_CS, PIN_RH_INT);
RHReliableDatagram RHNetwork(RHDriver, RH_CHANNEL_LOCAL);

// PWM DECLARATION
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x44);

//
// SETUP FUNCTION
//

void setup(){
  // --------------- Set pin and hull position -------------------- //
  pwm.begin();

  pwm.setPWMFreq(60);

  // open hull
  pwm.setPWM(SERVO_HULL, 0, map(79 - 5, 0, 180, SERVOMIN, SERVOMAX));
  delay(1000);
  pwm.setPWM(SERVO_HULL, 0, map(79, 0, 180, SERVOMIN, SERVOMAX));

  // --------------- Startup charm -------------------- //
  tone(BUZZ, 1000);
  digitalWrite(MCU_LED, HIGH);
  delay(500);
  noTone(BUZZ);
  digitalWrite(MCU_LED, LOW);
  delay(300);
  tone(BUZZ,1500);
  digitalWrite(MCU_LED, HIGH);
  delay(400);
  noTone(BUZZ);
  digitalWrite(MCU_LED, LOW);

  // --------------- Starting serial @ 115200 -------------------- //
  SerialUSB.begin(115200);
  delay(100);

  SerialUSB.println("Started setup @RHChannel_" + String(RH_CHANNEL_LOCAL));

  // --------------- Initializing RH_Datagram -------------------- //
  if(!RHNetwork.init()){
    SerialUSB.println("ERR: 11 -> RHNetwork INIT failed. Did you assign the right pins?");
    while(1);
  }

  // --------------- Setting RH_Driver frequency -------------------- //

  if(!RHDriver.setFrequency(RH_DRIVER_FREQ)){
    SerialUSB.println("ERR: 12 -> RHDriver setFrequency failed. Check the connection with the radio chip.");
    while(1);
    //exit(12);
  }

  // --------------- Setting RH_Driver TxPower to 23 (maximum) -------------------- //

  RHDriver.setTxPower(23, false);

  // --------------- Setting #retries for RH_Datagram -------------------- //

  RHNetwork.setRetries(0);

  // --------------- Setting duration timeout for RH_Datagram -------------------- //

  RHNetwork.setTimeout(200);


  SerialUSB.println("Setup finished in " + String(millis()) + " milliseconds.");
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
    SerialUSB.print("0x");
    SerialUSB.print(FROM_ADDRESS, HEX);
    SerialUSB.print(": ");
    SerialUSB.println((char*) BUF);
    delay(1000);
    RHNetwork.sendtoWait(BUF, LEN, FROM_ADDRESS);
  }
}
