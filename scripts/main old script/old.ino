/****************************
  Mu MotherCan MCU software.
****************************/

// LIBRARIES
#include <Wire.h>
#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GPS.h>

// PIN DEFINITIONS
const unsigned short int RF_RST = 10;
const unsigned short int RF_CS = 12;
const unsigned short int RF_INT = 6;
const unsigned short int BUZZ = 11;
const unsigned short int A_BAT = 9;

const string MCU_LED = LED_BUILTIN;

const float RF_FREQ = 868.0;

// OBJECT DECLARATION
RH_RF95 RF_Driver(RF_CS, RF_INT);
RHReliableDatagram RF_Datagram(RF_Driver, RF_ADDRESS_BETA);

// BUFFERS
String reader;


void setup() {
  // Initialize MCU_LED as an output.
  pinMode(LED_BUILTIN, OUTPUT);

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
  while(!Serial) {}
  delay(10);

  // --------------- Initializing RF_Datagram -------------------- //
  if(!RF_Datagram.init()){
    SerialUSB.print("RF_Datagram INIT failed (11)");
    while(1);
    //exit(11);
  }

}
