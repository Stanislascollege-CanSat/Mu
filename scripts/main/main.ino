/****************************
  Mu MotherCan MCU software.
****************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GPS.h>

/******************
  Pin definitions
******************/

#define RF_RST 10
#define RF_CS 12
#define RF_INT 6
#define BUZZ 11
#define A_BAT 9
#define MCU_LED LED_BUILTIN

// Create singleton instances.
Adafruit_GPS GPS(&Serial1);

#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean);

void setup() {
  // Initialize MCU_LED as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Start up charm.
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

  Serial.begin(115200);

}
