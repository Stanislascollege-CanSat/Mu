// INCLUDES
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <Adafruit_Sensor.h> // de rest
#include <Adafruit_SGP30.h> // gassensor
#include <Adafruit_Si7021.h> // humidity and airtemperature
#include <Adafruit_BMP280.h> // airtemperature / airpressure
#include <Adafruit_PWMServoDriver.h> // servo-driver
#include <Adafruit_FRAM_I2C.h> // fra
#include <Adafruit_GPS.h> // gps
#include <MPU9250.h> // gyroscoop, compas en accelerometer

// PIN DEFINITIONS
const unsigned short int PIN_RH_RST = 10;    //
const unsigned short int PIN_RH_CS = 12;     // Setting: RHDriver pins
const unsigned short int PIN_RH_INT = 6;    //
const unsigned short int PIN_BUZZ = 11;
const unsigned short int PIN_A_BAT = 9;

const int PIN_MCU_LED = LED_BUILTIN;

// RADIO CHANNELS
const unsigned short int RH_CHANNEL_GS_ALPHA = 1;   //
const unsigned short int RH_CHANNEL_GS_DELTA = 2;   //
const unsigned short int RH_CHANNEL_MU = 3;         // Available radio-network-channels
const unsigned short int RH_CHANNEL_BETA = 4;       //
const unsigned short int RH_CHANNEL_RHO = 5;        //

const unsigned short int RH_CHANNEL_LOCAL = RH_CHANNEL_MU; // Set local channel, used by the programme

const float RH_DRIVER_FREQ = 868.0;   // RHDriver Frequency

// SERVO VARIABLES
const int SERVOMIN = 150; // this is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 600; // this is the 'maximum' pulse length count (out of 4096)
const unsigned short int SERVO_PINS = 0;
const unsigned short int SERVO_HULL = 1;

// RADIO DECLARATION
RH_RF95 RHDriver(PIN_RH_CS, PIN_RH_INT);
RHReliableDatagram RHNetwork(RHDriver, RH_CHANNEL_LOCAL);

// PWM DECLARATION
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x44);


// SENSOR OBJECT DECLARATION
Adafruit_BMP280 Sensor_BMP;
Adafruit_SGP30 Sensor_SGP30;
Adafruit_Si7021 Sensor_Si7021 = Adafruit_Si7021();
Adafruit_FRAM_I2C FRAMDisk = Adafruit_FRAM_I2C();
uint64_t FRAM_LAST_LOCATION = 0;
Adafruit_GPS Sensor_GPS(&Serial1);


bool isSending;
unsigned int startupTime;

//
// SETUP FUNCTION
//

void setup(){
  isSending = true;
  // --------------- Set pin and hull position -------------------- //
  Serial.begin(115200);
  delay(2000);

  
  pwm.begin();

  pwm.setPWMFreq(60);

  // open hull
  pwm.setPWM(SERVO_HULL, 0, map(79 - 5, 0, 180, SERVOMIN, SERVOMAX));
  delay(1000);
  pwm.setPWM(SERVO_HULL, 0, map(79, 0, 180, SERVOMIN, SERVOMAX));
  delay(100);
  pwm.setPin(SERVO_HULL, 0);
  // close pins
  pwm.setPWM(SERVO_PINS, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
  delay(300);
  pwm.setPin(SERVO_PINS, 0);

  // --------------- Startup charm -------------------- //
  tone(PIN_BUZZ, 1000);
  digitalWrite(PIN_MCU_LED, HIGH);
  delay(500);
  noTone(PIN_BUZZ);
  digitalWrite(PIN_MCU_LED, LOW);
  delay(300);
  tone(PIN_BUZZ,1500);
  digitalWrite(PIN_MCU_LED, HIGH);
  delay(400);
  noTone(PIN_BUZZ);
  digitalWrite(PIN_MCU_LED, LOW);

  // --------------- Initializing RH_Datagram -------------------- //
  if(!RHNetwork.init()){
    while(1);
  }

  // --------------- Setting RH_Driver frequency -------------------- //

  if(!RHDriver.setFrequency(RH_DRIVER_FREQ)){
    while(1);
    //exit(12);
  }

  // --------------- Setting RH_Driver TxPower to 23 (maximum) -------------------- //

  RHDriver.setTxPower(23, false);

  // --------------- Setting #retries for RH_Datagram -------------------- //

  RHNetwork.setRetries(0);

  // --------------- Setting duration timeout for RH_Datagram -------------------- //

  RHNetwork.setTimeout(0);


  // --------------- INITIALIZING SENSORS ------------------------ //
  
  // Sensor_BMP
  if(!Sensor_BMP.begin()){
    
  }

  // Sensor_SGP30
  if(!Sensor_SGP30.begin()){
    
  }

  // Sensor_Si7021
  if(!Sensor_Si7021.begin()){
    
  }

  // fram disk
  if(!FRAMDisk.begin()){
    
  }

  // Adafruit_GPS
  Sensor_GPS.begin(9600);
  Sensor_GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  Sensor_GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  Sensor_GPS.sendCommand(PGCMD_ANTENNA);
  

  



  // --------------- Confirming boot -------------------- //
  
  String confirmBoot = "{CAN:" + String(RH_CHANNEL_LOCAL) + ";SBT:2;}";
  RHNetwork.sendtoWait((uint8_t*)confirmBoot.c_str(), confirmBoot.length(), RH_CHANNEL_GS_DELTA);
  for(int i = 100; i < 4000; i += 5){
    tone(PIN_BUZZ, i);
    delay(2);
  }
  noTone(PIN_BUZZ);

  startupTime = millis();
}

//
// LOOP FUNCTION
//

void loop(){
//  Sensor_BMP.readTemperature();
//  Sensor_BMP.readPressure();
//  Sensor_BMP.readAltitude(1013.25);
//
//  Sensor_Si7021.readHumidity();
//  Sensor_Si7021.readTemperature();

  uint8_t BUF[RH_RF95_MAX_MESSAGE_LEN] = "";
  uint8_t LEN = sizeof(BUF);
  uint8_t FROM_ADDRESS;
  uint8_t TO_ADDRESS;


  if(RHNetwork.recvfromAck(BUF, &LEN, &FROM_ADDRESS, &TO_ADDRESS)){
    String receive = String((char*)BUF);
    if(receive.equals("print")){
      isSending = false;
      String reader = "!!STORED IN FRAM!!";
      for(uint64_t i = 0; i < FRAM_LAST_LOCATION; ++i){
        reader += (char)FRAMDisk.read8(i);
      }
      delay(500);
      RHNetwork.sendtoWait((uint8_t*)reader.c_str(), reader.length(), RH_CHANNEL_GS_ALPHA);
    }
  }

  if(Serial.available()){
    if(Serial.readString().equals("print")){
//      String reader = "`";
//      for(uint64_t i = 0; i < FRAM_LAST_LOCATION; ++i){
//        reader += (char)FRAMDisk.read8(i);
//      }
      Serial.println(String((millis()-startupTime)/1000) + "(" + String(int(FRAM_LAST_LOCATION-1)) + " bytes)"/* + reader */);
    }
  }

  float temperature = Sensor_BMP.readTemperature();
  float pressure = Sensor_BMP.readPressure();
  float BMP_altitude = Sensor_BMP.readAltitude(1013.25);
  float humidity = Sensor_Si7021.readHumidity();


  String dataPointRH = "{CAN:" + String(RH_CHANNEL_LOCAL) + ";";

  dataPointRH += "AT:" + String(temperature) + ";";
  dataPointRH += "AP:" + String(pressure) + ";";
  dataPointRH += "AL:" + String(BMP_altitude) + ";";
  dataPointRH += "HM:" + String(humidity) + ";}";
  RHNetwork.sendtoWait((uint8_t*)dataPointRH.c_str(), dataPointRH.length(), RH_CHANNEL_GS_ALPHA);


  String dataPointFRAM = "{";
  
  dataPointFRAM += "AT" + String(temperature);
  dataPointFRAM += "AP" + String(pressure);
  dataPointFRAM += "AL" + String(BMP_altitude);
  dataPointFRAM += "HM" + String(humidity) + "}";

  for(int i = 0; i < 6; ++i){
    dataPointFRAM += dataPointFRAM;
  }


  for(int i = 0; i < dataPointFRAM.length(); ++i){
    if(FRAM_LAST_LOCATION < 255999){
      FRAMDisk.write8(FRAM_LAST_LOCATION, (uint8_t)dataPointFRAM.charAt(i));
      FRAM_LAST_LOCATION++;
    }
  }


  delay(500);
  
}
