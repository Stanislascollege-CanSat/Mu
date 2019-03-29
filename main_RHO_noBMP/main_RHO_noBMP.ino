//   ------------------------------------------------------------------------------   //
//  |                                 PROJECT BETA                                 |  //
//  |                                   Software                                   |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                   BABYCANS                                   |  //
//  |                                     BETA                                     |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                               Written by Rens Dur                            |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//   ------------------------------------------------------------------------------   //

// INCLUDES
#include <ArduinoSTL.h>               // Arduino STL
#include <SPI.h>                      // SPI
#include <Wire.h>                     // I2C
//#include <Adafruit_PWMServoDriver.h>  // servo driver
#include <RH_RF95.h>                  // Radio
#include <RHReliableDatagram.h>       // Network
#include <Adafruit_Sensor.h>          // Shared Class
#include <Adafruit_SGP30.h>           // TVOC & eCO2
#include <Adafruit_Si7021.h>          // Humidity & Airtemperature
#include <Adafruit_BMP280.h>          // Airtemperature & Airpressure
#include <Adafruit_FRAM_I2C.h>        // FRAM
#include <Adafruit_GPS.h>             // GPS

#include <MPU9250.h>                  // IMU

// FLIGHT MODE DEFINITION
enum APPMODE_SETTING {NON_FLIGHT_MODE = 0, FLIGHT_MODE = 1, LANDED_MODE = 2};
APPMODE_SETTING appMode;
bool SET_FLIGHT_MODE_ALLOWED;

// PIN DEFINITIONS
const unsigned short int PIN_RH_RST = 10;    //
const unsigned short int PIN_RH_CS = 12;     // RHDriver pins
const unsigned short int PIN_RH_INT = 6;     //

const unsigned short int PIN_BUZZ = 11;      // Buzzer
const unsigned short int PIN_A_BAT = 9;      // Battery voltage

const int PIN_MCU_LED = LED_BUILTIN;         // LED

// RADIO CHANNELS
const unsigned short int RH_CHANNEL_GS_ALPHA = 1;   //
const unsigned short int RH_CHANNEL_GS_DELTA = 2;   //
const unsigned short int RH_CHANNEL_MU = 3;         // Available radio-network-channels
const unsigned short int RH_CHANNEL_BETA = 4;       //
const unsigned short int RH_CHANNEL_RHO = 5;        //

const unsigned short int RH_CHANNEL_LOCAL = RH_CHANNEL_BETA; // Set local channel, used by the programme

const float RH_DRIVER_FREQ = 868.0;   // RHDriver Frequency

//// SERVO VARIABLES
//const int SERVOMIN = 150; // this is the 'minimum' pulse length count (out of 4096)
//const int SERVOMAX = 600; // this is the 'maximum' pulse length count (out of 4096)
//const unsigned short int SERVO_PINS = 0;
//const unsigned short int SERVO_HULL = 1;
//
//const int PINSERVO_angle_close = 0;    // Close value @50Hz 0
//const int PINSERVO_angle_open = 28;    // Open value @50Hz 28
//
//const int RINGSERVO_angle_silent = 55;
//const int RINGSERVO_speed = 5;
//
//bool DEPLOY_COMMAND_TRIGGERED;
//
//int RINGSERVO_startRecord;
//bool RINGSERVO_turning;
//bool RINGSERVO_direction;
//
//int PINSERVO_startRecord;
//bool PINSERVO_turning;
//bool PINSERVO_direction;

// RADIO DECLARATION
RH_RF95 RHDriver(PIN_RH_CS, PIN_RH_INT);
RHReliableDatagram RHNetwork(RHDriver, RH_CHANNEL_LOCAL);

// PWM DECLARATION
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x44);


// SENSOR OBJECT DECLARATION
Adafruit_BMP280 Sensor_BMP;
Adafruit_SGP30 Sensor_SGP30;
Adafruit_Si7021 Sensor_Si7021 = Adafruit_Si7021();
Adafruit_FRAM_I2C FRAMDisk = Adafruit_FRAM_I2C();
const uint16_t FRAM_DATA_BEGIN_LOCATION = 10;
uint16_t FRAM_LAST_LOCATION;
unsigned int FRAM_AMOUNT_MEASUREMENTS;
unsigned short int FRAM_STORE_RATIO;
bool FRAM_ALLOW_WRITING;
Adafruit_GPS Sensor_GPS(&Serial1);
MPU9250 Sensor_Motion(Wire, 0x68);

// TIME SINCE STARTUP
unsigned int startupTime;
unsigned int measureTime;
unsigned int loopTime;
unsigned int prevEndMeasureTime;
unsigned int listEndMeasureTime[] = {0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0};

// VARIABLES FOR CONFIG
int AIRPRESSURE_SEA_LEVEL;

// VARIABLES TO STORE THE NEWEST MEASUREMENTS
float BMP_temperature;
float BMP_airpressure;
float BMP_altitude;
float ALTITUDE_CORRECTION;
float BMP_PREVIOUS_altitudes[] = {0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0};
float CALC_verticalVelocity;
float CALC_prevVerticalVelocity[] = {0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0};
float Si7021_humidity;
float Si7021_temperature;
float SGP30_TVOC;
float SGP30_CO2;
float IMU_Acc_X;
float IMU_Acc_Y;
float IMU_Acc_Z;
float IMU_Gyro_X;
float IMU_Gyro_Y;
float IMU_Gyro_Z;
float IMU_Mag_X;
float IMU_Mag_Y;
float IMU_Mag_Z;
//float IMU_temperature;
int GPS_hour;
int GPS_minute;
int GPS_second;
int GPS_millisecond;
String GPS_timestring;
int GPS_day;
int GPS_month;
int GPS_year;
bool GPS_fix;
float GPS_latitude;
float GPS_longitude;
float GPS_speed;
float GPS_angle;
float GPS_altitude;
int GPS_satellites;

// LOOP READ DATA FROM SENSOR TIMER
unsigned int lastReadSensorTimeRecord;

// DEPLOYMENT SAFETY REQUIREMENT VARIABLES
bool DEPS_PASSED_HEIGHT_UP;
bool DEPS_DETECTED_PARABOLA;
bool DEPS_PASSED_HEIGHT_DOWN;
const int DEPS_BORDER_HEIGHT = 2;
const int DEPS_MINIMUM_HEIGHT = 1;
bool DEPS_DEPLOYED;
int prevCheckDeployTime;
int prevReadBaselinesTime;

// STATUS STRIGNS
String RH_INIT = "0";
String FRAM_INIT = "0";
String BMP_INIT = "0";
String SGP30_INIT = "0";
String SI7021_INIT = "0";
String GPS_INIT = "2";
String MPU9250_INIT = "0";

// DATA STRINGS
std::vector<String> STORED_DATA_POINTS;

bool RHreading = false;
String RHreadCommand = "";
std::vector<String> RHcommandList;

// --------------------------------------------------- //
// --------------- SETUP FUNCTION -------------------- //
// --------------------------------------------------- //

void setup(){

  appMode = NON_FLIGHT_MODE;
  SET_FLIGHT_MODE_ALLOWED = false;

  
  Serial.begin(115200);
  //while(!Serial);
  // --------------- Set pin and hull position -------------------- //
//  pwm.begin();
//  pwm.setPWMFreq(50); // Derived from MG90S datasheet.
//  delay(10);
//
//  // close pins
//  pwm.setPWM(SERVO_PINS, 0, map(PINSERVO_angle_close, 0, 180, SERVOMIN, SERVOMAX));
//  delay(300);
//  pwm.setPin(SERVO_PINS, 0);
//  // open hull
//  pwm.setPWM(SERVO_HULL, 0, map(RINGSERVO_angle_silent + RINGSERVO_speed, 0, 180, SERVOMIN, SERVOMAX));
//  delay(1000);
//  pwm.setPWM(SERVO_HULL, 0, map(RINGSERVO_angle_silent, 0, 180, SERVOMIN, SERVOMAX));
//  delay(300);
//  pwm.setPin(SERVO_HULL, 0);
//
//  RINGSERVO_turning = false;
//  RINGSERVO_direction = false;
//  DEPLOY_COMMAND_TRIGGERED = false;
//
//  PINSERVO_turning = false;
//  PINSERVO_direction = false;

  // DECLARING PINS
  pinMode(PIN_RH_RST, OUTPUT);
  digitalWrite(PIN_RH_RST, HIGH);

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
  // manual reset
  digitalWrite(PIN_RH_RST, LOW);
  delay(10);
  digitalWrite(PIN_RH_RST, HIGH);
  delay(10);

  if(!RHNetwork.init()){
    RH_INIT = "0";
  } else {
    RH_INIT = "2";
  }

  // --------------- Setting RH_Driver frequency -------------------- //
  if(!RHDriver.setFrequency(RH_DRIVER_FREQ)){
    while(1);
  }

  // --------------- Setting RH_Driver TxPower to 23 (maximum) -------------------- //
  RHDriver.setTxPower(23, false);

  // --------------- Setting #retries for RH_Datagram -------------------- //
  RHNetwork.setRetries(0);

  // --------------- Setting duration timeout for RH_Datagram -------------------- //
  RHNetwork.setTimeout(0);

  // --------------- INITIALIZING SENSORS ------------------------ //

  delay(1000);
  String sendStartBoot = "{CAN:" + String(RH_CHANNEL_LOCAL) + ";SBT:1;}";
  RHNetwork.sendtoWait((uint8_t*)sendStartBoot.c_str(), sendStartBoot.length(), RH_CHANNEL_GS_DELTA);
  RHNetwork.sendtoWait((uint8_t*)sendStartBoot.c_str(), sendStartBoot.length(), RH_CHANNEL_GS_ALPHA);
  RHNetwork.waitPacketSent();
  delay(2000);

  // FRAM disk
  if(FRAMDisk.begin()){
    FRAM_INIT = "2";
  }

  uint16_t loc = 10;
  FRAMDisk.write8(0x5, (loc >> 8));
  FRAMDisk.write8(0x6, (loc & 0xff));


  FRAM_LAST_LOCATION = ((FRAMDisk.read8(0x5) << 8) | (FRAMDisk.read8(0x6) & 0xff));
  if(FRAM_LAST_LOCATION < FRAM_DATA_BEGIN_LOCATION){
    FRAM_LAST_LOCATION = FRAM_DATA_BEGIN_LOCATION;
  }else if(FRAM_LAST_LOCATION > FRAM_DATA_BEGIN_LOCATION){
    if(FRAM_LAST_LOCATION <= 255000){
      FRAMDisk.write8(FRAM_LAST_LOCATION, (uint8_t)'$');
      FRAM_LAST_LOCATION++;
    }
  }
  FRAM_AMOUNT_MEASUREMENTS = 0;
  FRAM_STORE_RATIO = 1;
  FRAM_ALLOW_WRITING = false;

  // Sensor_BMP280
  if(Sensor_BMP.begin()){
    BMP_INIT = "2";
  }

  // Sensor_SGP30
  if(Sensor_SGP30.begin()){
    if(FRAMDisk.read8(0x0) == 1){
      uint16_t CO2_baseline = (FRAMDisk.read8(0x1) << 8) | (FRAMDisk.read8(0x2) & 0xff);
      uint16_t TVOC_baseline = (FRAMDisk.read8(0x3) << 8) | (FRAMDisk.read8(0x4) & 0xff);
      Sensor_SGP30.setIAQBaseline(CO2_baseline, TVOC_baseline);
    }
    SGP30_INIT = "2";
  }

  // Sensor_Si7021
  if(Sensor_Si7021.begin()){
    SI7021_INIT = "2";
  }

  // Adafruit_GPS
  Sensor_GPS.begin(9600);
  Sensor_GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  Sensor_GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  Sensor_GPS.sendCommand(PGCMD_ANTENNA);

  // Sensor_MPU-9250 (IMU)
  int IMU_STATUS;
  IMU_STATUS = Sensor_Motion.begin();
  if (IMU_STATUS) {
    MPU9250_INIT = "2";
  }
  //if(Sensor_Motion.begin() < 0){
  //  MPU9250_INIT = "2";
  //}

  // --------------- Set DEPS variables ----------------- //
  DEPS_PASSED_HEIGHT_UP = false;
  DEPS_DETECTED_PARABOLA = false;
  DEPS_PASSED_HEIGHT_DOWN = false;
  DEPS_DEPLOYED = false;
  prevEndMeasureTime = 0;
  lastReadSensorTimeRecord = 0;
//  BMP_PREVIOUS_altitudes = {};
//  CALC_prevVerticalVelocity = {};

//  for(int i = 0; i < sizeof(BMP_PREVIOUS_altitudes); ++i){
//    BMP_PREVIOUS_altitudes[i] = 0;
//  }
//
//  for(int i = 0; i < sizeof(CALC_prevVerticalVelocity); ++i){
//    CALC_prevVerticalVelocity[i] = 0;
//  }

  delay(2000);

  AIRPRESSURE_SEA_LEVEL = 1013;

  ALTITUDE_CORRECTION = 0;
  int nMeasurements = 10;
  for(int i = 1; i <= nMeasurements; ++i){
    ALTITUDE_CORRECTION += Sensor_BMP.readAltitude(AIRPRESSURE_SEA_LEVEL);
    delay(100);
  }
  ALTITUDE_CORRECTION /= -1*nMeasurements;

  // --------------- Confirming boot -------------------- //

  String confirmBoot = "{CAN:" + String(RH_CHANNEL_LOCAL) + ";SBT:2;";
  confirmBoot += "SMR:" + RH_INIT + ";";
  confirmBoot += "SMF:" + FRAM_INIT + ";";
  confirmBoot += "SMB:" + BMP_INIT + ";";
  confirmBoot += "SMC:" + SGP30_INIT + ";";
  confirmBoot += "SMS:" + SI7021_INIT + ";";
  confirmBoot += "SMG:" + GPS_INIT + ";";
  confirmBoot += "SMR:" + MPU9250_INIT + ";";
  confirmBoot += "SFM:" + String(appMode) + ";";
  confirmBoot += "SDP:0;";
  confirmBoot += "F:LOG,ALTITUDE_CORRECTION@" + String(ALTITUDE_CORRECTION) + ";";
  confirmBoot += "}";
  RHNetwork.sendtoWait((uint8_t*)confirmBoot.c_str(), confirmBoot.length(), RH_CHANNEL_GS_DELTA);
  RHNetwork.sendtoWait((uint8_t*)confirmBoot.c_str(), confirmBoot.length(), RH_CHANNEL_GS_ALPHA);
  RHNetwork.waitPacketSent();

  // --------------- Confirming setup charm -------------------- //
  for(int i = 100; i < 4000; i += 5){
    tone(PIN_BUZZ, i);
    delay(2);
  }
  noTone(PIN_BUZZ);

  startupTime = millis();
  measureTime = millis();
  prevCheckDeployTime = millis();
  prevReadBaselinesTime = millis();
}

// CALCULATE ABSOLUTE HUMIDITY
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}

//
// DEPLOY BABYCANS
//

//void openRing(){
//  // TURN THE RING OPEN
//  if(!RINGSERVO_turning){
//    //pwm.setPin(SERVO_HULL, 1);
//    //delay(200);
//    pwm.setPWM(SERVO_HULL, 0, map(RINGSERVO_angle_silent - RINGSERVO_speed, 0, 180, SERVOMIN, SERVOMAX));
//    RINGSERVO_turning = true;
//    RINGSERVO_direction = true;
//    RINGSERVO_startRecord = millis();
//  }
//}
//
//void closeRing(){
//  if(!RINGSERVO_turning){
//    //pwm.setPin(SERVO_HULL, 1);
//    //delay(200);
//    pwm.setPWM(SERVO_HULL, 0, map(RINGSERVO_angle_silent + RINGSERVO_speed, 0, 180, SERVOMIN, SERVOMAX));
//    RINGSERVO_turning = true;
//    RINGSERVO_direction = false;
//    RINGSERVO_startRecord = millis();
//  }
//}
//
//void stopRing(){
//  pwm.setPWM(SERVO_HULL, 0, map(RINGSERVO_angle_silent, 0, 180, SERVOMIN, SERVOMAX));
//}
//
//void openPins(){
//  //pwm.setPin(SERVO_PINS, 1);
//  //delay(200);
//  pwm.setPWM(SERVO_PINS, 0, map(PINSERVO_angle_open, 0, 180, SERVOMIN, SERVOMAX));
//  PINSERVO_turning = true;
//  PINSERVO_startRecord = millis();
//}
//
//void closePins(){
//  //pwm.setPin(SERVO_PINS, 1);
//  //delay(200);
//  pwm.setPWM(SERVO_PINS, 0, map(PINSERVO_angle_close, 0, 180, SERVOMIN, SERVOMAX));
//  PINSERVO_turning = true;
//  PINSERVO_startRecord = millis();
//}
//
//void deployBabyCans(){
//  openRing();
//  DEPLOY_COMMAND_TRIGGERED = true;
//}
//
//void closeDeploy(){
//  closePins();
//  delay(500);
//  closeRing();
//}

void consoleLogGS(String s){
  String toSend = "{CAN:" + String(RH_CHANNEL_LOCAL) + ";F:LOG," + s + ";}";
  RHNetwork.sendtoWait((uint8_t*)toSend.c_str(), toSend.length(), RH_CHANNEL_GS_DELTA);
  RHNetwork.sendtoWait((uint8_t*)toSend.c_str(), toSend.length(), RH_CHANNEL_GS_ALPHA);
  RHNetwork.waitPacketSent();
}

void logStatus(String stat, int val){
  String toSend = "{CAN:" + String(RH_CHANNEL_LOCAL) + ";" + stat + ":" + String(val) + ";}";
  RHNetwork.sendtoWait((uint8_t*)toSend.c_str(), toSend.length(), RH_CHANNEL_GS_DELTA);
  RHNetwork.sendtoWait((uint8_t*)toSend.c_str(), toSend.length(), RH_CHANNEL_GS_ALPHA);
  RHNetwork.waitPacketSent();
}

void logStatusFlightMode(){
  if(appMode == NON_FLIGHT_MODE){
    logStatus("SFM", 0);
  }else if(appMode == FLIGHT_MODE){
    logStatus("SFM", 1);
  }else if(appMode == LANDED_MODE){
    logStatus("SFM", 2);
  }
}

//void askDeployPermissionGS(){
//  String toSend = "{CAN:" + String(RH_CHANNEL_LOCAL) + ";F:ASK;}";
//  RHNetwork.sendtoWait((uint8_t*)toSend.c_str(), toSend.length(), RH_CHANNEL_GS_DELTA);
//  RHNetwork.sendtoWait((uint8_t*)toSend.c_str(), toSend.length(), RH_CHANNEL_GS_ALPHA);
//  RHNetwork.waitPacketSent();
//}

void fram_writeLastPosition(uint16_t a){
  FRAMDisk.write8(0x5, (a >> 8));
  FRAMDisk.write8(0x6, (a & 0xff));
}

uint16_t fram_readLastPosition(){
  return ((FRAMDisk.read8(0x5) << 8) | (FRAMDisk.read8(0x6) & 0xff));
}

void sendAllDataToGS(){
  uint16_t readPos = FRAM_DATA_BEGIN_LOCATION;
  String sendBuffer = "";
  FRAM_LAST_LOCATION = fram_readLastPosition();
  while(readPos <= FRAM_LAST_LOCATION){
    sendBuffer = "";
    for(int i = 0; i < 5 && readPos <= FRAM_LAST_LOCATION; ++i){
      sendBuffer += (char)FRAMDisk.read8(readPos);
      readPos++;
    }
    Serial.print(sendBuffer);
    RHNetwork.sendtoWait((uint8_t*)sendBuffer.c_str(), sendBuffer.length(), RH_CHANNEL_GS_DELTA);
    RHNetwork.sendtoWait((uint8_t*)sendBuffer.c_str(), sendBuffer.length(), RH_CHANNEL_GS_ALPHA);
    RHNetwork.waitPacketSent();
  }
}

void clearFRAMDisk(){
  FRAM_LAST_LOCATION = FRAM_DATA_BEGIN_LOCATION;
  fram_writeLastPosition(FRAM_LAST_LOCATION);
}

void receiveScriptFromRadio(){
  uint8_t BUF[RH_RF95_MAX_MESSAGE_LEN] = "";
  uint8_t LEN = sizeof(BUF);
  uint8_t FROM_ADDRESS;
  uint8_t TO_ADDRESS;


  if(RHNetwork.recvfromAck(BUF, &LEN, &FROM_ADDRESS, &TO_ADDRESS)){
    String reader = String((char*)BUF);
    for(int i = 0; i < reader.length(); ++i){
      switch(reader.charAt(i)){
        case '[':
          RHreadCommand = "";
          RHreading = true;
          break;
        case ']':
          RHreading = false;
          RHcommandList.push_back(RHreadCommand);
          RHreadCommand = "";
          break;
        default:
          if(RHreading){
            RHreadCommand += reader.charAt(i);
          }
          break;
      }
    }

    for(String s : RHcommandList){
      /*if(s.equals("DEP")){
        deployBabyCans();
        DEPS_DEPLOYED = true;
        Serial.println("-------------------------DEPLOY!");
        logStatus("SDP", 2);
      }else if(s.equals("OPR")){
        openRing();
      }else if(s.equals("CLR")){
        closeRing();
      }else if(s.equals("OPP")){
        openPins();
      }else if(s.equals("CLP")){
        closePins();
      }else if(s.equals("SAD")){
        sendAllDataToGS();
      }else */if(s.equals("STB")){
        noTone(PIN_BUZZ);
      }else if(s.equals("FLIGHT_MODE")){
        if(SET_FLIGHT_MODE_ALLOWED){
          appMode = FLIGHT_MODE;
//          String toSend = "[FLIGHT_MODE]";
//          RHNetwork.sendtoWait((uint8_t*)toSend.c_str(), toSend.length(), RH_CHANNEL_BETA);
//          RHNetwork.sendtoWait((uint8_t*)toSend.c_str(), toSend.length(), RH_CHANNEL_RHO);
//          RHNetwork.waitPacketSent();
          delay(5000);
          logStatusFlightMode();
          for(int i = 0; i < 3; ++i){
            delay(100);
            tone(PIN_BUZZ, 5000 - i*1000);
            delay(100);
            noTone(PIN_BUZZ);
          }
          measureTime = millis();
        }else{
          consoleLogGS("You must wait until the scanning is finished");
        }
      }
    }
    RHcommandList.clear();
  }
}

//
// LOOP FUNCTION
//

void loop(){

  if((appMode == NON_FLIGHT_MODE) && (millis() - startupTime >= 12000) && (!SET_FLIGHT_MODE_ALLOWED)){
    for(int i = 0; i < 3; ++i){
      delay(100);
      tone(PIN_BUZZ, 4000);
      delay(100);
      noTone(PIN_BUZZ);
    }
    SET_FLIGHT_MODE_ALLOWED = true;
    consoleLogGS("Flightmode unlocked");
  }

  
  loopTime = millis();
  //Serial.println("LOOP");

  // RADIO RECEIVE COMMAND
  receiveScriptFromRadio();
  
  

//  if(PINSERVO_turning){
//    if(millis() - PINSERVO_startRecord > 400){
//      PINSERVO_turning = false;
//      pwm.setPin(SERVO_PINS, 0);
//    }
//  }
//
//  if(RINGSERVO_turning && !DEPLOY_COMMAND_TRIGGERED){
//    //Serial.println("stopping servo");
//    if(millis() - RINGSERVO_startRecord > 1000){
//      stopRing();
//      if(millis() - RINGSERVO_startRecord > 2000){
//        RINGSERVO_turning = false;
//        pwm.setPin(SERVO_HULL, 0);
//      }
//    }
//  }
//
//  if(DEPLOY_COMMAND_TRIGGERED){
//    if(RINGSERVO_turning){
//      if(millis() - RINGSERVO_startRecord > 1000){
//        stopRing();
//        RINGSERVO_turning = false;
//      }
//    }else{
//      if(millis() - RINGSERVO_startRecord > 1500){
//        openPins();
//        if(millis() - RINGSERVO_startRecord > 2500){
//          DEPLOY_COMMAND_TRIGGERED = false;
//          PINSERVO_turning = false;
//          pwm.setPin(SERVO_HULL, 0);
//          pwm.setPin(SERVO_PINS, 0);
//        }
//      }
//    }
//  }

  if(Serial.available()){
    String reader = Serial.readString();
    /*if(reader.equals("[OPP]")){
      openPins();
    }else if(reader.equals("[CLP]")){
      closePins();
    }else if(reader.equals("[OPR]")){
      openRing();
    }else if(reader.equals("[CLR]")){
      closeRing();
    }else if(reader.equals("[STR]")){
      stopRing();
    }else if(reader.equals("[DEP]")){
      deployBabyCans();
      DEPS_DEPLOYED = true;
      Serial.println("-------------------------DEPLOY!");
      logStatus("SDP", 2);
    }else if(reader.equals("[CPR]")){
      closeDeploy();
    }else if(reader.equals("[RER]")){
      //removeRing();
      //PINSERVO_angle_close = reader.toInt();
    }else */if(reader.equals("[SAD]")){
      sendAllDataToGS();
    }else {
      Serial.print("{F:WRN,"+reader+" is not a command;}");
    }
  }

  //receiveScriptFromRadio();

  Sensor_GPS.read();
  if (Sensor_GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    if (!Sensor_GPS.parse(Sensor_GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  if(appMode == NON_FLIGHT_MODE && SET_FLIGHT_MODE_ALLOWED){
    BMP_altitude = Sensor_BMP.readAltitude(AIRPRESSURE_SEA_LEVEL) + ALTITUDE_CORRECTION;
    if(BMP_altitude >= DEPS_MINIMUM_HEIGHT){
      appMode = FLIGHT_MODE;
//      String toSend = "FLIGHT_MODE";
//      RHNetwork.sendtoWait((uint8_t*)toSend.c_str(), toSend.length(), RH_CHANNEL_BETA);
//      RHNetwork.sendtoWait((uint8_t*)toSend.c_str(), toSend.length(), RH_CHANNEL_RHO);
//      RHNetwork.waitPacketSent();
      delay(2000);
      logStatusFlightMode();
      for(int i = 0; i < 3; ++i){
        delay(100);
        tone(PIN_BUZZ, 5000 - i*1000);
        delay(100);
        noTone(PIN_BUZZ);
      }
      measureTime = millis();
    }
    delay(100);
  }

  //receiveScriptFromRadio();

  

  

  if((millis() - lastReadSensorTimeRecord >= 1000) && (appMode == FLIGHT_MODE || appMode == LANDED_MODE)){
    // READING SENSORS
    Sensor_Motion.readSensor();
    if(Sensor_SGP30.IAQmeasure()) {}
    // READING SENSOR DATA
    BMP_temperature = Sensor_BMP.readTemperature();
    BMP_airpressure = Sensor_BMP.readPressure();
    BMP_altitude = Sensor_BMP.readAltitude(AIRPRESSURE_SEA_LEVEL) + ALTITUDE_CORRECTION;
    Si7021_humidity = Sensor_Si7021.readHumidity();
    Si7021_temperature = Sensor_Si7021.readTemperature();
    // CALIBRATING SGP30
    Sensor_SGP30.setHumidity(getAbsoluteHumidity(Si7021_humidity, Si7021_temperature));
    SGP30_TVOC = Sensor_SGP30.TVOC;
    SGP30_CO2 = Sensor_SGP30.eCO2;
    IMU_Acc_X = Sensor_Motion.getAccelX_mss();
    IMU_Acc_Y = Sensor_Motion.getAccelY_mss();
    IMU_Acc_Z = Sensor_Motion.getAccelZ_mss();
    IMU_Gyro_X = Sensor_Motion.getGyroX_rads();
    IMU_Gyro_Y = Sensor_Motion.getGyroY_rads();
    IMU_Gyro_Z = Sensor_Motion.getGyroZ_rads();
    IMU_Mag_X = Sensor_Motion.getMagX_uT();
    IMU_Mag_Y = Sensor_Motion.getMagY_uT();
    IMU_Mag_Z = Sensor_Motion.getMagZ_uT();
    //IMU_temperature;
    GPS_hour = Sensor_GPS.hour + 1.0;
    GPS_minute = Sensor_GPS.minute;
    GPS_second = Sensor_GPS.seconds;
    GPS_millisecond = Sensor_GPS.milliseconds;
    GPS_timestring = ((String(GPS_hour).length() < 2) ? "0" + String(int(GPS_hour)) : String(int(GPS_hour)));
    GPS_timestring += ((String(GPS_minute).length() < 2) ? "0" + String(int(GPS_minute)) : String(int(GPS_minute)));
    GPS_timestring += ((String(GPS_second).length() < 2) ? "0" + String(int(GPS_second)) : String(int(GPS_second)));
    GPS_day = Sensor_GPS.day;
    GPS_month = Sensor_GPS.month;
    GPS_year = Sensor_GPS.year;
    GPS_fix = Sensor_GPS.fix;
    GPS_latitude = (GPS_fix ? Sensor_GPS.latitudeDegrees : 0);
    GPS_longitude = (GPS_fix ? Sensor_GPS.longitudeDegrees : 0);
    GPS_speed = (GPS_fix ? Sensor_GPS.speed : 0);
    GPS_angle = (GPS_fix ? Sensor_GPS.angle : 0);
    GPS_altitude = (GPS_fix ? Sensor_GPS.altitude : 0);
    GPS_satellites = (GPS_fix ? Sensor_GPS.satellites : 0);

    FRAM_AMOUNT_MEASUREMENTS++;

    //receiveScriptFromRadio();

//    Serial.print("temp:"); Serial.println(BMP_temperature);
//    Serial.print("pres:"); Serial.println(BMP_airpressure);
//    Serial.print("alti:"); Serial.println(BMP_altitude);
//    Serial.print("humi:"); Serial.println(Si7021_humidity);
//    Serial.print("temp:"); Serial.println(Si7021_temperature);
//    Serial.print("tvoc:"); Serial.println(SGP30_TVOC);
//    Serial.print("eco2:"); Serial.println(SGP30_CO2);
//    Serial.print("accx:"); Serial.println(IMU_Acc_X);
//    Serial.print("accy:"); Serial.println(IMU_Acc_Y);
//    Serial.print("accz:"); Serial.println(IMU_Acc_Z);
//    Serial.print("gyrx:"); Serial.println(IMU_Gyro_X);
//    Serial.print("gyry:"); Serial.println(IMU_Gyro_Y);
//    Serial.print("gyrz:"); Serial.println(IMU_Gyro_Z);
//    Serial.print("comx:"); Serial.println(IMU_Mag_X);
//    Serial.print("comy:"); Serial.println(IMU_Mag_Y);
//    Serial.print("comz:"); Serial.println(IMU_Mag_Z);
//    Serial.print("hour:"); Serial.println(GPS_hour);
//    Serial.print("mins:"); Serial.println(GPS_minute);
//    Serial.print("secs:"); Serial.println(GPS_second);
//    Serial.print("mils:"); Serial.println(GPS_millisecond);
//    Serial.print("day :"); Serial.println(GPS_day);
//    Serial.print("mont:"); Serial.println(GPS_month);
//    Serial.print("year:"); Serial.println(GPS_year);
//    Serial.print("fix :"); Serial.println(GPS_fix);
//    Serial.print("lati:"); Serial.println(GPS_latitude, 4);
//    Serial.print("long:"); Serial.println(GPS_longitude, 4);
//    Serial.print("sped:"); Serial.println(GPS_speed);
//    Serial.print("angl:"); Serial.println(GPS_angle);
//    Serial.print("alti:"); Serial.println(GPS_altitude);
//    Serial.print("stli:"); Serial.println(GPS_satellites);


    // WORKING WITH THE DATA
    String dataPointRH = "{CAN:" + String(RH_CHANNEL_LOCAL) + ";";
    dataPointRH += "GT:" + GPS_timestring + ";";
    //dataPointRH += "GM:" + GPS_millisecond + ";";
    dataPointRH += "TS:" + String(millis() - measureTime) + ";";
    dataPointRH += "GA:" + String(GPS_latitude, 4) + ";";
    dataPointRH += "GO:" + String(GPS_longitude, 4) + ";";
    dataPointRH += "GH:" + String(GPS_altitude) + ";";
    dataPointRH += "GS:" + String(GPS_speed) + ";";
    dataPointRH += "GV:" + String(GPS_angle) + ";";
    dataPointRH += "G3:" + String(GPS_fix) + ";";
    dataPointRH += "GN:" + String(GPS_satellites) + ";";
    dataPointRH += "AP:" + String(BMP_airpressure) + ";";
    dataPointRH += "AT:" + String(BMP_temperature, 1) + ";";
    dataPointRH += "AL:" + String(BMP_altitude) + ";";
    dataPointRH += "HM:" + String(Si7021_humidity) + ";";
    dataPointRH += "AX:" + String(IMU_Acc_X) + ";";
    dataPointRH += "AY:" + String(IMU_Acc_Y) + ";";
    dataPointRH += "AZ:" + String(IMU_Acc_Z) + ";";
    dataPointRH += "GX:" + String(IMU_Gyro_X) + ";";
    dataPointRH += "GY:" + String(IMU_Gyro_Y) + ";";
    dataPointRH += "GZ:" + String(IMU_Gyro_Z) + ";";
    dataPointRH += "CX:" + String(IMU_Mag_X) + ";";
    dataPointRH += "CY:" + String(IMU_Mag_Y) + ";";
    dataPointRH += "CZ:" + String(IMU_Mag_Z) + ";";
    dataPointRH += "OC:" + String(SGP30_TVOC) + ";";
    dataPointRH += "O2:" + String(SGP30_CO2) + ";";
    dataPointRH += "BV:" + String(analogRead(PIN_A_BAT)*2*3.3/1024) + ";";
    dataPointRH += "}";

    //RHNetwork.sendtoWait((uint8_t*)dataPointRH.c_str(), dataPointRH.length(), RH_CHANNEL_GS_DELTA);
    RHNetwork.sendtoWait((uint8_t*)dataPointRH.c_str(), dataPointRH.length(), RH_CHANNEL_GS_ALPHA);
    RHNetwork.waitPacketSent();
    //delay(1000);

    //STORED_DATA_POINTS.push_back(dataPointRH);

    

    // STORING DATA IN FRAM
    if(FRAM_AMOUNT_MEASUREMENTS % FRAM_STORE_RATIO == 0){
      dataPointRH = "{CAN" + String(RH_CHANNEL_LOCAL);
      dataPointRH += "Ac" + String(IMU_Acc_X) + "," + String(IMU_Acc_Y) + "," + String(IMU_Acc_Z);
      dataPointRH += "Gy" + String(IMU_Gyro_X) + "," + String(IMU_Gyro_Y) + "," + String(IMU_Gyro_Z);
      dataPointRH += "Co" + String(IMU_Mag_X) + "," + String(IMU_Mag_Y) + "," + String(IMU_Mag_Z);
      dataPointRH += "Gp" + String(GPS_longitude, 4) + "," + String(GPS_latitude, 4);
      dataPointRH += "TS" + String(millis() - startupTime);
      dataPointRH += "GT" + GPS_timestring;
      dataPointRH += "AP" + String(BMP_airpressure);
      dataPointRH += "AT" + String(BMP_temperature, 1);
      dataPointRH += "AL" + String(BMP_altitude);
      dataPointRH += "HM" + String(Si7021_humidity);
      dataPointRH += "OC" + String(SGP30_TVOC);
      dataPointRH += "O2" + String(SGP30_CO2);
      dataPointRH += "GH" + String(GPS_altitude);
      dataPointRH += "GS" + String(GPS_speed);
      dataPointRH += "GV" + String(GPS_angle);
      dataPointRH += "G3" + String(GPS_fix);
      dataPointRH += "GN" + String(GPS_satellites);
      dataPointRH += "}";
  
      for(int i = 0; i < dataPointRH.length(); ++i){
        if(FRAM_LAST_LOCATION <= 255000){
          FRAMDisk.write8(FRAM_LAST_LOCATION, (uint8_t)dataPointRH[i]);
          FRAM_LAST_LOCATION++;
        }else{
          consoleLogGS("FRAMDisk is full");
        }
      }
  
      fram_writeLastPosition(FRAM_LAST_LOCATION);
    }

    receiveScriptFromRadio();
    

    lastReadSensorTimeRecord = int(millis());

  }



  
  //
  // RELEASE SAFETY PROTOCOL
  //
  
  //Serial.println(DEPS_DEPLOYED);
  if(((millis() - prevCheckDeployTime >= 100)) && (appMode == FLIGHT_MODE || appMode == LANDED_MODE)){
    BMP_altitude = Sensor_BMP.readAltitude(AIRPRESSURE_SEA_LEVEL) + ALTITUDE_CORRECTION;
    //Serial.println(BMP_altitude);

    if(millis() - startupTime > sizeof(BMP_PREVIOUS_altitudes)/sizeof(float) * 500){
      CALC_verticalVelocity = pow((BMP_altitude - BMP_PREVIOUS_altitudes[3])/(float(millis())/1000 - float(listEndMeasureTime[3])/1000), 3);
      //Serial.println(CALC_verticalVelocity);

      if(DEPS_PASSED_HEIGHT_DOWN && (BMP_altitude <= DEPS_MINIMUM_HEIGHT && (CALC_verticalVelocity >= -1 && CALC_verticalVelocity <= 1))){
        appMode = LANDED_MODE;
        tone(PIN_BUZZ, 4000);
      }

      for(int i = 0; i < sizeof(BMP_PREVIOUS_altitudes)/sizeof(float); ++i){
        if(BMP_PREVIOUS_altitudes[i] < DEPS_BORDER_HEIGHT && BMP_altitude > DEPS_BORDER_HEIGHT){
          // CanSat passed DEPS_BORDER_HEIGHT border with upwards velocity
          DEPS_PASSED_HEIGHT_UP = true;
          Serial.println("-------------------------PASSED UP!");
        }else if(BMP_PREVIOUS_altitudes[i] > DEPS_BORDER_HEIGHT && BMP_altitude < DEPS_BORDER_HEIGHT){
          // CanSat passed DEPS_BORDER_HEIGHT border with downwards velocity
          DEPS_PASSED_HEIGHT_DOWN = true;
          Serial.println("-------------------------PASSED DOWN!");
        }
      }

      for(int i = 0; i < sizeof(CALC_prevVerticalVelocity)/sizeof(float); ++i){
        if(CALC_prevVerticalVelocity[i] > 6 && CALC_verticalVelocity < -6){
          // Detected a sign switch in vertical velocity
          DEPS_DETECTED_PARABOLA = true;
        Serial.println("-------------------------PARABOLA!");
          //tone(PIN_BUZZ, 1000);
          break;
          //Serial.println("parabola!");
        }
      }

//      if((((DEPS_PASSED_HEIGHT_UP || DEPS_DETECTED_PARABOLA) && DEPS_PASSED_HEIGHT_DOWN) && (BMP_altitude >= DEPS_MINIMUM_HEIGHT) && !DEPS_DEPLOYED)){
//        deployBabyCans();
//        DEPS_DEPLOYED = true;
//        Serial.println("-------------------------DEPLOY!");
//        logStatus("SDP", 2);
//      }else if(DEPS_PASSED_HEIGHT_DOWN){
//        askDeployPermissionGS();
//      }



      for(int i = sizeof(CALC_prevVerticalVelocity)/sizeof(float) - 1; i > 0; --i){
        CALC_prevVerticalVelocity[i] = CALC_prevVerticalVelocity[i-1];
      }
      CALC_prevVerticalVelocity[0] = CALC_verticalVelocity;
    }



    for(int i = sizeof(BMP_PREVIOUS_altitudes)/sizeof(float) - 1; i > 0; --i){
      BMP_PREVIOUS_altitudes[i] = BMP_PREVIOUS_altitudes[i-1];
    }
    BMP_PREVIOUS_altitudes[0] = BMP_altitude;

    prevEndMeasureTime = millis();

    for(int i = sizeof(listEndMeasureTime)/sizeof(unsigned int) - 1; i > 0; --i){
      listEndMeasureTime[i] = listEndMeasureTime[i-1];
    }
    listEndMeasureTime[0] = prevEndMeasureTime;

    prevCheckDeployTime = millis();
  }

  //receiveScriptFromRadio();

  // MEASURE TEMPERATURE FOR DEPLOY
//  if(int(millis()) % 100 == 0){
//    //Serial.print("{AT:"+String(Sensor_BMP.readTemperature(), 1)+";}");
//  }

  // UPDATING BASELINES FOR CHIP: SGP30
  if(millis() - prevReadBaselinesTime >= 30000){
    uint16_t TVOC_baseline, CO2_baseline;
    if(Sensor_SGP30.getIAQBaseline(&CO2_baseline, &TVOC_baseline)){
      FRAMDisk.write8(0x0, 1);
      FRAMDisk.write8(0x1, (CO2_baseline >> 8));
      FRAMDisk.write8(0x2, (CO2_baseline & 0xff));
      FRAMDisk.write8(0x3, (TVOC_baseline >> 8));
      FRAMDisk.write8(0x4, (TVOC_baseline & 0xff));
      // Inform GCS.
//      String messageBaseline = "{F:LOG,[SGP30] Baseline set.;}";
//      RHNetwork.sendtoWait((uint8_t*)messageBaseline.c_str(), messageBaseline.length(), RH_CHANNEL_GS_ALPHA);
//      RHNetwork.waitPacketSent();
//      Serial.println("Set baselines to " + String(CO2_baseline) + " and " + String(TVOC_baseline));
//      Serial.println(CO2_baseline);
//      Serial.println(TVOC_baseline);
    }else{
      FRAMDisk.write8(0x0, 0);
    }
    prevReadBaselinesTime = millis();
  }
}
