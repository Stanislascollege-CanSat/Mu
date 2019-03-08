//   ------------------------------------------------------------------------------   //
//  |                                 PROJECT BETA                                 |  //
//  |                                   Software                                   |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                                                              |  //
//  |                                   MOTHERCAN                                  |  //
//  |                                                                              |  //
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
#include <SPI.h>                      // SPI
#include <Wire.h>                     // I2C
#include <RH_RF95.h>                  // Radio
#include <RHReliableDatagram.h>       // Network
#include <Adafruit_Sensor.h>          // Shared Class
#include <Adafruit_SGP30.h>           // TVOC & eCO2
//#include <Adafruit_Si7021.h>          // Humidity & Airtemperature
#include <Adafruit_BMP280.h>          // Airtemperature & Airpressure
#include <Adafruit_PWMServoDriver.h>  // servo driver
#include <Adafruit_FRAM_I2C.h>        // FRAM
#include <Adafruit_GPS.h>             // GPS
#include <MPU9250.h>                  // IMU

// PIN DEFINITIONS
const unsigned short int PIN_RH_RST = 10;    //
const unsigned short int PIN_RH_CS = 12;     // RHDriver pins
const unsigned short int PIN_RH_INT = 6;     //

const unsigned short int PIN_BUZZ = 11;      // Buzzer
const unsigned short int PIN_A_BAT = 9;      // Battery voltage

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
//Adafruit_Si7021 Sensor_Si7021 = Adafruit_Si7021();
Adafruit_FRAM_I2C FRAMDisk = Adafruit_FRAM_I2C();
uint64_t FRAM_DATA_BEGIN_LOCATION = 10;
uint64_t FRAM_LAST_LOCATION = FRAM_DATA_BEGIN_LOCATION;
Adafruit_GPS Sensor_GPS(&Serial1);
MPU9250 Sensor_Motion(Wire, 0x68);


unsigned int startupTime;



// VARIABLES TO STORE THE NEWEST MEASUREMENTS
float BMP_temperature;
float BMP_airpressure;
float BMP_altitude;
float BMP_PREVIOUS_altitude;
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



bool DEPLOY_COMMAND_TRIGGERED;

int RINGSERVO_startRecord;
bool RINGSERVO_turning;
bool RINGSERVO_direction;
int RINGSERVO_angle_silent;
int RINGSERVO_speed;

int PINSERVO_startRecord;
bool PINSERVO_turning;
bool PINSERVO_direction;
int PINSERVO_angle_close;
int PINSERVO_angle_open;


//
// SETUP FUNCTION
//

void setup(){
  Serial.begin(115200);
  //while(!Serial);
  // --------------- Set pin and hull position -------------------- //
  pwm.begin();
  pwm.setPWMFreq(60); // Suitable frequency for most servo's.

  RINGSERVO_angle_silent = 78;
  RINGSERVO_speed = 50;
  PINSERVO_angle_close = 12;
  PINSERVO_angle_open = 45;

//  delay(300);
//  // close pins
  pwm.setPWM(SERVO_PINS, 0, map(PINSERVO_angle_close, 0, 180, SERVOMIN, SERVOMAX));
  delay(300);
  pwm.setPin(SERVO_PINS, 0);
//  // open hull
  pwm.setPWM(SERVO_HULL, 0, map(RINGSERVO_angle_silent + RINGSERVO_speed, 0, 180, SERVOMIN, SERVOMAX));
  delay(1000);
  pwm.setPWM(SERVO_HULL, 0, map(RINGSERVO_angle_silent, 0, 180, SERVOMIN, SERVOMAX));
  delay(200);
  pwm.setPin(SERVO_HULL, 0);

  RINGSERVO_turning = false;
  RINGSERVO_direction = false;
  DEPLOY_COMMAND_TRIGGERED = false;

  PINSERVO_turning = false;
  PINSERVO_direction = false;

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

  // fram disk
  if(!FRAMDisk.begin()){

  }

  // Sensor_BMP280
  if(!Sensor_BMP.begin()){

  }

  // Sensor_SGP30
  if(Sensor_SGP30.begin()){
    if(FRAMDisk.read8(0x0) == 1){
      uint16_t CO2_baseline = (FRAMDisk.read8(0x1) << 8) | (FRAMDisk.read8(0x2) & 0xff);
      uint16_t TVOC_baseline = (FRAMDisk.read8(0x3) << 8) | (FRAMDisk.read8(0x4) & 0xff);
      Sensor_SGP30.setIAQBaseline(CO2_baseline, TVOC_baseline);
    }
  }

  // Sensor_Si7021
//  if(!Sensor_Si7021.begin()){
//
//  }

  // Adafruit_GPS
  Sensor_GPS.begin(9600);
  Sensor_GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  Sensor_GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  Sensor_GPS.sendCommand(PGCMD_ANTENNA);

  // Sensor_MPU-9250 (IMU)
  if(Sensor_Motion.begin() < 0){

  }

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

// CALCULATE ABSOLUTE HUMIDITY
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}

//
// DEPLOY BABYCANS
//

void openRing(){
  // TURN THE RING OPEN
  if(!RINGSERVO_turning){
    //pwm.setPin(SERVO_HULL, 1);
    //delay(200);
    pwm.setPWM(SERVO_HULL, 0, map(RINGSERVO_angle_silent + RINGSERVO_speed, 0, 180, SERVOMIN, SERVOMAX));
    RINGSERVO_turning = true;
    RINGSERVO_direction = true;
    RINGSERVO_startRecord = millis();
  }
}

void closeRing(){
  if(!RINGSERVO_turning){
    //pwm.setPin(SERVO_HULL, 1);
    //delay(200);
    pwm.setPWM(SERVO_HULL, 0, map(RINGSERVO_angle_silent - RINGSERVO_speed, 0, 180, SERVOMIN, SERVOMAX));
    RINGSERVO_turning = true;
    RINGSERVO_direction = false;
    RINGSERVO_startRecord = millis();
  }
}

void stopRing(){
  pwm.setPWM(SERVO_HULL, 0, map(RINGSERVO_angle_silent, 0, 180, SERVOMIN, SERVOMAX));
}

void openPins(){
  //pwm.setPin(SERVO_PINS, 1);
  //delay(200);
  pwm.setPWM(SERVO_PINS, 0, map(PINSERVO_angle_open, 0, 180, SERVOMIN, SERVOMAX));
  PINSERVO_turning = true;
  PINSERVO_startRecord = millis();
}

void closePins(){
  //pwm.setPin(SERVO_PINS, 1);
  //delay(200);
  pwm.setPWM(SERVO_PINS, 0, map(PINSERVO_angle_close, 0, 180, SERVOMIN, SERVOMAX));
  PINSERVO_turning = true;
  PINSERVO_startRecord = millis();
}

void deployBabyCans(){
  openRing();
  DEPLOY_COMMAND_TRIGGERED = true;
}

void closeDeploy(){
  closePins();
  delay(500);
  closeRing();
}


//
// LOOP FUNCTION
//

void loop(){
  //Serial.print("LOOP");


  // RADIO RECEIVE COMMAND
//  uint8_t BUF[RH_RF95_MAX_MESSAGE_LEN] = "";
//  uint8_t LEN = sizeof(BUF);
//  uint8_t FROM_ADDRESS;
//  uint8_t TO_ADDRESS;
//
//
//  if(RHNetwork.recvfromAck(BUF, &LEN, &FROM_ADDRESS, &TO_ADDRESS)){
//    String reader = String((char*)BUF);
//    if(reader.equals("DEP")){
//      Serial.println("DEP called");
//      deployBabyCans();
//    }
//  }


  if(PINSERVO_turning){
    if(millis() - PINSERVO_startRecord > 400){
      PINSERVO_turning = false;
      pwm.setPin(SERVO_PINS, 0);
    }
  }


  if(RINGSERVO_turning && !DEPLOY_COMMAND_TRIGGERED){
    //Serial.println("stopping servo");
    if(millis() - RINGSERVO_startRecord > 1500){
      RINGSERVO_turning = false;
      pwm.setPin(SERVO_HULL, 0);
    }else if(millis() - RINGSERVO_startRecord > 1000){
      stopRing();
    }
  }

  if(DEPLOY_COMMAND_TRIGGERED){
    if(RINGSERVO_turning){
      if(millis() - RINGSERVO_startRecord > 1000){
        stopRing();
        RINGSERVO_turning = false;
      }
    }else{
      if(millis() - RINGSERVO_startRecord > 1500){
        DEPLOY_COMMAND_TRIGGERED = false;
        PINSERVO_turning = false;
        pwm.setPin(SERVO_HULL, 0);
        pwm.setPin(SERVO_PINS, 0);
      }else if(millis() - RINGSERVO_startRecord > 1100){
        openPins();
      }
    }
  }

  if(Serial.available()){
    String reader = Serial.readString();
    if(reader.equals("oP")){
      openPins();
    }else if(reader.equals("cP")){
      closePins();
    }else if(reader.equals("oR")){
      openRing();
    }else if(reader.equals("cR")){
      closeRing();
    }else if(reader.equals("sR")){
      stopRing();
    }else if(reader.equals("deploy")){
      deployBabyCans();
    }else if(reader.equals("close")){
      closeDeploy();
    }else{
      PINSERVO_angle_close = reader.toInt();
    }
  }

  
  
  Sensor_GPS.read();
  if (Sensor_GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    if (!Sensor_GPS.parse(Sensor_GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  if(int(millis()) % 500 == 0){
    // READING SENSORS
    Sensor_Motion.readSensor();




    if(Sensor_SGP30.IAQmeasure()){

    }

    // READING SENSOR DATA
    BMP_temperature = Sensor_BMP.readTemperature();
    BMP_airpressure = Sensor_BMP.readPressure();
    BMP_altitude = Sensor_BMP.readAltitude(992);
    //Si7021_humidity = Sensor_Si7021.readHumidity();
    //Si7021_temperature = Sensor_Si7021.readTemperature();
    // CALIBRATING SGP30
    //Sensor_SGP30.setHumidity(getAbsoluteHumidity(Si7021_humidity, Si7021_temperature));
    //
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
    dataPointRH += "TS:" + String(millis() - startupTime) + ";";
    dataPointRH += "GA:" + String(GPS_latitude, 4) + ";";
    dataPointRH += "GO:" + String(GPS_longitude, 4) + ";";
    dataPointRH += "GH:" + String(GPS_altitude) + ";";
    dataPointRH += "GS:" + String(GPS_speed) + ";";
    dataPointRH += "GV:" + String(GPS_angle) + ";";
    dataPointRH += "G3:" + String(GPS_fix) + ";";
    dataPointRH += "GN:" + String(GPS_satellites) + ";";
    dataPointRH += "AP:" + String(BMP_airpressure) + ";";
    dataPointRH += "AT:" + String(BMP_temperature) + ";";
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

    RHNetwork.sendtoWait((uint8_t*)dataPointRH.c_str(), dataPointRH.length(), RH_CHANNEL_GS_ALPHA);
    RHNetwork.waitPacketSent();

    dataPointRH = "{";
    dataPointRH += "GT:" + GPS_timestring + ";";
    dataPointRH += "TS:" + String(millis() - startupTime) + ";";
    dataPointRH += "GA:" + String(GPS_latitude, 4) + ";";
    dataPointRH += "GO:" + String(GPS_longitude, 4) + ";";
    dataPointRH += "GH:" + String(GPS_altitude) + ";";
    dataPointRH += "GS:" + String(GPS_speed) + ";";
    dataPointRH += "GV:" + String(GPS_angle) + ";";
    dataPointRH += "G3:" + String(GPS_fix) + ";";
    dataPointRH += "GN:" + String(GPS_satellites) + ";";
    dataPointRH += "AP:" + String(BMP_airpressure) + ";";
    dataPointRH += "AT:" + String(BMP_temperature) + ";";
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
    //dataPointRH += "BV:" + String(analogRead(PIN_A_BAT)*2*3.3/1024) + ";";
    dataPointRH += "}";

//    for(int i = 0; i < dataPointRH.length(); ++i){
//      if(FRAM_LAST_LOCATION < 255000 - FRAM_DATA_BEGIN_LOCATION){
//        FRAMDisk.write8(FRAM_LAST_LOCATION, (uint8_t)dataPointRH.charAt(i));
//        FRAM_LAST_LOCATION++;
//      }
//    }

//    Serial.println("Written " + String(int(FRAM_LAST_LOCATION)) + " bytes in " + String(millis() - startupTime) + " milliseconds");
//    Serial.println("GPS fix: " + String(GPS_fix));

    // RELEASE SAFETY PROTOCOL
    
    


    BMP_PREVIOUS_altitude = BMP_altitude;
  }


  // UPDATING BASELINES FOR CHIP: SGP30
  if(int(millis()) % 30000 == 0){
    uint16_t TVOC_baseline, CO2_baseline;
    if(Sensor_SGP30.getIAQBaseline(&CO2_baseline, &TVOC_baseline)){
      FRAMDisk.write8(0x0, 1);
      FRAMDisk.write8(0x1, (CO2_baseline >> 8));
      FRAMDisk.write8(0x2, (CO2_baseline & 0xff));
      FRAMDisk.write8(0x3, (TVOC_baseline >> 8));
      FRAMDisk.write8(0x4, (TVOC_baseline & 0xff));
//      Serial.println("Set baselines to " + String(CO2_baseline) + " and " + String(TVOC_baseline));
//      Serial.println(CO2_baseline);
//      Serial.println(TVOC_baseline);
    }else{
      FRAMDisk.write8(0x0, 0);
    }
  }

}
