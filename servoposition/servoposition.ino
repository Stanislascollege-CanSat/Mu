// INCLUDES
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x44);
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servo_pins = 0;
uint8_t servo_hull = 1;

void setup() {
  Serial.begin(115200);
  Serial.println("Find servo zero value");

  pwm.begin();

  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates

  delay(10);
}

void loop() {

  if(Serial.available()){
    String reader = Serial.readString();
    pwm.setPWM(servo_pins, 0, map(reader.toInt(), 0, 180, SERVOMIN, SERVOMAX));
    Serial.println("Set servo to "+reader);

    // if(reader.equals("openRing")){
    //   // open ring
    //   pwm.setPWM(servo_hull, 0, map(79 - 5, 0, 180, SERVOMIN, SERVOMAX));
    //   delay(1000);
    //   pwm.setPWM(servo_hull, 0, map(79, 0, 180, SERVOMIN, SERVOMAX));
    // }else if(reader.equals("closeRing")){
    //   // close ring
    //   pwm.setPWM(servo_hull, 0, map(79 + 5, 0, 180, SERVOMIN, SERVOMAX));
    //   delay(1000);
    //   pwm.setPWM(servo_hull, 0, map(79, 0, 180, SERVOMIN, SERVOMAX));
    // }else if(reader.equals("openPins")){
    //   // open pins
    //   pwm.setPWM(servo_pins, 0, map(26, 0, 180, SERVOMIN, SERVOMAX));
    // }else if(reader.equals("closePins")){
    //   // close pins
    //   pwm.setPWM(servo_pins, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    // }else{
    //   //pwm.setPWM(servo_pins, 0, map(reader.toInt(), 0, 180, SERVOMIN, SERVOMAX));
    // }
  }
}
