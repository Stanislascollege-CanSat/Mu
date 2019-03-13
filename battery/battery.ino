#define VBATPIN A7

void setup() {
Serial.begin(115200);
}

void loop() {
  float measuredvbat = analogRead(VBATPIN);
  float vbat = measuredvbat;
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 102.4; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  Serial.print(vbat);
}
