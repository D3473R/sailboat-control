float precision = 0.015;
float voltages[] = {3.84, 1.98, 2.25, 0.41, 0.45, 0.32, 0.9, 0.62, 1.4, 1.19, 3.08, 2.93, 4.62, 4.04, 4.33, 3.43};
float directions[] = {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5};

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  for (int i = 0; i < sizeof(voltages); i++) {
    if (abs(voltage - voltages[i]) < precision) {
      Serial.println(directions[i]);
      return;
    }
  }
  Serial.println(359);
}
