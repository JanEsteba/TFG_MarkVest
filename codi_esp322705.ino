
#include <SoftwareSerial.h>
SoftwareSerial SerialBT(8, 9); // RX, TX per al mòdul Bluetooth

const int motorPins[10] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18}; // Pins dels motors al torso

void setup() {
  Serial.begin(115200);
  SerialBT.begin(9600); // Velocitat de comunicació del mòdul Bluetooth
  
  for (int i = 0; i < 10; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
}

void loop() {
  if (SerialBT.available()) {
    int motorIndex = SerialBT.parseInt();
    activateMotor(motorIndex);
  }
}

void activateMotor(int motorIndex) {
  for (int i = 0; i < 10; i++) {
    if (i == motorIndex) {
      digitalWrite(motorPins[i], HIGH); // Activa el motor corresponent
    } else {
      digitalWrite(motorPins[i], LOW); // Apaga els altres motors
    }
  }
}
