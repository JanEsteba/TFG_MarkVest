#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 3); // RX, TX per al GPS

#include <SoftwareSerial.h>
SoftwareSerial SerialBT(8, 9); // RX, TX per al mòdul Bluetooth

float latitudWaypoint = 41.349389; // Latitud del waypoint
float longitudWaypoint = 2.212583; // Longitud del waypoint
float lastDistance = 0; // Distància anterior al waypoint

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  SerialBT.begin(9600); // Velocitat de comunicació del mòdul Bluetooth
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      float currentLat = gps.location.lat();
      float currentLon = gps.location.lng();
      float distance = calcularDistancia(currentLat, currentLon, latitudWaypoint, longitudWaypoint);

      if ((distance < 200 && lastDistance >= 200) || (distance >= 200 && lastDistance < 200) || (distance < 200 && millis() % 10000 == 0) || (distance >= 200 && millis() % 5000 == 0)) {
        float rumbo = calcularRumbo(currentLat, currentLon, latitudWaypoint, longitudWaypoint);
        int motorIndex = mapCourseToMotor(rumbo);
        SerialBT.println(motorIndex); // Envia l'índex del motor a l'ESP32
      }

      lastDistance = distance;
    }
  }
}

float calcularRumbo(float latitudActual, float longitudActual, float latitudWaypoint, float longitudWaypoint) {
  // Codi per calcular el rumb cap al waypoint
}

float calcularDistancia(float latitudActual, float longitudActual, float latitudWaypoint, float longitudWaypoint) {
  // Codi per calcular la distància entre dues coordenades GPS
}

int mapCourseToMotor(float rumbo) {
  // Codi per assignar l'índex del motor segons el rumb
}
