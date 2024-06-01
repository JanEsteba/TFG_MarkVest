#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>

TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 3); // RX, TX per al GPS
SoftwareSerial SerialBT(8, 9); // RX, TX per al mòdul Bluetooth

float latitudWaypoint = 41.349389; // Latitud del waypoint
float longitudWaypoint = 2.212583; // Longitud del waypoint
float lastDistance = 0; // Distància anterior al waypoint
unsigned long lastCheck = 0;
const long checkInterval = 200; // 5hz
const int distanceThreshold = 200;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  SerialBT.begin(9600); // Velocitat de comunicació del mòdul Bluetooth
}

void loop() {
  unsigned long now = millis();
  bool shouldCheck = now - lastCheck > checkInterval;
  if (shouldCheck && gpsSerial.available() > 0 && gps.encode(gpsSerial.read())) {
      float currentLat = gps.location.lat();
      float currentLon = gps.location.lng();
      float distance = getDistance(currentLat, currentLon, latitudWaypoint, longitudWaypoint);

      bool isCloseDistance = distance < distanceThreshold && lastDistance >= distanceThreshold;
      bool wasCloseDistance = distance >= distanceThreshold && lastDistance < distanceThreshold;
      if (
        isCloseDistance || 
        wasCloseDistance || 
        (distance < distanceThreshold && now % 10000 == 0) || 
        (distance >= distanceThreshold && now % 5000 == 0)
      ) {
        float bearing = getRelativeBearing(currentLat, currentLon, latitudWaypoint, longitudWaypoint);
        int motorIndex = (int)map(bearing, -180, 180, 0, 10); // Maps the value from the range [-180, 180] to [0,9]
        SerialBT.println(motorIndex); // Envia l'índex del motor a l'ESP32
      }

      lastDistance = distance;
    }
  }

// Util functions for degrees <-> radians transformation
double degreesToRadians(double degrees) {
    return degrees * (PI / 180);
}
double radiansToDegrees(double radians) {
    return radians * (180 / PI);
}

// Function to convert angle to half circle (-180 to +180 degrees)
double fullToHalfCircle(double angle) {
    const double fullCircle = 360;
    const double halfCircle = 180;

    double angleModFull = angle % fullCircle;
    if (angleModFull >= -halfCircle && angleModFull <= halfCircle)
        return angleModFull;
    if (angleModFull < -halfCircle) return halfCircle + angle % halfCircle;
    return -halfCircle + angle % halfCircle;
}

// Function to calculate the bearing between two points
double getRelativeBearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 = degreesToRadians(lat1);
    lon1 = degreesToRadians(lon1);
    lat2 = degreesToRadians(lat2);
    lon2 = degreesToRadians(lon2);

    double dLon = lon2 - lon1;

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double bearing = atan2(y, x);

    // Convert bearing from radians to degrees
    bearing = radiansToDegrees(atan2(y, x));

    return fullToHalfCircle(bearing);
}

// Function to calculate distance between two points using the Haversine formula
double getDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000; // Radius of Earth in meters

    double dLat = degreesToRadians(lat2 - lat1);
    double dLon = degreesToRadians(lon2 - lon1);

    lat1 = degreesToRadians(lat1);
    lat2 = degreesToRadians(lat2);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double distance = R * c;

    return distance;
}