#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define GSM_RX_PIN 7
#define GSM_TX_PIN 8
#define GPS_RX_PIN 3
#define GPS_TX_PIN 4


SoftwareSerial gsmSerial(GSM_RX_PIN, GSM_TX_PIN);
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);


TinyGPSPlus gps;


float policeStations[][2] = {
  {47.12345, -122.45678}, 
  {47.23456, -122.56789},

};


const char* phoneNumber = "+918767756944";
const char* smsMessage = "Nearest Police Station: ";

void setup() {
  
  Serial.begin(9600);
  gsmSerial.begin(9600); 
  gpsSerial.begin(9600); 
  delay(1000);

  
  gsmSerial.println("AT"); 
  delay(1000);
  gsmSerial.println("AT+CMGF=1"); 
  delay(1000);
}

void loop() {
  
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        
        float currentLat = gps.location.lat();
        float currentLng = gps.location.lng();

        
        Serial.print("Current Location: ");
        Serial.print(currentLat, 6);
        Serial.print(", ");
        Serial.println(currentLng, 6);

        
        int nearestStationIndex = findNearestPoliceStation(currentLat, currentLng);

        
        sendSMS(nearestStationIndex);
        delay(30000); 
      }
    }
  }
}
int findNearestPoliceStation(float currentLat, float currentLng) {
  float minDistance = 999999; 
  int nearestIndex = -1;
  for (int i = 0; i < sizeof(policeStations) / sizeof(policeStations[0]); i++) {
    float stationLat = policeStations[i][0];
    float stationLng = policeStations[i][1];
    float distance = calculateDistance(currentLat, currentLng, stationLat, stationLng);

    if (distance < minDistance) {
      minDistance = distance;
      nearestIndex = i;
    }
  }

  return nearestIndex;
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {

  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  float dlon = lon2 - lon1;
  float dlat = lat2 - lat1;
  float a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float radiusOfEarth = 6371000; 
  float distance = radiusOfEarth * c;

  return distance;
}

void sendSMS(int nearestIndex) {
 
  char smsBuffer[100];
  snprintf(smsBuffer, sizeof(smsBuffer), "%s %.6f, %.6f", smsMessage, policeStations[nearestIndex][0], policeStations[nearestIndex][1]);

  // Send SMS
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(phoneNumber);
  gsmSerial.println("\"");
  delay(1000);
  gsmSerial.println(smsBuffer);
  delay(100);
  gsmSerial.println((char)26); // End of SMS
  delay(1000);

  // Display SMS status
  Serial.println("SMS sent!");
}
