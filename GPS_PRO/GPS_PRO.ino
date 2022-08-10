#include "TinyGPS++.h"

#include <SoftwareSerial.h>   
TinyGPSPlus gps;
SoftwareSerial GPSC(5,6);

char GPS[6];
char UTC[10];
char Lat[11];
char NSInd[2];
char Long[12];
char EWInd[2];
char PosFix[2];
char Sat[3];
char hdop[5];
char Altitude[5];
float la, lo;
unsigned long age;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
    GPSC.begin(9600);


}

void loop() {
  // put your main code here, to run repeatedly:
  if (GPSC.available() && gps.encode(GPSC.read())){
//gps.f_get_position(&la, &lo, &age);
Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
Serial.print("ALT="); Serial.println(gps.altitude.meters());
Serial.print("time="); Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
Serial.print("hour="); Serial.println(gps.time.hour()); // Hour (0-23) (u8)
Serial.print("min="); Serial.println(gps.time.minute()); // Minute (0-59) (u8)
Serial.print("sec="); Serial.println(gps.time.second()); // Second (0-59) (u8)
Serial.print("lat="); Serial.println(gps.location.rawLat().negative ? "w" : "e");
Serial.print("long="); Serial.println(gps.location.rawLng().negative ? "s" : "n");
Serial.print("sat="); Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
//  while(Serial.read()=='$'){
//    Serial.readBytesUntil(',',GPS,6);
//    GPS[5] = '\0'; 
//    if (GPS[2]=='G' && GPS[3]=='G' && GPS[4]=='A'){
//      Serial.readBytesUntil(',',UTC, 10);
//      Serial.readBytesUntil(',',Lat, 11);
//      Serial.readBytesUntil(',',NSInd, 2);
//      Serial.readBytesUntil(',',Long, 12);
//      Serial.readBytesUntil(',',EWInd, 2);
//      Serial.readBytesUntil(',',PosFix, 2);
//      Serial.readBytesUntil(',',Sat, 3);
//      Serial.readBytesUntil(',',hdop, 5);
//      Serial.readBytesUntil(',',Altitude, 5);
//
//      UTC[9] = '\0';
//      Lat[10] = '\0';
//      NSInd[1] = '\0';
//      Long[11] = '\0';
//      EWInd[1] = '\0';
//      PosFix[1] = '\0';
//      Sat[2] = '\0';
//      hdop[4] = '\0';
//      Altitude[4] = '\0';
//
//      Serial.print("GPS,");
//      //Serial.print(GPS);
//
//      Serial.print(UTC);
//      Serial.print(",");
//
//
//      Serial.print(Lat);
//      Serial.print(",");
//      
//      Serial.print(Long);
//      Serial.print(",");
//      
//      Serial.print(NSInd);
//      Serial.print(",");
//
//      Serial.print(EWInd);
//      Serial.print(",");
//      
//      Serial.print(Altitude);
//      Serial.print(",");
//
//      Serial.print(PosFix);
//      Serial.print(",");
//
//      Serial.print(Sat);
//      Serial.print(",");
//    }
//    }
  }
}
