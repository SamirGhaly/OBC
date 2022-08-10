
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "TinyGPS++.h"

//#include <INA219.h>
//INA219 monitor;
//monitor.begin();

String command;
#include <SoftwareSerial.h>   

SoftwareSerial GPSC(5, 6);
SoftwareSerial HC12(2, 3); // HC-12 TX Pin, HC-12 RX Pin
MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  GPSC.begin(9600);
  
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float Gyro[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// join I2C bus (I2Cdev library doesn't do this automatically)
        Wire.begin();
        Wire.setClock(200000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        
    //MPU Initialization & Setting offsets    
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {}
    }


void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();

   if (command.equals("GPS")) {
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

    }
   }
    else if (command.equals("MPU")) {
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(Gyro, &q, &gravity);
            Serial.print("\n");
            Serial.print("GYRO,");
            Serial.print(Gyro[2] * 180/M_PI);
            Serial.print(",");
            Serial.print(Gyro[1] * 180/M_PI);
            Serial.print(",");
            Serial.println(Gyro[0] * 180/M_PI);
    }
    }

      else if (command.equals("SB")) {
        
  Serial.print("shunt voltage: ");
    Serial.print(monitor.shuntVoltage() * 1000, 4);
    Serial.println(" mV");
    
    Serial.print("shunt current: ");
    Serial.print(monitor.shuntCurrent() * 1000, 4);
    Serial.println(" mA");
    
    Serial.print("bus voltage:   ");
    Serial.print(monitor.busVoltage(), 4);
    Serial.println(" V");
    
    Serial.print("bus power:     ");
    Serial.print(monitor.busPower() * 1000, 4);
    Serial.println(" mW");
      }
    else if (command.equals("off")) {
      digitalWrite(greenLed, LOW);
      digitalWrite(redLed, LOW);
    }
    else {
      Serial.println("bad command");
    }
    Serial.print("Command: ");
    Serial.println(command);
  }
}
