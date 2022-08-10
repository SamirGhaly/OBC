
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "TinyGPS++.h"
#include <SoftwareSerial.h>  
//wasal tx b tx 
  
TinyGPSPlus gps;

String command;

//SoftwareSerial GPSC(5, 6);
//SoftwareSerial HC12(2, 3); // HC-12 TX Pin, HC-12 RX Pin
MPU6050 mpu;

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

void setup() {
  Serial.begin(9600);
//  GPSC.begin(9600);

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
//  if (Serial.available()) {
//    command = Serial.readStringUntil('\n');
//    command.trim();
//    Serial.print("Command: ");
//    Serial.println(command);
//  }
//   if (command.equals("GPS") || command.equals("TEL")) {
//if (GPSC.available() && gps.encode(GPSC.read())){
//if (gps.location.isValid()){
//    Serial.print("\n");
//    Serial.print("GPS,"+String(gps.time.value())+ ',' + String(gps.location.lat()) + ',' + gps.location.lng() + ',' +(gps.location.rawLng().negative ? "S" : "N")+','+(gps.location.rawLat().negative ? "W" : "E") +',' + String(gps.altitude.meters()) + ',' + String(gps.satellites.value())); 
//Serial.print("\n");
//Serial.print("GPS,2301555,30.2437,31.4578,N,E,12,2");
//    //delay(1000);
//}
//    }
   //}
  //   if (command.equals("MPU") || command.equals("TEL")) {
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(Gyro, &q, &gravity);
            Serial.print("\n");
            Serial.print("GYRO," + String(Gyro[2] * 180/M_PI) + ','+ String(Gyro[1] * 180/M_PI) + ',' + (Gyro[0] * 180/M_PI));
    }
    //}
    }
