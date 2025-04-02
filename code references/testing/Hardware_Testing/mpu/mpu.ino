// #include "Wire.h"
// #include <MPU6050_light.h>

// MPU6050 mpu(Wire);
// unsigned long timer = 0;

// void setup() {
//   Serial.begin(9600);
//   Wire.begin();
  
//   byte status = mpu.begin();
//   Serial.print(F("MPU6050 status: "));
//   Serial.println(status);
//   while(status!=0){ } // stop everything if could not connect to MPU6050
  
//   Serial.println(F("Calculating offsets, do not move MPU6050"));
//   delay(1000);
//   mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
//   mpu.calcOffsets(); // gyro and accelero
//   Serial.println("Done!\n");
// }

// void loop() {
//   mpu.update();
  
//   if((millis()-timer)>10){ // print data every 10ms
// 	Serial.print("X : ");
// 	Serial.print(mpu.getAngleX());
// 	Serial.print("\tY : ");
// 	Serial.print(mpu.getAngleY());
// 	Serial.print("\tZ : ");
// 	Serial.println(mpu.getAngleZ());
// 	timer = millis();  
//   }
// }

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
    Serial.println(F("Connection failed. Retrying..."));
    delay(1000);
  }
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // Calculate gyro and accelerometer offsets
  Serial.println(F("Done!\n"));

  // Print the calculated offsets
  Serial.println(F("Calculated Offsets:"));
  Serial.println("Calibration complete. Offsets:");
  Serial.print("Accelerometer Offsets: ");
  Serial.print("X: "); Serial.print(mpu.getAccXoffset()); 
  Serial.print(", Y: "); Serial.print(mpu.getAccYoffset()); 
  Serial.print(", Z: "); Serial.println(mpu.getAccZoffset());

  Serial.print("Gyroscope Offsets: ");
  Serial.print("X: "); Serial.print(mpu.getGyroXoffset()); 
  Serial.print(", Y: "); Serial.print(mpu.getGyroYoffset()); 
  Serial.print(", Z: "); Serial.println(mpu.getGyroZoffset());
}

void loop() {
  mpu.update();
  
  if ((millis() - timer) > 10) { // Print data every 10ms
    Serial.print("X : ");
    Serial.print(mpu.getAngleX());
    Serial.print("\tY : ");
    Serial.print(mpu.getAngleY());
    Serial.print("\tZ : ");
    Serial.println(mpu.getAngleZ());
    timer = millis();  
  }
}
