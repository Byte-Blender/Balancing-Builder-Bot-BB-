#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize the MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 initialization failed with status code: ");
    Serial.println(status);
    while (1); // Stop if initialization fails
  }

  Serial.println("MPU6050 initialized successfully!");

  // Calibrate the MPU6050 to calculate offsets
  Serial.println("Calibrating MPU6050...");
  delay(1000); // Allow time for stabilization
  mpu.calcOffsets(); // Calculate offsets

  // Print calculated offsets
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
  // No need for repeated actions in the loop for offset calculation
}
